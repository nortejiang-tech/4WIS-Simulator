#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <cstdint>
#include <cstring>
#include "core.hpp"

namespace {
// Buffer views own a reference until release. Validate dtype, layout, alignment,
// exact length and mutability before exposing any pointer to the numerical core.
struct Buffer {
    Py_buffer view{};
    Buffer(PyObject* obj, Py_ssize_t count, bool writable=false) {
        const int flags=PyBUF_FORMAT|PyBUF_C_CONTIGUOUS|(writable?PyBUF_WRITABLE:0);
        if(PyObject_GetBuffer(obj,&view,flags)<0) throw std::invalid_argument("cannot acquire float64 buffer");
        if(view.itemsize!=sizeof(double) || !view.format || std::strcmp(view.format,"d")!=0 ||
           (count>=0 && view.len!=count*static_cast<Py_ssize_t>(sizeof(double))) ||
           view.len%sizeof(double)!=0 || reinterpret_cast<uintptr_t>(view.buf)%alignof(double)!=0) {
            PyBuffer_Release(&view);view={};
            throw std::invalid_argument("expected aligned contiguous native float64 buffer of exact size");
        }
        if(!writable) {
            const auto* data=static_cast<const double*>(view.buf);
            for(Py_ssize_t i=0;i<view.len/static_cast<Py_ssize_t>(sizeof(double));++i) {
                if(!std::isfinite(data[i])) {
                    PyBuffer_Release(&view);view={};
                    throw std::invalid_argument("native input must be finite");
                }
            }
        }
    }
    ~Buffer() {if(view.obj) PyBuffer_Release(&view);}
    Buffer(const Buffer&)=delete;
    double* data() const {return static_cast<double*>(view.buf);}
    size_t size() const {return view.len/sizeof(double);}
};
void nonoverlap(const Buffer& in,const Buffer& out) {
    const auto a=reinterpret_cast<uintptr_t>(in.view.buf), b=reinterpret_cast<uintptr_t>(out.view.buf);
    if(a<b+out.view.len && b<a+in.view.len) throw std::invalid_argument("native input/output buffers must not overlap");
}
void check_tire(const double* t) {
    if((t[0]!=0.0 && t[0]!=1.0) || t[1]<0 || t[2]<0 ||
       (t[0]==1.0 && (t[6]<=0 || t[7]<=0 || t[10]<=0)))
        throw std::invalid_argument("invalid native tyre coefficients");
}
void check_body(const double* p) {
    if(p[sim4wis::MASS]<=0 || p[sim4wis::IZ]<=0 || p[sim4wis::L]<=0 || p[sim4wis::RADIUS]<=0 || p[sim4wis::LIMIT]<0)
        throw std::invalid_argument("invalid native body parameters");
}
void check_output(const Buffer& out) {
    for(size_t i=0;i<out.size();++i) if(!std::isfinite(out.data()[i]))
        throw std::runtime_error("non-finite native physical state");
}
template<typename Fn> PyObject* guarded(Fn fn) {
    try {fn();Py_RETURN_NONE;}
    catch(const std::invalid_argument& e) {if(!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError,e.what());}
    catch(const std::bad_alloc&) {PyErr_NoMemory();}
    catch(const std::exception& e) {PyErr_SetString(PyExc_ArithmeticError,e.what());}
    catch(...) {PyErr_SetString(PyExc_RuntimeError,"unknown native core error");}
    return nullptr;
}
PyObject* integrate(PyObject*,PyObject* args) {
    int model;double dt;PyObject *po,*to,*fo,*so,*ko,*yo,*oo;
    if(!PyArg_ParseTuple(args,"idOOOOOOO",&model,&dt,&po,&to,&fo,&so,&ko,&yo,&oo)) return nullptr;
    return guarded([&] {
        if(model!=0 && model!=1) throw std::invalid_argument("unknown native body model");
        if(!std::isfinite(dt) || dt<=0) throw std::invalid_argument("dt must be finite and positive");
        const int n=model==0?3:24;
        Buffer p(po,sim4wis::NP),t(to,sim4wis::NT),f(fo,4*sim4wis::NF),
               s(so,model==0?0:sim4wis::NS),k(ko,-1),y(yo,n),out(oo,2*n,true);
        for(const auto* in:{&p,&t,&f,&s,&k,&y}) nonoverlap(*in,out);
        check_body(p.data());check_tire(t.data());
        // Compute in local memory. An error never leaves a partially written state.
        std::array<double,48> result{};
        if(model==0) sim4wis::rk4<3>(y.data(),dt,result.data(),[&](const double* v,double* d) {
            sim4wis::planar_rhs(p.data(),t.data(),f.data(),v,d);
        });
        else {
            if(s.data()[sim4wis::MU_S]<=0 || s.data()[sim4wis::MS]<=0 || s.data()[sim4wis::IROLL]<=0 || s.data()[sim4wis::IPITCH]<=0)
                throw std::invalid_argument("invalid native suspension masses/inertias");
            const sim4wis::KC kc(k.data(),k.size());
            sim4wis::rk4<24>(y.data(),dt,result.data(),[&](const double* v,double* d) {
                sim4wis::multibody_rhs(p.data(),t.data(),f.data(),s.data(),kc,v,d);
            });
        }
        for(int i=0;i<2*n;++i) if(!std::isfinite(result[i])) throw std::runtime_error("non-finite native physical state");
        std::copy(result.begin(),result.begin()+2*n,out.data());
    });
}
PyObject* spin(PyObject*,PyObject* args) {
    PyObject *to,*qo,*io,*oo;if(!PyArg_ParseTuple(args,"OOOO",&to,&qo,&io,&oo)) return nullptr;
    return guarded([&] {
        Buffer t(to,sim4wis::NT),q(qo,4),in(io,28),out(oo,20,true);
        for(const auto* b:{&t,&q,&in}) nonoverlap(*b,out);
        check_tire(t.data());
        if(q.data()[0]<0 || q.data()[1]<=0 || q.data()[3]<=0) throw std::invalid_argument("invalid wheel-spin settings");
        std::array<double,20> result{};sim4wis::spin(t.data(),q.data(),in.data(),result.data());
        for(double v:result) if(!std::isfinite(v)) throw std::runtime_error("non-finite native wheel state");
        std::copy(result.begin(),result.end(),out.data());
    });
}
PyObject* tires(PyObject*,PyObject* args) {
    PyObject *to,*io,*oo;if(!PyArg_ParseTuple(args,"OOO",&to,&io,&oo)) return nullptr;
    return guarded([&] {
        Buffer t(to,sim4wis::NT),in(io,16),out(oo,12,true);
        nonoverlap(t,out);nonoverlap(in,out);check_tire(t.data());
        for(int i=0;i<4;++i) {const double* v=in.data()+4*i;const auto f=sim4wis::tire(t.data(),v[0],v[1],v[2],v[3]);
            out.data()[3*i]=f.fx;out.data()[3*i+1]=f.fy;out.data()[3*i+2]=f.mz;}
        check_output(out);
    });
}
PyObject* linkage(PyObject*,PyObject* args) {
    PyObject *d,*g,*o;double eta,fallback;
    if(!PyArg_ParseTuple(args,"OOddO",&d,&g,&eta,&fallback,&o)) return nullptr;
    return guarded([&] {
        Buffer delta(d,4),geom(g,12),out(o,28,true);nonoverlap(delta,out);nonoverlap(geom,out);
        if(!std::isfinite(eta) || !std::isfinite(fallback)) throw std::invalid_argument("non-finite linkage setting");
        sim4wis::linkage(delta.data(),geom.data(),eta,fallback,out.data());
    });
}
PyMethodDef methods[]={
    {"integrate",integrate,METH_VARARGS,"Integrate one body step and accepted derivative."},
    {"spin",spin,METH_VARARGS,"Four nonlinear wheel-spin solves."},
    {"tires",tires,METH_VARARGS,"Four built-in combined-slip tyre evaluations."},
    {"linkage",linkage,METH_VARARGS,"Four planar steering linkage constraints."},
    {nullptr,nullptr,0,nullptr}
};
PyModuleDef module={PyModuleDef_HEAD_INIT,"_native",nullptr,-1,methods};
}
PyMODINIT_FUNC PyInit__native() {
    PyObject* m=PyModule_Create(&module);if(!m) return nullptr;
    if(PyModule_AddIntConstant(m,"ABI_VERSION",sim4wis::ABI)<0) {Py_DECREF(m);return nullptr;}
    if(PyModule_AddStringConstant(m,"SOURCE_SHA256",SIM4WIS_NATIVE_SOURCE_SHA256)<0) {Py_DECREF(m);return nullptr;}
    return m;
}
