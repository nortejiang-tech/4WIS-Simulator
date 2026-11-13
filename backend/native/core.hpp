// Native numerical blocks. Equations/order mirror the v0.105.0 Python oracle.
// No fast-math, FMA contraction, solver substitution or time-step enlargement.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace sim4wis {
constexpr int ABI = 1, NP = 19, NF = 15, NS = 20, NT = 11;
enum P { MASS, IZ, L, A, RADIUS, CRR, RHO, CD, AREA, CGAMMA, CAMBER,
         CALPHA, LOAD_ON, LOAD_EXP, MU_LOAD, RELAX, LIMIT, TF, TR };
enum F { X, Y, DELTA, MU, FZ, OMEGA, ALPHA_SCALE, ALPHA_LAG, EXTRA_CAMBER,
         TOE, ROAD, KC_FX, KC_FY, KC_FZ, KC_MZ };
enum S { M, MU_S, MS, H, KS, CS, KT, ARBF, ARBR, ARBB, IROLL, IPITCH,
         XSCG, QAERO, CLF, CLR, W0, W1, W2, W3 };
inline double clip(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }
struct Force { double fx, fy, mz; };

// tyre vector: kind(0 linear/1 Pacejka), C_alpha, C_kappa, trail, Fz_nom,
// exponent, Cx, Cy, Ex, Ey, alpha_sl. Preserve the Python combined-slip law.
inline Force tire(const double* t, double alpha, double kappa, double fz, double mu) {
    double ca = t[1];
    if (t[4] > 0.0 && t[5] > 0.0) ca *= std::pow(std::max(fz / t[4], 1e-3), t[5]);
    double fx = t[2] * kappa, fy = -ca * alpha;
    const double cap = std::max(0.0, mu * fz);
    double trail = t[3];
    if (t[0] == 0.0) {
        const double mag = std::hypot(fx, fy);
        if (mag > cap && mag > 1e-9) { const double q = cap / mag; fx *= q; fy *= q; }
    } else {
        if (cap == 0.0) { fx = 0.0; fy = 0.0; }
        else {
            const double xk = t[2] / (t[6] * cap) * kappa;
            const double xa = ca / (t[7] * cap) * alpha;
            fx = cap * std::sin(t[6] * std::atan(xk - t[8] * (xk - std::atan(xk))));
            fy = -cap * std::sin(t[7] * std::atan(xa - t[9] * (xa - std::atan(xa))));
            const double q = (fx / cap) * (fx / cap) + (fy / cap) * (fy / cap);
            if (q > 1.0) { const double scale = 1.0 / std::sqrt(q); fx *= scale; fy *= scale; }
        }
        trail *= std::max(0.0, 1.0 - std::abs(alpha) / t[10]);
    }
    return {fx, fy, -fy * trail};
}

inline double implicit_spin(const double* t, double omega, double vx, double alpha,
                            double fz, double mu, double torque, double dt,
                            double radius, double inertia, double denom) {
    const double cap = std::max(mu * fz, 0.0);
    if (cap == 0.0 || dt == 0.0) return omega + dt * torque / inertia;
    const auto residual = [&](double w) {
        const double fx = tire(t, alpha, (radius * w - vx) / denom, fz, mu).fx;
        return inertia * (w - omega) - dt * (torque - radius * fx);
    };
    const double tol = 1e-11 * std::max({inertia * std::abs(omega), dt * std::abs(torque), 1.0});
    if (std::abs(residual(omega)) <= tol) return omega;
    double lo = omega + dt * (torque - radius * cap) / inertia;
    double hi = omega + dt * (torque + radius * cap) / inertia;
    double w = clip(omega, lo, hi);
    for (int k = 0; k < 48; ++k) {
        const double f = residual(w);
        if (std::abs(f) <= tol) return w;
        if (f > 0.0) hi = w; else lo = w;
        if (hi - lo <= 1e-12 * std::max(std::abs(w), 1.0)) return 0.5 * (hi + lo);
        const double eps = 1e-5 * std::max(std::abs(w), 1.0);
        const double slope = (residual(w + eps) - residual(w - eps)) / (2.0 * eps);
        const double candidate = slope > 1e-12 ? w - f / slope : NAN;
        w = lo < candidate && candidate < hi ? candidate : 0.5 * (lo + hi);
    }
    throw std::runtime_error("wheel-spin implicit solve did not converge");
}

// in: four rows (omega,vx,alpha,fz,mu,torque,brake); out: (omega,Fx,Fy,Mz,kappa).
inline void spin(const double* t, const double* q, const double* in, double* out) {
    const double dt = q[0], r = q[1], iw = std::max(q[2], 1e-6), minv = q[3];
    for (int i = 0; i < 4; ++i) {
        const double* v = in + 7*i; double* o = out + 5*i;
        const double den = std::max(std::abs(v[1]), minv);
        double w = implicit_spin(t, v[0], v[1], v[2], v[3], v[4], v[5], dt, r, iw, den);
        if (v[6] != 0.0 && (v[0]*w < 0.0 || (std::abs(v[0]) <= 1e-3 && w*v[6] > 0.0))) w = 0.0;
        const double kappa = (r*w-v[1])/den;
        const auto f = tire(t, v[2], kappa, v[3], v[4]);
        o[0]=w; o[1]=f.fx; o[2]=f.fy; o[3]=f.mz; o[4]=kappa;
    }
}

inline std::array<double,3> planar(double mass, double iz, double e,
                                  double u, double v, double r, double fx, double fy, double mz) {
    const double rd = (mz-e*fy)/iz;
    return {fx/mass+r*v+e*r*r, fy/mass-r*u-e*rd, rd};
}
inline double resistance(const double* p, double u) {
    const double roll = p[CRR]*p[MASS]*9.80665*std::tanh(u/0.5);
    const double aero = 0.5*p[RHO]*p[CD]*p[AREA]*u*std::abs(u);
    return -(roll+aero);
}
inline Force body_force(const double* p, const double* t, const double* f,
                        double u, double v, double r, double omega,
                        double delta, double fz, double extra_camber, int i) {
    const double c=std::cos(delta), s=std::sin(delta);
    const double vx=u-r*f[Y], vy=v+r*f[X];
    const double vxw=c*vx+s*vy, vyw=-s*vx+c*vy;
    const double den=std::max(std::abs(vxw),0.5);
    double alpha=std::atan2(vyw,den);
    if (p[RELAX]>0.0) alpha=f[ALPHA_LAG]/std::max(f[ALPHA_SCALE],1e-9);
    double ca=p[CALPHA];
    if (p[LOAD_ON]!=0.0) ca *= std::pow(std::max(fz/std::max(p[MASS]*9.80665/4.0,1.0),1e-3),p[LOAD_EXP]);
    ca=std::max(ca,1.0);
    const double camber=((i%2==0)?p[CAMBER]:-p[CAMBER])+extra_camber;
    const double offset=-(p[CGAMMA]*camber*fz)/ca;
    double mu=f[MU];
    if (p[MU_LOAD]>0.0) mu=std::max(mu*(1.0-p[MU_LOAD]*(fz/std::max(p[MASS]*9.80665/4.0,1.0)-1.0)),0.05*mu);
    const auto force=tire(t, f[ALPHA_SCALE]*alpha+offset, (p[RADIUS]*omega-vxw)/den, fz, mu);
    return {c*force.fx-s*force.fy, s*force.fx+c*force.fy, force.mz};
}
inline void planar_rhs(const double* p, const double* t, const double* frozen,
                       const double* y, double* dy) {
    double fx=0.0,fy=0.0,mz=0.0;
    for (int i=0;i<4;++i) {
        const double* f=frozen+i*NF;
        const auto force=body_force(p,t,f,y[0],y[1],y[2],f[OMEGA],f[DELTA],f[FZ],f[EXTRA_CAMBER],i);
        fx+=force.fx; fy+=force.fy; mz+=f[X]*force.fy-f[Y]*force.fx+force.mz;
    }
    const double grade=std::atan2(0.5*(frozen[ROAD]+frozen[NF+ROAD])-0.5*(frozen[2*NF+ROAD]+frozen[3*NF+ROAD]),p[L]);
    fx += -p[MASS]*9.81*std::sin(grade);
    fx += resistance(p,y[0]);
    const auto d=planar(p[MASS],p[IZ],p[L]/2.0-p[A],y[0],y[1],y[2],fx,fy,mz);
    std::copy(d.begin(),d.end(),dy);
}

struct Curve { std::vector<double> x,y;
    double at(double v) const {
        if (x.size()<2) return 0.0;
        if (v<x.front()) return y.front();
        if (v>=x.back()) return y.back();
        const auto hi=std::upper_bound(x.begin(),x.end(),v)-x.begin();
        const auto lo=hi-1;
        return y[lo]+(y[hi]-y[lo])/(x[hi]-x[lo])*(v-x[lo]);
    }
};
struct KC {
    std::array<double,10> coeff{};
    std::array<Curve,4> curves; // front toe/camber, rear toe/camber
    KC(const double* a, size_t n) {
        if(n<14) throw std::invalid_argument("invalid K&C buffer");
        std::copy(a,a+10,coeff.begin()); size_t at=10;
        for(auto& curve:curves) {
            if(at>=n || a[at]<0 || a[at]>100000 || std::floor(a[at])!=a[at]) throw std::invalid_argument("invalid K&C count");
            size_t count=static_cast<size_t>(a[at++]);
            if(count>(n-at)/2) throw std::invalid_argument("truncated K&C curve");
            for(size_t j=0;j<count;++j) {curve.x.push_back(a[at++]);curve.y.push_back(a[at++]);}
            if(!std::is_sorted(curve.x.begin(),curve.x.end())) throw std::invalid_argument("unordered K&C curve");
        }
        if(at!=n) throw std::invalid_argument("extra K&C data");
    }
    std::array<double,2> offsets(int i,double comp,const double* f) const {
        const int axle=i/2; const double sign=i%2==0 ? 1.0:-1.0;
        const auto* c=coeff.data()+5*axle;
        return {-sign*curves[2*axle].at(comp)+c[0]*f[KC_FY]+c[1]*f[KC_FX]+c[2]*f[KC_MZ],
                 sign*curves[2*axle+1].at(comp)+c[3]*f[KC_FY]+c[4]*f[KC_FZ]};
    }
};
inline void multibody_rhs(const double* p,const double* t,const double* frozen,
                          const double* s,const KC& kc,const double* y,double* dy) {
    std::fill(dy,dy+24,0.0);
    double fs[4],fz[4],fx=0.0,fy=0.0,mz=0.0,fsum=0.0,qroll=0.0,qpitch=0.0;
    const double u=y[10],v=y[11],r=y[12];
    for(int i=0;i<4;++i) {
        const auto* f=frozen+i*NF;
        const double comp=y[6+i]-(y[3]+f[X]*y[5]+f[Y]*y[4]);
        const double rate=y[16+i]-(y[13]+f[X]*y[15]+f[Y]*y[14]);
        fs[i]=s[W0+i]+s[KS]*comp+s[CS]*rate;
        if(s[ARBF]>0.0 || s[ARBR]>0.0) {
            const double k=(i<2?s[ARBF]:s[ARBR])/std::max(i<2?p[TF]:p[TR],1e-6);
            const double sign=(f[Y]>0.0)-(f[Y]<0.0);
            fs[i] += -k*y[4]*sign;
        }
        fs[i]=std::max(fs[i],0.0);
        fz[i]=std::max(s[W0+i]+s[MU_S]*9.81+s[KT]*(f[ROAD]-y[6+i]),0.0);
        const auto offsets=kc.offsets(i,comp,f);
        const double delta=clip(f[DELTA]+f[TOE]+offsets[0],-p[LIMIT],p[LIMIT]);
        const auto force=body_force(p,t,f,u,v,r,y[20+i],delta,fz[i],offsets[1],i);
        fx+=force.fx; fy+=force.fy; mz+=f[X]*force.fy-f[Y]*force.fx+force.mz;
        fsum+=fs[i]; qroll+=f[Y]*fs[i]; qpitch+=f[X]*fs[i];
    }
    const double grade=std::atan2(0.5*(frozen[ROAD]+frozen[NF+ROAD])-0.5*(frozen[2*NF+ROAD]+frozen[3*NF+ROAD]),p[L]);
    fx += -s[M]*9.81*std::sin(grade); fx += resistance(p,u);
    const double e=p[L]/2.0-p[A];
    const auto d=planar(p[MASS],p[IZ],e,u,v,r,fx,fy,mz);
    const double ax=d[0]-r*v-e*r*r, ay=d[1]+r*u+e*d[2];
    const double qdyn=s[QAERO]*u*u, lf=s[CLF]*qdyn, lr=s[CLR]*qdyn;
    double vz=(fsum+lf+lr-s[MS]*9.81)/s[MS];
    const double roll=(qroll+s[MS]*ay*s[H]-s[ARBB]*y[4])/s[IROLL];
    qpitch-=s[MS]*9.81*s[XSCG];
    const double pitch=(qpitch+s[MS]*ax*s[H]+0.5*p[L]*(lf-lr)-s[XSCG]*s[MS]*vz)/s[IPITCH];
    vz-=s[XSCG]*pitch;
    const double cp=std::cos(y[2]),sp=std::sin(y[2]);
    dy[0]=u*cp-v*sp;dy[1]=u*sp+v*cp;dy[2]=r;
    dy[3]=y[13];dy[4]=y[14];dy[5]=y[15];
    dy[10]=d[0];dy[11]=d[1];dy[12]=d[2];dy[13]=vz;dy[14]=roll;dy[15]=pitch;
    for(int i=0;i<4;++i) {dy[6+i]=y[16+i];dy[16+i]=(fz[i]-fs[i]-s[MU_S]*9.81)/s[MU_S];}
}

template<size_t N,typename RHS>
inline void rk4(const double* y,double dt,double* out,RHS rhs) {
    std::array<double,N> k1,k2,k3,k4,tmp;
    rhs(y,k1.data());
    for(size_t i=0;i<N;++i) tmp[i]=y[i]+(dt*0.5)*k1[i];
    rhs(tmp.data(),k2.data());
    for(size_t i=0;i<N;++i) tmp[i]=y[i]+(dt*0.5)*k2[i];
    rhs(tmp.data(),k3.data());
    for(size_t i=0;i<N;++i) tmp[i]=y[i]+dt*k3[i];
    rhs(tmp.data(),k4.data());
    for(size_t i=0;i<N;++i) out[i]=y[i]+(dt/6.0)*(k1[i]+2.0*k2[i]+2.0*k3[i]+k4[i]);
    rhs(out,out+N); // derivative at the accepted state, not the last RK stage
}

// Geometry input: 2 rows (outer_x,outer_y,inner_x,inner_y,axis_rad,travel_limit).
// Output rows: arm/tie angle, tie/rack angle, efficiency, length, travel,
// signed travel derivative, validity. Infinity marks an invalid singularity.
inline void linkage(const double* delta,const double* geom,double eta,double fallback,double* out) {
    eta=clip(eta,1e-6,1.0);
    for(int i=0;i<4;++i) {
        const double* g=geom+6*(i/2);const double mirror=i%2==0?1.0:-1.0;
        double ax=std::cos(g[4]),ay=mirror*std::sin(g[4]);
        const double an=std::max(std::sqrt(ax*ax+ay*ay),1e-12);ax/=an;ay/=an;
        const double ox=g[0],oy=mirror*g[1],ix=g[2],iy=mirror*g[3];
        const double c=std::cos(delta[i]),s=std::sin(delta[i]);
        const double x=c*ox-s*oy,y=s*ox+c*oy;
        const double dx=ix-ox,dy=iy-oy,len=std::max(std::sqrt(dx*dx+dy*dy),1e-6);
        const double rx=ix-x,ry=iy-y,b=2.0*(ax*rx+ay*ry),cq=(rx*rx+ry*ry)-len*len;
        const double disc=b*b-4.0*cq;bool valid=disc>=0.0;double travel;
        if(valid) {const double root=std::sqrt(disc),a=(-b+root)*0.5,z=(-b-root)*0.5;travel=std::abs(a)<=std::abs(z)?a:z;}
        else travel=-(ax*rx+ay*ry);
        const double limit=std::max(g[5],0.0);
        if(limit>0.0) {valid=valid && std::abs(travel)<=limit+1e-10;travel=clip(travel,-limit,limit);}
        const double tx=ix+travel*ax-x,ty=iy+travel*ay-y;
        const double la=std::max(std::sqrt(x*x+y*y),1e-9),lt=std::max(std::sqrt(tx*tx+ty*ty),1e-9);
        const double ux=x/la,uy=y/la,vx=tx/lt,vy=ty/lt;
        const double dot=clip(ux*vx+uy*vy,-1.0,1.0),cross=ux*vy-uy*vx,proj=vx*ax+vy*ay;
        valid=valid && la>1e-8 && lt>1e-8 && std::abs(cross)>1e-10;
        out[i]=std::acos(dot);out[4+i]=std::acos(clip(std::abs(proj),0.0,1.0));
        out[8+i]=std::max(std::abs(cross)*std::abs(proj)*eta,1e-6);
        out[12+i]=la>1e-8?la:fallback;out[16+i]=travel;
        if(std::abs(proj)>1e-12) out[20+i]=la*cross/proj;
        else {out[20+i]=std::copysign(INFINITY,cross);valid=false;}
        out[24+i]=valid?1.0:0.0;
    }
}
} // namespace sim4wis
