#!/usr/bin/env python3
# Build an offline C++ application, with optional Python user-code support.
import argparse,hashlib,importlib.util,json,os,pathlib,shutil,subprocess,sys,zipfile
ROOT=pathlib.Path(__file__).resolve().parents[2]
def copy_tree(src,dst):
 if dst.exists():shutil.rmtree(dst)
 shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','*.pyc','.DS_Store','node_modules','.git','*.so','*.pyd'))
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--target',choices=['macos-arm64','windows-x64'],required=True);parser.add_argument('--binary',required=True,type=pathlib.Path);parser.add_argument('--compat-package',type=pathlib.Path);parser.add_argument('--without-plugin-runtime',action='store_true');args=parser.parse_args()
 out=ROOT/'dist_cpp';out.mkdir(exist_ok=True)
 package=out/f'4WIS_CPP_{args.target}'
 package.mkdir(exist_ok=True);(package/'app').mkdir(exist_ok=True)
 binary=package/'app'/('sim4wis-cpp.exe' if args.target=='windows-x64' else 'sim4wis-cpp')
 shutil.copy2(args.binary,binary);binary.chmod(0o755)
 copy_tree(ROOT/'frontend/dist',package/'app/web');copy_tree(ROOT/'cpp_backend/data',package/'app/data')
 for name in ('projects','scripts_lib','plugins','targets','procedures','experiments','vehicle_profiles','kc_profiles'):
  if (ROOT/name).exists():copy_tree(ROOT/name,package/name)
 copy_tree(ROOT/'docs',package/'docs')
 copy_tree(ROOT/'backend/src/sim4wis',package/'app/backend/sim4wis')
 shutil.copy2(ROOT/'cpp_backend/tools/plugin_worker.py',package/'app/plugin_worker.py')
 # Original user programs require Python. It is a compatibility dependency,
 # not an alternate simulation engine hidden behind the executable.
 if not args.without_plugin_runtime:
  if args.compat_package:
   copy_tree(args.compat_package/'runtime',package/'app/python')
   # Extension modules in dependencies must remain in the runtime.
   if (package/'vendor').exists():shutil.rmtree(package/'vendor')
   shutil.copytree(args.compat_package/'vendor',package/'vendor',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
   # copy_tree intentionally filters application extensions, but CPython needs
   # all its native standard library modules, so copy runtime without that filter.
   shutil.rmtree(package/'app/python')
   shutil.copytree(args.compat_package/'runtime',package/'app/python',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
  elif not (package/'app/python').exists():
   spec=importlib.util.spec_from_file_location('portable',ROOT/'scripts/build_portable.py');portable=importlib.util.module_from_spec(spec);spec.loader.exec_module(portable)
   target=portable.TARGETS[args.target];name,url=portable.resolve_pbs_asset(target['pbs_glob']);archive=portable.download(url,out/'.cache'/name)
   temporary=out/'runtime-assembly';temporary.mkdir(exist_ok=True)
   if (temporary/'runtime').exists():shutil.rmtree(temporary/'runtime')
   portable.extract_runtime(archive,temporary);shutil.move(str(temporary/'runtime'),package/'app/python')
   portable.vendor_deps(package/'vendor',target['pip_platforms'],['numpy','pydantic','pyyaml'])
 # Both launchers use the same binary; each data directory is separate from A.
 mac='''#!/bin/bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"
exec "$DIR/app/sim4wis-cpp" --root "$DIR" '''
 windows='''@echo off
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0app\\sim4wis-cpp.exe" --root "%~dp0." '''
 if args.target=='macos-arm64':
  for name,tail in [('Start.command','--serve --port 8020'),('MCP.command','--mcp --port 8020 "$@"'),('Self-test.command','--self-test')]:
   p=package/name;p.write_text(mac+tail+'\n',encoding='utf-8');p.chmod(0o755)
 else:
  for name,tail in [('Start.bat','--serve --port 8020'),('MCP.bat','--mcp --port 8020 %*'),('Self-test.bat','--self-test')]:
   (package/name).write_text(windows+tail+'\n',encoding='utf-8',newline='\r\n')
 (package/'使用说明.txt').write_text('''4WIS 全 C++ 后端对比版 v0.107.0
双击 Start，浏览器访问 http://127.0.0.1:8020 。关闭终端停止服务。
端口 8020 与内核迁移版 8010 分开，两个包的 projects/runs 等数据也分开。
MCP：运行 app/sim4wis-cpp[.exe] --root 包路径 --print-config 获取 Agent 配置。
Self-test 执行冻结的六组黄金工况；内置仿真与分析不需要 Python/Node。
现有 React 浏览器界面保留。Python/FMU/MATLAB 用户插件在独立 Python 兼容进程执行；附带基础 Python 依赖，FMU/MATLAB 的额外依赖与许可仍需自行配置。
可使用 SIM4WIS_PLUGIN_PYTHON 指定插件解释器；外部插件为受信任用户代码，进程隔离并非安全沙箱。
本预发布版用于离线软件对比，不代表台架或实车相关性验收；研究型多体模型和简化选型假设见报告。
回滚：停止本服务，启动另一个候选目录或原稳定版，无需覆盖旧目录。
''',encoding='utf-8')
 licenses=package/'third-party-licenses';licenses.mkdir(exist_ok=True)
 for p in (ROOT/'cpp_backend/third_party/licenses').rglob('*'):
  if p.is_file() and any(x in p.name.upper() for x in ('LICENSE','COPYING')):
   dest=licenses/p.relative_to(ROOT/'cpp_backend/third_party');dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
 shutil.copy2(ROOT/'cpp_backend/third_party/manifest.json',licenses/'manifest.json')
 manifest={'version':'0.107.0','route':'cpp-full-backend','target':args.target,'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'plugin_runtime_bundled':not args.without_plugin_runtime,'builtins_require_python':False}
 (package/'build-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 zip_path=out/f'4WIS_CPP_v0.107.0_{args.target}.zip'
 with zipfile.ZipFile(zip_path,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
  for path in sorted(package.rglob('*')):
   if path.is_file():archive.write(path,path.relative_to(out))
 print(json.dumps({'archive':str(zip_path),'sha256':hashlib.sha256(zip_path.read_bytes()).hexdigest(),'bytes':zip_path.stat().st_size,'package':str(package)}))
if __name__=='__main__':main()
