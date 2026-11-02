#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
import argparse,hashlib,json,subprocess
from pathlib import Path
r=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--ndk',required=True,type=Path);g=p.add_mutually_exclusive_group(required=True);g.add_argument('--stock-cpp',type=Path);g.add_argument('--vendor-cache',type=Path);p.add_argument('--device',type=Path,default=r.parent/'qiyu-device-service');p.add_argument('--output',required=True,type=Path);a=p.parse_args()
if a.vendor_cache:a.stock_cpp=a.vendor_cache/'controller/root/system/lib/libc++_shared.so'
if a.output.exists():p.error('Choose a new output directory')
tools=a.ndk/'toolchains/llvm/prebuilt/linux-x86_64/bin';cc=tools/'armv7a-linux-androideabi28-clang++'
if not cc.is_file() or not a.stock_cpp.is_file():p.error('Need the NDK compiler and stock ARM32 libc++_shared.so')
a.output.mkdir(parents=True);commands=[]
for src,name in [('controller_sdk_pose.cpp','controller-sdk-pose'),('controller_sdk_imu.c','controller-sdk-imu'),('controller_sdk_probe.c','controller-sdk-probe')]:
 compiler=cc if src.endswith('.cpp') else tools/'armv7a-linux-androideabi28-clang'
 args=[str(compiler),'-O2','-g','-Wall','-Wextra','-fPIE','-pie','-I'+str(r/'include'),'-I'+str(a.device/'include'),str(r/'src'/src)]
 if src.endswith('.cpp'):args+=['-std=c++17','-nostdlib++',str(a.stock_cpp)]
 args+=['-ldl','-o',str(a.output/name)];subprocess.run(args,check=True);commands.append(args)
files={q.name:hashlib.sha256(q.read_bytes()).hexdigest() for q in a.output.iterdir() if q.is_file()}
(a.output/'build-manifest.json').write_text(json.dumps({'commands':commands,'files_sha256':files,'stock_cpp_sha256':hashlib.sha256(a.stock_cpp.read_bytes()).hexdigest(),'vendor_cache_receipt_sha256':hashlib.sha256((a.vendor_cache/'receipt.json').read_bytes()).hexdigest() if a.vendor_cache else None,'device_tested':False},indent=2)+'\n')
