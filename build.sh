#!/bin/bash
script_path=`dirname $(readlink -f $0)`
cd $script_path/../../../build
cmake -G "Ninja" -DLLVM_TARGETS_TO_BUILD="X86;ARM;AArch64" -DLLVM_ENABLE_PROJECTS="clang;lld" -DLLVM_ENABLE_ASSERTIONS=true -DCMAKE_BUILD_TYPE=Release ../llvm
ninja llvm-mctoll && ninja lld
