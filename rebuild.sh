#!/bin/bash
script_path=`dirname $(readlink -f $0)`
cd $script_path/../../../build
pwd
ninja llvm-mctoll
