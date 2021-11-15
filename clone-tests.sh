#!/bin/bash
git clone https://github.com/riscv-software-src/riscv-tests.git
cd riscv-tests
git submodule update --init --recursive
autoconf
./configure --prefix=$RISCV/target
make
make install
