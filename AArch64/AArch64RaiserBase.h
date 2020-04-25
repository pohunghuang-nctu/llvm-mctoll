//===-- AArch64RaiserBase.h -----------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the AArch64RaiserBase class. This class
// is a base class that provides some basic utilities to AArch64 raisers.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AARCH64RAISERBASE_H
#define LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AARCH64RAISERBASE_H

#include "AArch64ModuleRaiser.h"
#include "llvm-mctoll.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/Pass.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

class AArch64RaiserBase : public FunctionPass {
protected:
  AArch64RaiserBase() = delete;
  AArch64RaiserBase(char &PassID, AArch64ModuleRaiser &mr)
      : FunctionPass(PassID), MR(&mr) {
    PrintPass =
        (cl::getRegisteredOptions()["print-after-all"]->getNumOccurrences() >
         0);
    M = MR->getModule();
  }
  ~AArch64RaiserBase() override {}
  virtual void init(MachineFunction *mf = nullptr, Function *rf = nullptr) {
    if (mf)
      MF = mf;
    if (rf)
      RF = rf;
  }
  virtual bool runOnMachineFunction(MachineFunction &mf) { return false; }
  bool runOnFunction(Function &Func) override {
    RF = &Func;
    MF = MR->getMachineFunction(&Func);
    return runOnMachineFunction(*MF);
  }

  /// Get current raised llvm::Function.
  Function *getCRF() { return RF; }

  bool PrintPass;
  AArch64ModuleRaiser *MR;
  /// Current raised llvm::Function.
  Function *RF;
  MachineFunction *MF;
  Module *M;
};

#endif // LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AARCH64RAISERBASE_H
