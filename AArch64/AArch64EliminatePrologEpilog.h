//===-- AArch64EliminatePrologEpilog.h ------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of AArch64EliminatePrologEpilog class for
// use by llvm-mctoll.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AArch64ELIMINATEPROLOGEPILOG_H
#define LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AArch64ELIMINATEPROLOGEPILOG_H

#include "AArch64RaiserBase.h"

using namespace llvm;

class AArch64EliminatePrologEpilog : public AArch64RaiserBase {
public:
  static char ID;

  AArch64EliminatePrologEpilog(AArch64ModuleRaiser &mr);
  ~AArch64EliminatePrologEpilog();
  void init(MachineFunction *mf = nullptr, Function *rf = nullptr) override;
  bool eliminate();
  bool runOnMachineFunction(MachineFunction &mf) override;

private:
  bool checkRegister(unsigned Reg, std::vector<MachineInstr *> &instrs) const;
  bool eliminateProlog(MachineFunction &mf) const;
  bool eliminateEpilog(MachineFunction &mf) const;
  /// Analyze stack size base on moving sp.
  void analyzeStackSize(MachineFunction &mf);
  /// Analyze frame adjustment base on the offset between fp and base sp.
  void analyzeFrameAdjustment(MachineFunction &mf);
};

#endif // LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AARCH64ELIMINATEPROLOGEPILOG_H
