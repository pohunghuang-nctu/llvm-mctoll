//===-- AArch64EliminatePrologEpilog.h ------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of AArch64MachineInstructionRaiser class for
// use by llvm-mctoll.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AARCH64MACHINEINSTRUCTIONRAISER_H
#define LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AARCH64MACHINEINSTRUCTIONRAISER_H

#include "MachineInstructionRaiser.h"

class AArch64MachineInstructionRaiser : public MachineInstructionRaiser {
public:
  AArch64MachineInstructionRaiser() = delete;
  AArch64MachineInstructionRaiser(MachineFunction &machFunc, const ModuleRaiser *mr,
                              MCInstRaiser *mcir);
  bool raise();
  FunctionType *getRaisedFunctionPrototype();
  int getArgumentNumber(unsigned PReg);
  Value *getRegOrArgValue(unsigned PReg, int MBBNo);
  bool buildFuncArgTypeVector(const std::set<MCPhysReg> &,
                              std::vector<Type *> &);

  std::vector<JumpTableInfo> jtList;

private:
  bool raiseMachineFunction();
  // Commonly used LLVM data structures during this phase
  MachineRegisterInfo &machRegInfo;
};
#endif // LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AArch64MACHINEINSTRUCTIONRAISER_H
