//===- AArch64SelectionDAGISel.h ------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of AArch64SelectionDAGISel class for
// use by llvm-mctoll.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AARCH64SELECTIONDAGISEL_H
#define LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AARCH64SELECTIONDAGISEL_H

#include "AArch64RaiserBase.h"
#include "DAGBuilder.h"
#include "DAGRaisingInfo.h"
#include "FunctionRaisingInfo.h"
#include "IREmitter.h"
#include "InstSelector.h"
#include "ModuleRaiser.h"
#include "llvm/Analysis/OptimizationRemarkEmitter.h"

/// This is responsible for constructing DAG, and does instruction selection on
/// the DAG, eventually emits SDNodes of the DAG to LLVM IRs.
class AArch64SelectionDAGISel : public AArch64RaiserBase {
public:
  AArch64SelectionDAGISel(AArch64ModuleRaiser &mr);
  ~AArch64SelectionDAGISel() override;
  void init(MachineFunction *mf = nullptr, Function *rf = nullptr) override;
  bool doSelection();
  bool runOnMachineFunction(MachineFunction &mf) override;
  bool setjtList(std::vector<JumpTableInfo> &List);
  static char ID;

private:
  void initEntryBasicBlock();
  void selectBasicBlock();
  void doInstructionSelection();
  void emitDAG();

  std::unique_ptr<OptimizationRemarkEmitter> ORE;

  AArch64FunctionRaisingInfo *FuncInfo;
  AArch64DAGBuilder *SDB;
  AArch64InstSelector *SLT;

  SelectionDAG *CurDAG;
  AArch64DAGRaisingInfo *DAGInfo;
  MachineBasicBlock *MBB;
  BasicBlock *BB;
  std::vector<JumpTableInfo> jtList;
};

#endif // LLVM_TOOLS_LLVM_MCTOLL_AARCH64_AARCH64SELECTIONDAGISEL_H
