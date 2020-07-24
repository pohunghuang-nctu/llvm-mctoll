//===-- AArch64EliminatePrologEpilog.cpp ----------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of AArch64MachineInstructionRaiser class
// for use by llvm-mctoll.
//
//===----------------------------------------------------------------------===//

#include "AArch64MachineInstructionRaiser.h"
#include "AArch64ArgumentRaiser.h"
#include "AArch64CreateJumpTable.h"
#include "AArch64EliminatePrologEpilog.h"
#include "AArch64FrameBuilder.h"
#include "AArch64FunctionPrototype.h"
#include "AArch64InstructionSplitting.h"
#include "AArch64MIRevising.h"
#include "AArch64ModuleRaiser.h"
#include "AArch64SelectionDAGISel.h"

using namespace llvm;

AArch64MachineInstructionRaiser::AArch64MachineInstructionRaiser(
    MachineFunction &machFunc, const ModuleRaiser *mr, MCInstRaiser *mcir)
    : MachineInstructionRaiser(machFunc, mr, mcir),
      machRegInfo(MF.getRegInfo()) {}

bool AArch64MachineInstructionRaiser::raiseMachineFunction() {
  // Paul Huang 20200427 going to dig down. 
  const AArch64ModuleRaiser *amr = dyn_cast<AArch64ModuleRaiser>(MR);
  assert(amr != nullptr && "The AArch64 module raiser is not initialized!");
  AArch64ModuleRaiser &rmr = const_cast<AArch64ModuleRaiser &>(*amr);

  AArch64MIRevising mir(rmr);
  mir.init(&MF, raisedFunction);
  mir.setMCInstRaiser(mcInstRaiser);
  mir.revise();
  

  AArch64EliminatePrologEpilog epe(rmr);
  epe.init(&MF, raisedFunction);
  epe.eliminate();

  AArch64CreateJumpTable cjt(rmr);
  cjt.init(&MF, raisedFunction);
  cjt.setMCInstRaiser(mcInstRaiser);
  cjt.create();
  exit(0);
  cjt.getJTlist(jtList);

  AArch64ArgumentRaiser ar(rmr);
  ar.init(&MF, raisedFunction);
  ar.raiseArgs();

  AArch64FrameBuilder fb(rmr);
  fb.init(&MF, raisedFunction);
  fb.build();

  AArch64InstructionSplitting ispl(rmr);
  ispl.init(&MF, raisedFunction);
  ispl.split();

  AArch64SelectionDAGISel sdis(rmr);
  sdis.init(&MF, raisedFunction);
  sdis.setjtList(jtList);
  sdis.doSelection();

  return true;
}

bool AArch64MachineInstructionRaiser::raise() {
  raiseMachineFunction();
  return true;
}

int AArch64MachineInstructionRaiser::getArgumentNumber(unsigned PReg) {
  // NYI
  assert(false &&
         "Unimplemented AArch64MachineInstructionRaiser::getArgumentNumber()");
  return -1;
}

bool AArch64MachineInstructionRaiser::buildFuncArgTypeVector(
    const std::set<MCPhysReg> &PhysRegs, std::vector<Type *> &ArgTyVec) {
  // NYI
  assert(false &&
         "Unimplemented AArch64MachineInstructionRaiser::buildFuncArgTypeVector()");
  return false;
}

Value *AArch64MachineInstructionRaiser::getRegOrArgValue(unsigned PReg, int MBBNo) {
  assert(false &&
         "Unimplemented AArch64MachineInstructionRaiser::getRegOrArgValue()");
  return nullptr;
}

FunctionType *AArch64MachineInstructionRaiser::getRaisedFunctionPrototype() {
  AArch64FunctionPrototype AFP;
  raisedFunction = AFP.discover(MF);

  Function *ori = const_cast<Function *>(&MF.getFunction());
  // Insert the map of raised function to tempFunctionPointer.
  const_cast<ModuleRaiser *>(MR)->insertPlaceholderRaisedFunctionMap(
      raisedFunction, ori);

  return raisedFunction->getFunctionType();
}

// Create a new MachineFunctionRaiser object and add it to the list of
// MachineFunction raiser objects of this module.
MachineFunctionRaiser *AArch64ModuleRaiser::CreateAndAddMachineFunctionRaiser(
    Function *f, const ModuleRaiser *mr, uint64_t start, uint64_t end) {
  MachineFunctionRaiser *mfRaiser = new MachineFunctionRaiser(
      *M, mr->getMachineModuleInfo()->getOrCreateMachineFunction(*f), mr, start,
      end);
  mfRaiser->setMachineInstrRaiser(new AArch64MachineInstructionRaiser(
      mfRaiser->getMachineFunction(), mr, mfRaiser->getMCInstRaiser()));
  mfRaiserVector.push_back(mfRaiser);
  return mfRaiser;
}
