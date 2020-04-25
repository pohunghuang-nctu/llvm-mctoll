//===- AArch64ArgumentRaiser.cpp - Binary raiser utility llvm-mctoll ----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of AArch64ArgumentRaiser class for use by
// llvm-mctoll.
//
//===----------------------------------------------------------------------===//

#include "AArch64ArgumentRaiser.h"
#include "AArch64Subtarget.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include <vector>

#define DEBUG_TYPE "mctoll"

using namespace llvm;

char AArch64ArgumentRaiser::ID = 0;

AArch64ArgumentRaiser::AArch64ArgumentRaiser(AArch64ModuleRaiser &mr)
    : AArch64RaiserBase(ID, mr) {}

AArch64ArgumentRaiser::~AArch64ArgumentRaiser() {}

void AArch64ArgumentRaiser::init(MachineFunction *mf, Function *rf) {
  AArch64RaiserBase::init(mf, rf);
  MFI = &MF->getFrameInfo();
  TII = MF->getSubtarget<AArch64Subtarget>().getInstrInfo();
}

/// Change all return relative register operands to stack 0.
void AArch64ArgumentRaiser::updateReturnRegister(MachineFunction &mf) {
  for (MachineBasicBlock &mbb : mf) {
    if (mbb.succ_empty()) {
      bool loop = true;
      for (MachineBasicBlock::reverse_iterator ii = mbb.rbegin(),
                                               ie = mbb.rend();
           (ii != ie) && loop; ++ii) {
        MachineInstr &mi = *ii;
        for (MachineInstr::mop_iterator oi = mi.operands_begin(),
                                        oe = mi.operands_end();
             oi != oe; oi++) {
          MachineOperand &mo = *oi;
          if (mo.isReg() && (mo.getReg() == AArch64::X0)) {
            if (mo.isDef()) {
              mo.ChangeToFrameIndex(0);
              loop = false;
              break;
            }
          }
        }
      }
    }
  }
}

/// Change all function arguments of registers into stack elements with same
/// indexes of arguments.
void AArch64ArgumentRaiser::updateParameterRegister(unsigned reg,
                                                MachineBasicBlock &mbb) {
  for (MachineBasicBlock::iterator ii = mbb.begin(), ie = mbb.end(); ii != ie;
       ++ii) {
    MachineInstr &mi = *ii;
    for (MachineInstr::mop_iterator oi = mi.operands_begin(),
                                    oe = mi.operands_end();
         oi != oe; oi++) {
      MachineOperand &mo = *oi;
      if (mo.isReg() && (mo.getReg() == reg)) {
        if (mo.isUse()) {
          // The argument's index on frame starts from 1.
          // Such as X0 = 1, X1 = 2, X2 = 3, X3 = 4
          // For instance: X3 - X0 + 1 = 4
          mo.ChangeToFrameIndex(reg - AArch64::X0 + 1);
        } else
          return;
      }
    }
  }
}

/// Change rest of function arguments on stack frame into stack elements.
void AArch64ArgumentRaiser::updateParameterFrame(MachineFunction &mf) {

  for (MachineFunction::iterator mbbi = mf.begin(), mbbe = mf.end();
       mbbi != mbbe; ++mbbi) {
    MachineBasicBlock &mbb = *mbbi;

    for (MachineBasicBlock::iterator mii = mbb.begin(), mie = mbb.end();
         mii != mie; ++mii) {
      MachineInstr &mi = *mii;
      // Match pattern like ldr r1, [fp, #8].
      if (mi.getOpcode() == AArch64::LDRXui && mi.getNumOperands() > 2) {
        MachineOperand &mo = mi.getOperand(1);
        MachineOperand &mc = mi.getOperand(2);
        if (mo.isReg() && mo.getReg() == AArch64::SP && mc.isImm()) {
          // TODO: Need to check the imm is larger than 0 and it is align by
          // 4(32 bit).
          int imm = mc.getImm();
          if (imm >= 0) {
            int idx = imm / 4 - 2 + 5; // The index 0 is reserved to return
                                       // value. From 1 to 4 are the register
                                       // argument indices. Plus 5 to the index.
            mi.getOperand(1).ChangeToFrameIndex(idx);
            mi.RemoveOperand(2);
          }
        }
      }
    }
  }
}

/// Move arguments which are passed by AArch64 registers(R0 - R3) from function
/// arg.x to corresponding registers in entry block.
void AArch64ArgumentRaiser::moveArgumentToRegister(unsigned Reg,
                                               MachineBasicBlock &PMBB) {
  const MCInstrDesc &mcInstrDesc = TII->get(AArch64::ORRXrs);
  MachineInstrBuilder builder = BuildMI(*MF, *(new DebugLoc()), mcInstrDesc);
  builder.addDef(Reg);
  builder.addFrameIndex(Reg - AArch64::X0 + 1);
  PMBB.insert(PMBB.begin(), builder.getInstr());
}

/// updateParameterInstr - Using newly created stack elements replace relative
/// operands in MachineInstr.
void AArch64ArgumentRaiser::updateParameterInstr(MachineFunction &mf) {
  Function *fn = getCRF();
  // Move arguments to corresponding registers.
  MachineBasicBlock &EntryMBB = mf.front();
  switch (fn->arg_size()) {
  default:
    updateParameterFrame(mf);
    LLVM_FALLTHROUGH;
  case 8:
    moveArgumentToRegister(AArch64::X7, EntryMBB);
    LLVM_FALLTHROUGH;
  case 7:
    moveArgumentToRegister(AArch64::X6, EntryMBB);
    LLVM_FALLTHROUGH;
  case 6:
    moveArgumentToRegister(AArch64::X5, EntryMBB);
    LLVM_FALLTHROUGH;
  case 5:
    moveArgumentToRegister(AArch64::X4, EntryMBB);
    LLVM_FALLTHROUGH;
  case 4:
    moveArgumentToRegister(AArch64::X3, EntryMBB);
    LLVM_FALLTHROUGH;
  case 3:
    moveArgumentToRegister(AArch64::X2, EntryMBB);
    LLVM_FALLTHROUGH;
  case 2:
    moveArgumentToRegister(AArch64::X1, EntryMBB);
    LLVM_FALLTHROUGH;
  case 1:
    moveArgumentToRegister(AArch64::X0, EntryMBB);
    LLVM_FALLTHROUGH;
  case 0:
    break;
  }
}

bool AArch64ArgumentRaiser::raiseArgs() {
  if (PrintPass)
    dbgs() << "AArch64ArgumentRaiser start.\n";

  Function *fn = getCRF();

  int argidx = 1;
  for (Function::arg_iterator argi = fn->arg_begin(), arge = fn->arg_end();
       argi != arge; ++argi)
    argi->setName("arg." + std::to_string(argidx++));

  for (unsigned i = 0, e = fn->arg_size() + 1; i < e; ++i) {
    Align ALG(32);
    MFI->CreateStackObject(32, ALG, false);
  }

  updateParameterInstr(*MF);

  // For debugging.
  if (PrintPass) {
    LLVM_DEBUG(MF->dump());
    LLVM_DEBUG(getCRF()->dump());
    dbgs() << "AArch64ArgumentRaiser end.\n";
  }

  return true;
}

bool AArch64ArgumentRaiser::runOnMachineFunction(MachineFunction &mf) {
  bool rtn = false;
  init();
  rtn = raiseArgs();
  return rtn;
}

#ifdef __cplusplus
extern "C" {
#endif

FunctionPass *InitializeAArch64ArgumentRaiser(AArch64ModuleRaiser &mr) {
  return new AArch64ArgumentRaiser(mr);
}

#ifdef __cplusplus
}
#endif
