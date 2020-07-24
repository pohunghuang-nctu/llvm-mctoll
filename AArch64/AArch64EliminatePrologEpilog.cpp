//===- AArch64EliminatePrologEpilog.cpp - Binary raiser utility llvm-mctoll ---===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of AArch64EliminatePrologEpilog class
// for use by llvm-mctoll.
//
//===----------------------------------------------------------------------===//

#include "AArch64EliminatePrologEpilog.h"
#include "AArch64Subtarget.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mctoll"

using namespace llvm;

char AArch64EliminatePrologEpilog::ID = 0;

AArch64EliminatePrologEpilog::AArch64EliminatePrologEpilog(AArch64ModuleRaiser &mr)
    : AArch64RaiserBase(ID, mr) {}

AArch64EliminatePrologEpilog::~AArch64EliminatePrologEpilog() {}

void AArch64EliminatePrologEpilog::init(MachineFunction *mf, Function *rf) {
  AArch64RaiserBase::init(mf, rf);
}

/// Return true if an operand in the instrs vector matches the passed register
/// number, otherwise false.
bool AArch64EliminatePrologEpilog::checkRegister(
    unsigned Reg, std::vector<MachineInstr *> &instrs) const {
  std::vector<MachineInstr *>::iterator it = instrs.begin();
  for (; it < instrs.end(); ++it) {
    MachineInstr *mi = *it;
    if (mi->mayStore()) {
      for (unsigned i = 0; i < mi->getNumOperands(); i++) {
        MachineOperand MO = mi->getOperand(i);

        // Compare the register number.
        if (MO.isReg() && MO.getReg() == Reg)
          return true;
      }
    }
  }
  return false;
}

/// Raise the function prolog.
///
/// Look for the following instructions and eliminate them:
///       str fp, [sp, #-4]!
///       add fp, sp, #0
///
///       sub sp, fp, #0
///       ldr fp, [sp], #4
/// AND
///       push {r11,lr}
///       add r11, sp, #4
///
///       sub sp, r11, #4
///       pop	{r11, pc}
/// AND
///       stmdb r13!, {r0-r3}
///       stmdb r13!, {r4-r12,r13,r14}
///
///       ldmia r13, {r4-r11, r13, r15}
/// AND
///       mov r12, r13
///       stmdb r13!, {r0-r3}
///       stmdb r13!, {r4-r12, r14}
///       sub r11, r12, #16
///
///       ldmdb r13, {r4-r11, r13, r15}
bool AArch64EliminatePrologEpilog::eliminateProlog(MachineFunction &MF) const {
  std::vector<MachineInstr *> prologInstrs;
  MachineBasicBlock &frontMBB = MF.front();

  const AArch64Subtarget &STI = MF.getSubtarget<AArch64Subtarget>();
  const AArch64RegisterInfo *RegInfo = STI.getRegisterInfo();
  unsigned FramePtr = RegInfo->getFrameRegister(MF);

  for (MachineBasicBlock::iterator frontMBBIter = frontMBB.begin();
       frontMBBIter != frontMBB.end(); frontMBBIter++) {
    MachineInstr &curMachInstr = (*frontMBBIter);
    //curMachInstr.dump();
    // move sp, fp  -- ??? not be seen
    if (curMachInstr.getOpcode() == AArch64::ADDXri) {
      if (curMachInstr.getOperand(0).isReg() &&
          curMachInstr.getOperand(0).getReg() == AArch64::SP &&
          curMachInstr.getOperand(1).isReg() &&
          curMachInstr.getOperand(1).getReg() == FramePtr) {
        prologInstrs.push_back(&curMachInstr);
        continue;
      }
    }

    // Push the STORE instruction
    if (curMachInstr.mayStore()) {
      MachineOperand storeOperand = curMachInstr.getOperand(0);
      if (storeOperand.isReg() && storeOperand.getReg() == FramePtr) {
        prologInstrs.push_back(&curMachInstr);
        continue;
      }
    }

    // Push the ADDri instruction
    // stp, x29, x30, [sp, 0x40]  ; preserved FR & LR
    if (curMachInstr.getOpcode() == AArch64::STPXi &&
        curMachInstr.getOperand(0).getReg() == AArch64::FP &&
        curMachInstr.getOperand(1).getReg() == AArch64::LR) {
      prologInstrs.push_back(&curMachInstr);
      continue;
    }

    // Push the add fp, sp, 0x40
    if (curMachInstr.getOpcode() == AArch64::ADDXri &&
        curMachInstr.getOperand(0).getReg() == AArch64::FP&&
        curMachInstr.getOperand(1).getReg() == AArch64::SP) {
      prologInstrs.push_back(&curMachInstr);
      continue;
    }

    // Push sub sp, sp, 0x50
    if (curMachInstr.getOpcode() == AArch64::SUBXri &&
        curMachInstr.getOperand(0).getReg() == AArch64::SP &&
        curMachInstr.getOperand(1).getReg() == AArch64::SP) {
      prologInstrs.push_back(&curMachInstr);
      continue;
    }
    //prologInstrs.back()->dump();
  }
  
  // Create the stack frame
  const TargetRegisterInfo *TRI = MF.getRegInfo().getTargetRegisterInfo();
  const MCPhysReg *CSRegs = TRI->getCalleeSavedRegs(&MF);

  std::vector<CalleeSavedInfo> CSI;
  for (unsigned i = 0; CSRegs[i]; ++i) {
    unsigned Reg = CSRegs[i];

    // Save register.
    if (checkRegister(Reg, prologInstrs)) {
      CSI.push_back(CalleeSavedInfo(Reg));
      dbgs() << "called saved register: " << Reg << "\n";
    }
  }
  dbgs() << "total # of saved register: " << CSI.size() << "\n";
  const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  if (!TFI->assignCalleeSavedSpillSlots(MF, RegInfo, CSI)) {
    // If target doesn't implement this, use generic code.
    dbgs() << "assignCalleeSavedSpillSlots not implemented.\n ";
    if (CSI.empty())
      return true; // Early exit if no callee saved registers are modified!

    unsigned NumFixedSpillSlots;
    const TargetFrameLowering::SpillSlot *FixedSpillSlots =
        TFI->getCalleeSavedSpillSlots(NumFixedSpillSlots);
    dbgs() << "# of fixed spilled slots: " << NumFixedSpillSlots << "\n";

    // Allocate stack slots for the registers that need to be saved and restored
    unsigned Offset = 0;
    for (auto &CS : CSI) {
      unsigned Reg = CS.getReg();
      const TargetRegisterClass *RC = RegInfo->getMinimalPhysRegClass(Reg);

      int FrameIdx;
      if (RegInfo->hasReservedSpillSlot(MF, Reg, FrameIdx)) {
        dbgs() << "Reg: " << Reg << "has reserved spilled slot: " << FrameIdx << "\n";
        CS.setFrameIdx(FrameIdx);
        continue;
      }
      dbgs() << "Reg: " << Reg << " has no reserved spilled slot \n";
      // Check if this physreg must be spilled to a particular stack slot for
      // this target
      const TargetFrameLowering::SpillSlot *FixedSlot = FixedSpillSlots;
      while (FixedSlot != FixedSpillSlots + NumFixedSpillSlots &&
             FixedSlot->Reg != Reg)
        ++FixedSlot;

      unsigned Size = RegInfo->getSpillSize(*RC);
      if (FixedSlot == FixedSpillSlots + NumFixedSpillSlots) {
        // Nope, just spill it anywhere convenient.
        unsigned Align = RegInfo->getSpillAlignment(*RC);
        unsigned StackAlign = TFI->getStackAlignment();

        // The alignment is the minimum of the desired alignment of the
        // TargetRegisterClass and the stack alignment, whichever is smaller.
        Align = std::min(Align, StackAlign);
        FrameIdx = MFI.CreateStackObject(Size, Align, true);
        printf("stack object created at index: %d, size is %d\n", FrameIdx, Size);
        Offset += Size;

        // Set the object offset
        MFI.setObjectOffset(FrameIdx, MFI.getObjectOffset(FrameIdx) - Offset); // ?? what for?
      } else {
        // Spill to the stack.
        FrameIdx = MFI.CreateFixedSpillStackObject(Size, FixedSlot->Offset);
        printf("fixed spilled stack object created at index: %d, size is %d\n", FrameIdx, Size);
      }

      // Set the frame index
      CS.setFrameIdx(FrameIdx);
    }
    MFI.setCalleeSavedInfo(CSI);
  }

  // Eliminate the instructions identified in function prologue
  unsigned int delInstSz = prologInstrs.size();
  dbgs() << "### prolog instructions to be erased:" << delInstSz << "\n";
  for (unsigned int i = 0; i < delInstSz; i++) {
    prologInstrs[i]->dump();
    frontMBB.erase(prologInstrs[i]);
  }
  dbgs() << "### prolog instructions erased done.\n";
  return true;
}

bool AArch64EliminatePrologEpilog::eliminateEpilog(MachineFunction &MF) const {
  const AArch64Subtarget &STI = MF.getSubtarget<AArch64Subtarget>();
  const AArch64RegisterInfo *RegInfo = STI.getRegisterInfo();
  const AArch64InstrInfo *TII = STI.getInstrInfo();
  unsigned FramePtr = RegInfo->getFrameRegister(MF);

  for (MachineBasicBlock &MBB : MF) {
    std::vector<MachineInstr *> epilogInstrs;
    // MBBI may be invalidated by the raising operation.
    for (MachineBasicBlock::iterator backMBBIter = MBB.begin();
         backMBBIter != MBB.end(); backMBBIter++) {
      MachineInstr &curMachInstr = (*backMBBIter);

      // Push the LOAD instruction
      if (curMachInstr.mayLoad()) {
        MachineOperand loadOperand = curMachInstr.getOperand(0);
        if (loadOperand.isReg() && loadOperand.getReg() == FramePtr) {
          // If the register list of current POP includes PC register,
          // it should be replaced with return instead of removed.
          /* Paul :In ARMv8, it's no longer allowed access PC directly, so this case is gone.  
          if (curMachInstr.findRegisterUseOperandIdx(AArch64::PC) != -1) {
            MachineInstrBuilder mib =
                BuildMI(MBB, &curMachInstr, DebugLoc(), TII->get(AArch64::BX_RET));
            int cpsridx = curMachInstr.findRegisterUseOperandIdx(AArch64::CPSR);
            if (cpsridx == -1) {
              mib.addImm(AArch64CC::AL);
            } else {
              mib.add(curMachInstr.getOperand(cpsridx - 1))
                  .add(curMachInstr.getOperand(cpsridx));
            }
            mib.add(curMachInstr.getOperand(
                curMachInstr.getNumExplicitOperands() - 1));
          }
          */
          epilogInstrs.push_back(&curMachInstr);
          continue;
        }
      }

      // Push the LDP instruction, eq ldp fr, lr, [sp + 8]
      if (curMachInstr.getOpcode() == AArch64::LDPXi &&
          curMachInstr.getOperand(0).getReg() == FramePtr &&
          curMachInstr.getOperand(1).getReg() == AArch64::LR &&
          curMachInstr.getOperand(2).getReg() == AArch64::SP) {
        epilogInstrs.push_back(&curMachInstr);
        continue;
      }

      // Push the ADDri instruction, eq add sp, sp, 0x50, revert sp
      if (curMachInstr.getOpcode() == AArch64::ADDXri &&
          curMachInstr.getOperand(0).isReg()) {
        if (curMachInstr.getOperand(0).getReg() == AArch64::SP &&
            curMachInstr.getOperand(1).isReg() &&
            curMachInstr.getOperand(1).getReg() == AArch64::SP &&
            curMachInstr.getOperand(2).isImm() &&
            curMachInstr.getOperand(2).getImm() == (int64_t)(MF.getFrameInfo().getStackSize())) {
          epilogInstrs.push_back(&curMachInstr);
          continue;
        }
      }
    }


    // Eliminate the instructions identified in function epilogue
    unsigned int delInstSz = epilogInstrs.size();
    if (delInstSz > 0) {
      dbgs() << "### epilog instructions to be erased:" << delInstSz << "\n";          
      for (unsigned int i = 0; i < delInstSz; i++) {
        epilogInstrs[i]->dump();
        MBB.erase(epilogInstrs[i]);      
      }
      dbgs() << "### epilog instructions erased done\n" ;    
    }  
  }

  return true;
}

/// Analyze stack size base on moving sp.
/// Patterns like:
/// sub	sp, sp, #28
void AArch64EliminatePrologEpilog::analyzeStackSize(MachineFunction &mf) {
  if (mf.size() < 1)
    return;

  const MachineBasicBlock &mbb = mf.front();

  for (const MachineInstr &mi : mbb.instrs()) {
    if (mi.getOpcode() == AArch64::SUBXri && mi.getNumOperands() >= 3 &&
        mi.getOperand(0).isReg() && mi.getOperand(0).getReg() == AArch64::SP &&
        mi.getOperand(1).isReg() && mi.getOperand(1).getReg() == AArch64::SP &&
        mi.getOperand(2).isImm() && mi.getOperand(2).getImm() > 0) {
      mf.getFrameInfo().setStackSize(mi.getOperand(2).getImm());
      break;
    }
  }
}

/// Analyze frame adjustment base on the offset between fp and base sp.
/// Patterns like:
/// add	fp, sp, #8
void AArch64EliminatePrologEpilog::analyzeFrameAdjustment(MachineFunction &mf) {
  if (mf.size() < 1)
    return;

  const MachineBasicBlock &mbb = mf.front();

  for (const MachineInstr &mi : mbb.instrs()) {
    if (mi.getOpcode() == AArch64::ADDXri && mi.getNumOperands() >= 3 &&
        mi.getOperand(0).isReg() && mi.getOperand(0).getReg() == AArch64::FP &&
        mi.getOperand(1).isReg() && mi.getOperand(1).getReg() == AArch64::SP &&
        mi.getOperand(2).isImm() && mi.getOperand(2).getImm() > 0) {
      mf.getFrameInfo().setOffsetAdjustment(mi.getOperand(2).getImm());
      break;
    }
  }
}

bool AArch64EliminatePrologEpilog::eliminate() {
  if (PrintPass)
    dbgs() << "AArch64EliminatePrologEpilog start.\n";

  analyzeStackSize(*MF);
  analyzeFrameAdjustment(*MF);
  bool success = eliminateProlog(*MF);

  if (success) {
    success = eliminateEpilog(*MF);
  }
  exit(0);
  // For debugging.
  if (PrintPass) {
    LLVM_DEBUG(MF->dump());
    LLVM_DEBUG(getCRF()->dump());
    dbgs() << "AArch64EliminatePrologEpilog end.\n";
  }

  return !success;
}

bool AArch64EliminatePrologEpilog::runOnMachineFunction(MachineFunction &mf) {
  bool rtn = false;
  init();
  rtn = eliminate();
  return rtn;
}

#ifdef __cplusplus
extern "C" {
#endif

FunctionPass *InitializeAArch64EliminatePrologEpilog(AArch64ModuleRaiser &mr) {
  return new AArch64EliminatePrologEpilog(mr);
}

#ifdef __cplusplus
}
#endif
