//===- AArch64FrameBuilder.cpp - Binary raiser utility llvm-mctoll ------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of AArch64FrameBuilder class for use by
// llvm-mctoll.
//
//===----------------------------------------------------------------------===//

#include "AArch64FrameBuilder.h"
#include "AArch64Subtarget.h"
#include "llvm/ADT/DenseMap.h"

#define DEBUG_TYPE "mctoll"

using namespace llvm;

char AArch64FrameBuilder::ID = 0;

AArch64FrameBuilder::AArch64FrameBuilder(AArch64ModuleRaiser &mr) : AArch64RaiserBase(ID, mr) {}

AArch64FrameBuilder::~AArch64FrameBuilder() {}

void AArch64FrameBuilder::init(MachineFunction *mf, Function *rf) {
  AArch64RaiserBase::init(mf, rf);
  MFI = &MF->getFrameInfo();
  CTX = &M->getContext();
  DLT = &M->getDataLayout();
}

static bool isLoadOP(unsigned opcode) {
  switch (opcode) {
  default:
    return false;
  case AArch64::LDRAAindexed  :
  case AArch64::LDRAAwriteback  :
  case AArch64::LDRABindexed  :
  case AArch64::LDRABwriteback  :
  case AArch64::LDRBBpost :
  case AArch64::LDRBBpre  :
  case AArch64::LDRBBroW  :
  case AArch64::LDRBBroX  :
  case AArch64::LDRBBui :
  case AArch64::LDRBpost  :
  case AArch64::LDRBpre :
  case AArch64::LDRBroW :
  case AArch64::LDRBroX :
  case AArch64::LDRBui  :
  case AArch64::LDRDl :
  case AArch64::LDRDpost  :
  case AArch64::LDRDpre :
  case AArch64::LDRDroW :
  case AArch64::LDRDroX :
  case AArch64::LDRDui  :
  case AArch64::LDRHHpost :
  case AArch64::LDRHHpre  :
  case AArch64::LDRHHroW  :
  case AArch64::LDRHHroX  :
  case AArch64::LDRHHui :
  case AArch64::LDRHpost  :
  case AArch64::LDRHpre :
  case AArch64::LDRHroW :
  case AArch64::LDRHroX :
  case AArch64::LDRHui  :
  case AArch64::LDRQl :
  case AArch64::LDRQpost  :
  case AArch64::LDRQpre :
  case AArch64::LDRQroW :
  case AArch64::LDRQroX :
  case AArch64::LDRQui  :
  case AArch64::LDRSBWpost  :
  case AArch64::LDRSBWpre :
  case AArch64::LDRSBWroW :
  case AArch64::LDRSBWroX :
  case AArch64::LDRSBWui  :
  case AArch64::LDRSBXpost  :
  case AArch64::LDRSBXpre :
  case AArch64::LDRSBXroW :
  case AArch64::LDRSBXroX :
  case AArch64::LDRSBXui  :
  case AArch64::LDRSHWpost  :
  case AArch64::LDRSHWpre :
  case AArch64::LDRSHWroW :
  case AArch64::LDRSHWroX :
  case AArch64::LDRSHWui  :
  case AArch64::LDRSHXpost  :
  case AArch64::LDRSHXpre :
  case AArch64::LDRSHXroW :
  case AArch64::LDRSHXroX :
  case AArch64::LDRSHXui  :
  case AArch64::LDRSWl  :
  case AArch64::LDRSWpost :
  case AArch64::LDRSWpre  :
  case AArch64::LDRSWroW  :
  case AArch64::LDRSWroX  :
  case AArch64::LDRSWui :
  case AArch64::LDRSl :
  case AArch64::LDRSpost  :
  case AArch64::LDRSpre :
  case AArch64::LDRSroW :
  case AArch64::LDRSroX :
  case AArch64::LDRSui  :
  case AArch64::LDRWl :
  case AArch64::LDRWpost  :
  case AArch64::LDRWpre :
  case AArch64::LDRWroW :
  case AArch64::LDRWroX :
  case AArch64::LDRWui  :
  case AArch64::LDRXl :
  case AArch64::LDRXpost  :
  case AArch64::LDRXpre :
  case AArch64::LDRXroW :
  case AArch64::LDRXroX :
  case AArch64::LDRXui  :
    return true;
  }
}

static bool isStoreOP(unsigned opcode) {
  switch (opcode) {
  default:
    return false;
  case AArch64::STRBBpost :
  case AArch64::STRBBpre  :
  case AArch64::STRBBroW  :
  case AArch64::STRBBroX  :
  case AArch64::STRBBui :
  case AArch64::STRBpost  :
  case AArch64::STRBpre :
  case AArch64::STRBroW :
  case AArch64::STRBroX :
  case AArch64::STRBui  :
  case AArch64::STRDpost  :
  case AArch64::STRDpre :
  case AArch64::STRDroW :
  case AArch64::STRDroX :
  case AArch64::STRDui  :
  case AArch64::STRHHpost :
  case AArch64::STRHHpre  :
  case AArch64::STRHHroW  :
  case AArch64::STRHHroX  :
  case AArch64::STRHHui :
  case AArch64::STRHpost  :
  case AArch64::STRHpre :
  case AArch64::STRHroW :
  case AArch64::STRHroX :
  case AArch64::STRHui  :
  case AArch64::STRQpost  :
  case AArch64::STRQpre :
  case AArch64::STRQroW :
  case AArch64::STRQroX :
  case AArch64::STRQui  :
  case AArch64::STRSpost  :
  case AArch64::STRSpre :
  case AArch64::STRSroW :
  case AArch64::STRSroX :
  case AArch64::STRSui  :
  case AArch64::STRWpost  :
  case AArch64::STRWpre :
  case AArch64::STRWroW :
  case AArch64::STRWroX :
  case AArch64::STRWui  :
  case AArch64::STRXpost  :
  case AArch64::STRXpre :
  case AArch64::STRXroW :
  case AArch64::STRXroX :
  case AArch64::STRXui  :
    return true;
  }
}

static bool isAddOP(unsigned opcode) {
  switch (opcode) {
  default:
    return false;
  case AArch64::ADDSWri :
  case AArch64::ADDSWrs :
  case AArch64::ADDSWrx :
  case AArch64::ADDSXri :
  case AArch64::ADDSXrs :
  case AArch64::ADDSXrx :
  case AArch64::ADDSXrx64 :
  case AArch64::ADDVL_XXI :
  case AArch64::ADDVv16i8v  :
  case AArch64::ADDVv4i16v  :
  case AArch64::ADDVv4i32v  :
  case AArch64::ADDVv8i16v  :
  case AArch64::ADDVv8i8v :
  case AArch64::ADDWri  :
  case AArch64::ADDWrs  :
  case AArch64::ADDWrx  :
  case AArch64::ADDXri  :
  case AArch64::ADDXrs  :
  case AArch64::ADDXrx  :
  case AArch64::ADDXrx64  :
    return true;
  }
}

static inline bool isHalfwordOP(unsigned Opcode) {
  bool Res = false;
  switch (Opcode) {
  default:
    Res = false;
    break;
  case AArch64::STRHHui:
  case AArch64::LDRHHui:
    Res = true;
    break;
  }
  return Res;
}

unsigned AArch64FrameBuilder::getBitCount(unsigned opcode) {
  unsigned ret;

  switch (opcode) {
  default:
    ret = Log2(DLT->getStackAlignment());
    break;
  case AArch64::LDRXui:
  case AArch64::STRXui:
    ret = 8;
    break;
  case AArch64::LDRBBui:
  case AArch64::STRBBui:
    ret = 1;
    break;
  case AArch64::STRHHui:
  case AArch64::LDRHHui:
    ret = 2;
    break;
  case AArch64::ADDWri:
  case AArch64::STRWui:
  case AArch64::LDRWui:

    ret = 4;
    break;
  }

  return ret;
}

Type *AArch64FrameBuilder::getStackType(unsigned size) {
  Type *t = nullptr;

  switch (size) {
  default:
    t = Type::getIntNTy(M->getContext(),
                        M->getDataLayout().getPointerSizeInBits());
    break;
  case 8:
    t = Type::getInt64Ty(*CTX);
    break;
  case 4:
    t = Type::getInt32Ty(*CTX);
    break;
  case 2:
    t = Type::getInt16Ty(*CTX);
    break;
  case 1:
    t = Type::getInt8Ty(*CTX);
    break;
  }

  return t;
}

/// Replace common regs assigned by SP to SP.
/// Patterns like:
/// mov r5, sp
/// ldr r3, [r5, #4]
/// In this case, r5 should be replace by sp.
bool AArch64FrameBuilder::replaceNonSPBySP(MachineInstr &mi) {
  if (mi.getOpcode() == AArch64::ADDXri) {
    if (mi.getOperand(1).isReg() && mi.getOperand(1).getReg() == AArch64::SP) {
      if (mi.getOperand(0).isReg() && mi.getOperand(0).isDef()) {
        RegAssignedBySP.push_back(mi.getOperand(0).getReg());
        return true;
      }
    }
  }

  // Replace regs which are assigned by sp.
  for (MachineOperand &mo : mi.uses()) {
    for (unsigned odx : RegAssignedBySP) {
      if (mo.isReg() && mo.getReg() == odx) {
        mo.ChangeToRegister(AArch64::SP, false);
      }
    }
  }

  // Record regs which are assigned by sp.
  for (MachineOperand &mo : mi.defs()) {
    for (SmallVector<unsigned, 16>::iterator I = RegAssignedBySP.begin();
         I != RegAssignedBySP.end();) {
      if (mo.isReg() && mo.getReg() == *I) {
        RegAssignedBySP.erase(I);
      } else
        ++I;
    }
  }

  return false;
}

/// Analyze frame index of stack operands.
/// Some patterns like:
/// ldr r3, [sp, #12]
/// str r4, [fp, #-8]
/// add r0, sp, #imm
int64_t AArch64FrameBuilder::identifyStackOp(const MachineInstr &mi) {
  unsigned opc = mi.getOpcode();
  if (!isLoadOP(opc) && !isStoreOP(opc) && !isAddOP(opc))
    return -1;

  if (mi.getNumOperands() < 3)
    return -1;

  int64_t offset = -1;
  const MachineOperand &mo = mi.getOperand(1);

  if (!mo.isReg())
    return -1;

  if (isHalfwordOP(opc))
    offset = mi.getOperand(3).getImm();
  else
    offset = mi.getOperand(2).getImm();

  if (mo.getReg() == AArch64::SP && offset >= 0)
    return offset;

  if (mo.getReg() == AArch64::SP) {
    if (offset > 0) {
      if (isHalfwordOP(opc))
        offset = 0 - static_cast<int64_t>(static_cast<int8_t>(offset));
      else
        return -1;
    }
    return MFI->getStackSize() + offset + MFI->getOffsetAdjustment();
  }

  return -1;
}

/// Find out all of frame relative operands, and update them.
void AArch64FrameBuilder::searchStackObjects(MachineFunction &mf) {
  // <SPOffset, frame_element_ptr>
  std::map<int64_t, StackElement *, std::greater<int64_t>> SPOffElementMap;
  DenseMap<MachineInstr *, StackElement *> InstrToElementMap;

  std::vector<MachineInstr *> removelist;
  for (MachineFunction::iterator mbbi = mf.begin(), mbbe = mf.end();
       mbbi != mbbe; ++mbbi) {
    for (MachineBasicBlock::iterator mii = mbbi->begin(), mie = mbbi->end();
         mii != mie; ++mii) {
      MachineInstr &mi = *mii;

      if (replaceNonSPBySP(mi)) {
        removelist.push_back(&mi);
        continue;
      }

      int64_t off = identifyStackOp(mi);
      if (off >= 0) {
        StackElement *se = nullptr;
        if (SPOffElementMap.count(off) == 0) {
          se = new StackElement();
          se->Size = getBitCount(mi.getOpcode());
          se->SPOffset = off;
          SPOffElementMap.insert(std::make_pair(off, se));
        } else {
          se = SPOffElementMap[off];
        }

        if (se != nullptr) {
          InstrToElementMap[&mi] = se;
        }
      }
    }
  }

  // Remove instructions of MOV sp to non-sp.
  for (MachineInstr *mi : removelist)
    mi->removeFromParent();

  // TODO: Before generating StackObjects, we need to check whether there is
  // any missed StackElement.

  BasicBlock *pBB = &getCRF()->getEntryBlock();

  assert(pBB != nullptr && "There is no BasicBlock in this Function!");
  // Generate StackObjects.
  for (auto ii = SPOffElementMap.begin(), ie = SPOffElementMap.end(); ii != ie;
       ++ii) {
    StackElement *sem = ii->second;
    MaybeAlign MALG(sem->Size);
    AllocaInst *alc =
        new AllocaInst(getStackType(sem->Size), 0, nullptr, MALG, "", pBB);
    int idx = MFI->CreateStackObject(sem->Size, 4, false, alc);
    alc->setName("stack." + std::to_string(idx));
    MFI->setObjectOffset(idx, sem->SPOffset);
    sem->ObjectIndex = idx;
  }

  // Replace original SP operands by stack operands.
  for (auto msi = InstrToElementMap.begin(), mse = InstrToElementMap.end();
       msi != mse; ++msi) {
    MachineInstr *pmi = msi->first;
    StackElement *pse = msi->second;
    pmi->getOperand(1).ChangeToFrameIndex(pse->ObjectIndex);
    unsigned opc = pmi->getOpcode();
    if (isHalfwordOP(opc)) {
      pmi->RemoveOperand(3);
    }
    pmi->RemoveOperand(2);
  }

  for (auto &e : SPOffElementMap)
    delete e.second;
}

bool AArch64FrameBuilder::build() {
  if (PrintPass)
    dbgs() << "AArch64FrameBuilder start.\n";

  searchStackObjects(*MF);

  // For debugging.
  if (PrintPass) {
    LLVM_DEBUG(MF->dump());
    LLVM_DEBUG(getCRF()->dump());
    dbgs() << "AArch64FrameBuilder end.\n";
  }

  return true;
}

bool AArch64FrameBuilder::runOnMachineFunction(MachineFunction &mf) {
  bool rtn = false;
  init();
  rtn = build();
  return rtn;
}

#ifdef __cplusplus
extern "C" {
#endif

FunctionPass *InitializeAArch64FrameBuilder(AArch64ModuleRaiser &mr) {
  return new AArch64FrameBuilder(mr);
}

#ifdef __cplusplus
}
#endif
