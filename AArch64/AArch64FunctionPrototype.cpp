//===- AArch64FunctionPrototype.cpp - Binary raiser utility llvm-mctoll -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of AArch64FunctionPrototype class
// for use by llvm-mctoll.
//
//===----------------------------------------------------------------------===//

#include "AArch64FunctionPrototype.h"
#include "AArch64Subtarget.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mctoll"

using namespace llvm;

char AArch64FunctionPrototype::ID = 0;

AArch64FunctionPrototype::AArch64FunctionPrototype() : MachineFunctionPass(ID) {
  PrintPass =
      (cl::getRegisteredOptions()["print-after-all"]->getNumOccurrences() > 0);
}

AArch64FunctionPrototype::~AArch64FunctionPrototype() {}

/// Check the first reference of the reg is USE.
bool AArch64FunctionPrototype::isUsedRegiser(unsigned reg,
                                         const MachineBasicBlock &mbb) {
  for (MachineBasicBlock::const_iterator ii = mbb.begin(), ie = mbb.end();
       ii != ie; ++ii) {
    const MachineInstr &mi = *ii;
    for (MachineInstr::const_mop_iterator oi = mi.operands_begin(),
                                          oe = mi.operands_end();
         oi != oe; oi++) {
      const MachineOperand &mo = *oi;
      if (mo.isReg() && (mo.getReg() == reg))
        return mo.isUse();
    }
  }

  return false;
}
/// calculate framesize by reading first sub command in 1st block. 
void AArch64FunctionPrototype::calculateFrameSize() {
  const MachineBasicBlock &mbb = MF->front();
  for (MachineBasicBlock::const_iterator mii = mbb.begin(), mie = mbb.end();
         mii != mie; ++mii) { 
    const MachineInstr &mi = *mii;
    // sub sp, sp, 0x50
    if (mi.getOpcode() == AArch64::SUBXri && mi.getOperand(0).isReg() &&
        mi.getOperand(1).isReg() && mi.getOperand(2).isImm() &&
        mi.getOperand(0).getReg() == AArch64::SP &&
        mi.getOperand(1).getReg() == AArch64::SP ) {
      this->frameSize = mi.getOperand(2).getImm();
      MF->getFrameInfo().setStackSize(mi.getOperand(2).getImm());
      dbgs() << "frame size of function: " << this->MF->getName().data() << " is " << this->frameSize << "\n";
      //printf("frame size of function: %s is %ld\n", this->MF->getName().data(), this->frameSize);
    } 
    // add x29, sp, 0x40
    if (mi.getOpcode() == AArch64::ADDXri && mi.getOperand(0).isReg() &&
        mi.getOperand(1).isReg() && mi.getOperand(2).isImm() &&
        mi.getOperand(0).getReg() == AArch64::FP &&
        mi.getOperand(1).getReg() == AArch64::SP ) {
      this->fpHeight = mi.getOperand(2).getImm();
    }     
  }  
}
/// loop register to find if register used as parameters
int AArch64FunctionPrototype::getParaOfRegs(
    const MachineBasicBlock *Mbb, unsigned regStart, unsigned regEnd,
    DenseMap<unsigned, bool> &ArgObtain,
    DenseMap<int, Type *> &tarr, int maxidx, Type *paraType) {
  int midx = -1;
  for (unsigned IReg = regStart; IReg <= regEnd; IReg++) {
    if (!ArgObtain[IReg] && Mbb->isLiveIn(IReg)) {
      for (MachineBasicBlock::const_iterator ii = Mbb->begin(),
                                            ie = Mbb->end();
          ii != ie; ++ii) {
        const MachineInstr &LMI = *ii;
        auto RUses = LMI.uses();
        auto ResIter =
            std::find_if(RUses.begin(), RUses.end(),
                        [IReg](const MachineOperand &OP) -> bool {
                          return OP.isReg() && (OP.getReg() == IReg);
                        });
        if (ResIter != RUses.end()) {
          midx = IReg - regStart;
          tarr[midx] = paraType;
          break;
        }
      }
      ArgObtain[IReg] = true;
    }
  }
  if (midx > maxidx) { return midx; }
  else return maxidx;
}
/// Check the first reference of the reg is DEF.
void AArch64FunctionPrototype::genParameterTypes(std::vector<Type *> &paramTypes) {
  this->calculateFrameSize();
  assert(!MF->empty() && "The function body is empty!!!");
  MF->getRegInfo().freezeReservedRegs(*MF);
  LivePhysRegs liveInPhysRegs;
  for (MachineBasicBlock &EMBB : *MF) {
    computeAndAddLiveIns(liveInPhysRegs, EMBB);
    printf("live-in reg in block, %s\n", EMBB.getName().data());
    for (auto liveInReg: EMBB.liveins()) {
       printf("%d ", liveInReg.PhysReg);
    }
    printf("\n");
  }
  // Walk the CFG DFS to discover first register usage
  df_iterator_default_set<const MachineBasicBlock *, 16> Visited;
  DenseMap<unsigned, bool> ArgObtain;
  for (unsigned iR=AArch64::X0; iR <= AArch64::X7; ++iR ) ArgObtain[iR] = false;
  for (unsigned iR=AArch64::W0; iR <= AArch64::W7; ++iR ) ArgObtain[iR] = false;
  for (unsigned iR=AArch64::B0; iR <= AArch64::B7; ++iR ) ArgObtain[iR] = false;
  for (unsigned iR=AArch64::H0; iR <= AArch64::H7; ++iR ) ArgObtain[iR] = false;
  for (unsigned iR=AArch64::D0; iR <= AArch64::D7; ++iR ) ArgObtain[iR] = false;
  for (unsigned iR=AArch64::S0; iR <= AArch64::S7; ++iR ) ArgObtain[iR] = false;
  for (unsigned iR=AArch64::Q0; iR <= AArch64::Q7; ++iR ) ArgObtain[iR] = false;
  const MachineBasicBlock &fmbb = MF->front();
  DenseMap<int, Type *> tarr;
  int maxidx = -1; // When the maxidx is -1, means there is no argument.
  // Track register liveness on CFG.
  printf("start obtaining arguments...\n");
  for (const MachineBasicBlock *Mbb : depth_first_ext(&fmbb, Visited)) {
    maxidx = this->getParaOfRegs(Mbb, (unsigned int)AArch64::X0, (unsigned int)AArch64::X7, ArgObtain, tarr, maxidx, Type::getIntNTy(*CTX, 64));
    maxidx = this->getParaOfRegs(Mbb, (unsigned int)AArch64::W0, (unsigned int)AArch64::W7, ArgObtain, tarr, maxidx, Type::getIntNTy(*CTX, 32));
    maxidx = this->getParaOfRegs(Mbb, (unsigned int)AArch64::B0, (unsigned int)AArch64::B7, ArgObtain, tarr, maxidx, Type::getIntNTy(*CTX, 8));
    maxidx = this->getParaOfRegs(Mbb, (unsigned int)AArch64::H0, (unsigned int)AArch64::H7, ArgObtain, tarr, maxidx, Type::getIntNTy(*CTX, 16));
    maxidx = this->getParaOfRegs(Mbb, (unsigned int)AArch64::D0, (unsigned int)AArch64::D7, ArgObtain, tarr, maxidx, Type::getDoubleTy(*CTX));
    maxidx = this->getParaOfRegs(Mbb, (unsigned int)AArch64::S0, (unsigned int)AArch64::S7, ArgObtain, tarr, maxidx, Type::getFloatTy(*CTX));
    maxidx = this->getParaOfRegs(Mbb, (unsigned int)AArch64::Q0, (unsigned int)AArch64::Q7, ArgObtain, tarr, maxidx, Type::getFP128Ty(*CTX));
  }
  // The rest of function arguments are from stack.
  for (MachineFunction::const_iterator mbbi = MF->begin(), mbbe = MF->end();
       mbbi != mbbe; ++mbbi) {
    const MachineBasicBlock &mbb = *mbbi;
    for (MachineBasicBlock::const_iterator mii = mbb.begin(), mie = mbb.end();
         mii != mie; ++mii) {
      const MachineInstr &mi = *mii;
      // Match pattern like ldr w9, [sp, 0x50]. or ldr w9, [fp, 0x10]
      auto opc = mi.getOpcode();
      if ((opc == AArch64::LDRXui || opc == AArch64::LDRWui ||
           opc == AArch64::LDRHui || opc == AArch64::LDRBui ) 
          && mi.getNumOperands() > 2) {
        const MachineOperand &mo = mi.getOperand(1);
        const MachineOperand &mc = mi.getOperand(2);
        unsigned int regWidth = 8;
        if (opc == AArch64::LDRWui) regWidth = 4;
        if (opc == AArch64::LDRHui) regWidth = 2;
        if (opc == AArch64::LDRBui) regWidth = 1;
        if (mo.isReg() && 
            (mo.getReg() == AArch64::SP || mo.getReg() == AArch64::FP) && 
            mc.isImm()) {
          // TODO: Need to check the imm is larger than 0 and it is align
          // by 4(32 bit).
          int imm = mc.getImm();
          unsigned int stride = regWidth; 
          if ((mo.getReg() == AArch64::SP && (imm*stride) >= this->frameSize) || 
              (mo.getReg() == AArch64::FP && (imm*stride + this->fpHeight >= this->frameSize)) ) {
            int idx = -1 ;
            if (mo.getReg() == AArch64::SP) {
              idx = (imm * stride - this->frameSize)/8 + 8; // eq. ldr w8, [sp+32], and frame size = 32, ==> idx = 0 + 8 = 8
            } else {
              idx = (imm * stride + this->fpHeight - this->frameSize)/8 + 8;
            }
            if (maxidx < idx)
              maxidx = idx;
            tarr[idx] = Type::getIntNTy(*CTX, regWidth * 8);
            // tarr[idx] = getDefaultType();
          }
        }
      }
    }
  }
  dbgs() << "total # of parameters == " << (maxidx+1) << "\n";
  for (int i = 0; i <= maxidx; ++i) {
    if (tarr[i] == nullptr)
      paramTypes.push_back(getDefaultType());
    else
      paramTypes.push_back(tarr[i]);
    // printf("%d : %d\n", i, paramTypes.back()->getTypeID());
  }
}

/// Get all arguments types of current MachineFunction.
bool AArch64FunctionPrototype::isDefinedRegiser(unsigned reg,
                                            const MachineBasicBlock &mbb) {
  for (MachineBasicBlock::const_reverse_iterator ii = mbb.rbegin(),
                                                 ie = mbb.rend();
       ii != ie; ++ii) {
    const MachineInstr &mi = *ii;
    for (MachineInstr::const_mop_iterator oi = mi.operands_begin(),
                                          oe = mi.operands_end();
         oi != oe; oi++) {
      const MachineOperand &mo = *oi;
      if (mo.isReg() && (mo.getReg() == reg)) {
        // The return register must not be tied to another register.
        // If it was, it should not be return register.
        if (mo.isTied())
          return false;

        return mo.isDef();
      }
    }
  }

  return false;
}

/// Get return type of current MachineFunction.
Type *AArch64FunctionPrototype::genReturnType() {
  // TODO: Need to track register liveness on CFG.
  Type *retTy;
  retTy = Type::getVoidTy(*CTX);
  for (const MachineBasicBlock &mbb : *MF) {
    if (mbb.succ_empty()) {
      if (isDefinedRegiser(AArch64::X0, mbb)) {
        retTy = Type::getIntNTy(*CTX, 64);
        break;
      }
      if (isDefinedRegiser(AArch64::W0, mbb)) {
        retTy = Type::getIntNTy(*CTX, 32);
        break;
      }      
      if (isDefinedRegiser(AArch64::H0, mbb)) {
        retTy = Type::getIntNTy(*CTX, 16);
        break;
      }      
      if (isDefinedRegiser(AArch64::B0, mbb)) {
        retTy = Type::getIntNTy(*CTX, 8);
        break;
      }      
      if (isDefinedRegiser(AArch64::S0, mbb)) {
        retTy = Type::getFloatTy(*CTX);
        break;
      }      
      if (isDefinedRegiser(AArch64::D0, mbb)) {
        retTy = Type::getDoubleTy(*CTX);
        break;
      }      
    }        
  }

  return retTy;
}

Function *AArch64FunctionPrototype::discover(MachineFunction &mf) {
  if (PrintPass)
    dbgs() << "AArch64FunctionPrototype start.\n";

  MF = &mf;
  Function &fn = const_cast<Function &>(mf.getFunction());
  CTX = &fn.getContext();
  std::vector<Type *> paramTys;
  genParameterTypes(paramTys);
  Type *retTy = genReturnType();
  FunctionType *fnTy = FunctionType::get(retTy, paramTys, false);

  MachineModuleInfo &mmi = mf.getMMI();
  Module *mdl = const_cast<Module *>(mmi.getModule());
  mdl->getFunctionList().remove(&fn);
  Function *pnfn =
      Function::Create(fnTy, GlobalValue::ExternalLinkage, fn.getName(), mdl);
  // When run as FunctionPass, the Function must not be empty, so add
  // EntryBlock at here.
  BasicBlock::Create(pnfn->getContext(), "EntryBlock", pnfn);

  if (PrintPass) {
    LLVM_DEBUG(MF->dump());
    LLVM_DEBUG(pnfn->dump());
    dbgs() << "AArch64FunctionPrototype end.\n";
  }

  return pnfn;
}

bool AArch64FunctionPrototype::runOnMachineFunction(MachineFunction &mf) {
  discover(mf);
  return true;
}

#ifdef __cplusplus
extern "C" {
#endif

MachineFunctionPass *InitializeAArch64FunctionPrototype() {
  return new AArch64FunctionPrototype();
}

#ifdef __cplusplus
}
#endif
