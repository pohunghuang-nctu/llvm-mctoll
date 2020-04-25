//===- AArch64InstructionSplitting.cpp - Binary raiser utility llvm-mctoll ----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of AArch64InstructionSplitting class
// for use by llvm-mctoll.
//
//===----------------------------------------------------------------------===//

#include "AArch64InstructionSplitting.h"
#include "AArch64InstrInfo.h"
#include "AArch64Subtarget.h"
#include "MCTargetDesc/AArch64AddressingModes.h"
#include "llvm/CodeGen/MachineOperand.h"

#define DEBUG_TYPE "mctoll"

using namespace llvm;

char AArch64InstructionSplitting::ID = 0;

AArch64InstructionSplitting::AArch64InstructionSplitting(AArch64ModuleRaiser &mr)
    : AArch64RaiserBase(ID, mr) {}

AArch64InstructionSplitting::~AArch64InstructionSplitting() {}

void AArch64InstructionSplitting::init(MachineFunction *mf, Function *rf) {
  AArch64RaiserBase::init(mf, rf);
  TII = MF->getSubtarget<AArch64Subtarget>().getInstrInfo();
  MRI = &MF->getRegInfo();
  CTX = &M->getContext();
}

/// Check if the MI has shift pattern.
unsigned AArch64InstructionSplitting::checkisShifter(unsigned Opcode) {
  switch (Opcode) {
    // TODO: PAUL
  /*
  case AArch64::MOVsr:
  case AArch64::MOVsi:
    return AArch64::MOVr;
  case AArch64::ADCrsi:
  case AArch64::ADCrsr:
    return AArch64::ADCrr;
  case AArch64::ADDrsi:
  case AArch64::ADDrsr:
    return AArch64::ADDrr;
  case AArch64::ANDrsi:
  case AArch64::ANDrsr:
    return AArch64::ANDrr;
  case AArch64::BICrsr:
  case AArch64::BICrsi:
    return AArch64::BICrr;
  case AArch64::CMNzrsi:
  case AArch64::CMNzrsr:
    return AArch64::CMNzrr;
  case AArch64::CMPrsi:
  case AArch64::CMPrsr:
    return AArch64::CMPrr;
  case AArch64::EORrsr:
  case AArch64::EORrsi:
    return AArch64::EORrr;
  case AArch64::MVNsr:
  case AArch64::MVNsi:
    return AArch64::MVNr;
  case AArch64::ORRrsi:
  case AArch64::ORRrsr:
    return AArch64::ORRrr;
  case AArch64::RSBrsi:
  case AArch64::RSBrsr:
    return AArch64::RSBrr;
  case AArch64::SUBrsi:
  case AArch64::SUBrsr:
    return AArch64::SUBrr;
  case AArch64::TEQrsr:
  case AArch64::TEQrsi:
    return AArch64::TEQrr;
  case AArch64::TSTrsr:
  case AArch64::TSTrsi:
    return AArch64::TSTrr;
  */
  default:
    return 0;
  }
}

/// If the MI is load/store which needs wback, it will return true.
bool AArch64InstructionSplitting::isLDRSTRPre(unsigned Opcode) {
  switch (Opcode) {
  // TODO: PAUL
  /*  
  case AArch64::LDR_PRE_REG:
  case AArch64::LDR_PRE_IMM:
  case AArch64::LDRB_PRE_REG:
  case AArch64::LDRB_PRE_IMM:
  case AArch64::STR_PRE_REG:
  case AArch64::STR_PRE_IMM:
  case AArch64::STRB_PRE_REG:
  case AArch64::STRB_PRE_IMM:
    return true;
  */
  default:
    return false;
  }
}

/// No matter what pattern of Load/Store is, change the Opcode to xxxi12.
unsigned AArch64InstructionSplitting::getLoadStoreOpcode(unsigned Opcode) {
  switch (Opcode) {
    // TODO: PAUL
  /*
  case AArch64::LDRrs:
  case AArch64::LDRi12:
  case AArch64::LDR_PRE_REG:
  case AArch64::LDR_PRE_IMM:
    return AArch64::LDRi12;
  case AArch64::LDRBrs:
  case AArch64::LDRBi12:
  case AArch64::LDRB_PRE_REG:
  case AArch64::LDRB_PRE_IMM:
    return AArch64::LDRBi12;
  case AArch64::STRrs:
  case AArch64::STRi12:
  case AArch64::STR_PRE_REG:
  case AArch64::STR_PRE_IMM:
    return AArch64::STRi12;
  case AArch64::STRBrs:
  case AArch64::STRBi12:
  case AArch64::STRB_PRE_REG:
  case AArch64::STRB_PRE_IMM:
    return AArch64::STRBi12;
  */
  default:
    return 0;
  }
}

/// True if the AArch64 instruction performs Shift_C().
bool AArch64InstructionSplitting::isShift_C(unsigned Opcode) {
  switch (Opcode) {
// TODO: PAUL
  /*    
  case AArch64::ANDrsr:
  case AArch64::ANDrsi:
  case AArch64::BICrsr:
  case AArch64::BICrsi:
  case AArch64::EORrsr:
  case AArch64::EORrsi:
  case AArch64::MVNsr:
  case AArch64::MVNsi:
  case AArch64::ORRrsr:
  case AArch64::ORRrsi:
  case AArch64::TEQrsr:
  case AArch64::TEQrsi:
  case AArch64::TSTrsr:
  case AArch64::TSTrsi:
    return true;
  */
  default:
    return false;
  }
}

/// Get the shift opcode in MI.
unsigned AArch64InstructionSplitting::getShiftOpcode(AArch64_AM::ShiftExtendType SOpc,
                                                 unsigned OffSet) {
  switch (SOpc) {
  case AArch64_AM::ASR: {
      // TODO: PAUL
  }
  case AArch64_AM::LSL: {
    // TODO: PAUL
  }
  case AArch64_AM::LSR: {
    // TODO: PAUL
  }
  case AArch64_AM::ROR: {
    // TODO: PAUL
  }
  case AArch64_AM::MSL:
    // TODO: PAUL
  case AArch64_AM::UXTB:
    // TODO: PAUL
  case AArch64_AM::UXTH:
    // TODO: PAUL
  case AArch64_AM::UXTW:
    // TODO: PAUL 
  case AArch64_AM::UXTX:
    // TODO: PAUL
  case AArch64_AM::SXTB:
    // TODO: PAUL 
  case AArch64_AM::SXTH:
    // TODO: PAUL 
  case AArch64_AM::SXTW:
    // TODO: PAUL
  case AArch64_AM::SXTX:
    // TODO: PAUL       
  case AArch64_AM::InvalidShiftExtend:
  default:
    return 0;
  }
}

MachineInstrBuilder &
AArch64InstructionSplitting::addOperand(MachineInstrBuilder &mib,
                                    MachineOperand &mo, bool isDef) {
  switch (mo.getType()) {
  default:
    assert(false && "Unsupported MachineOperand type!");
    break;
  case MachineOperand::MO_Register: {
    if (isDef)
      mib.addDef(mo.getReg());
    else
      mib.addUse(mo.getReg());
  } break;
  case MachineOperand::MO_FrameIndex: {
    mib.addFrameIndex(mo.getIndex());
  } break;
  }

  return mib;
}

/// Split LDRxxx/STRxxx<c><q> <Rt>, [<Rn>, #+/-<imm>]! to:
/// ADD Rn, Rn, #imm
/// LDRxxx/STRxxx Rt, [Rn]
MachineInstr *AArch64InstructionSplitting::splitLDRSTRPreImm(MachineBasicBlock &MBB,
                                                         MachineInstr &MI) {
  MachineOperand &Rd = MI.getOperand(0);
  MachineOperand &Rn = MI.getOperand(1);
  MachineOperand &Rm = MI.getOperand(2);
  MachineOperand &Rs = MI.getOperand(3);

  // MI is splitted into 2 instructions.
  // So get Metadata for the first instruction.
  ConstantAsMetadata *CMD_fst = ConstantAsMetadata::get(
      ConstantInt::get(*CTX, llvm::APInt(64, 0, false)));
  MDNode *N_fst = MDNode::get(*CTX, CMD_fst);

  // Get Metadata for the second instruction.
  ConstantAsMetadata *CMD_sec = ConstantAsMetadata::get(
      ConstantInt::get(*CTX, llvm::APInt(64, 1, false)));
  MDNode *N_sec = MDNode::get(*CTX, CMD_sec);

  unsigned newOpc = getLoadStoreOpcode(MI.getOpcode());
  // Add Rm,[Rm, #imm]!
  MachineInstrBuilder fst =
      BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(AArch64::ADDXri));
  addOperand(fst, Rm, true);
  addOperand(fst, Rm);
  fst.addImm(Rs.getImm());

  MachineInstrBuilder sec =
      BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
  if (MI.mayStore())
    // STRxxx Rn, [Rm]
    addOperand(sec, Rn);
  else if (MI.mayLoad())
    // LDRxxx Rd, [Rm]
    addOperand(sec, Rd, true);
  addOperand(sec, Rm);

  int idx = MI.findRegisterUseOperandIdx(AArch64::NZCV);
  // Add CPSR if the MI has.
  if (idx != -1) {
    fst.addImm(MI.getOperand(idx - 1).getImm());
    addOperand(fst, MI.getOperand(idx));
    sec.addImm(MI.getOperand(idx - 1).getImm());
    addOperand(sec, MI.getOperand(idx));
  }
  fst.addMetadata(N_fst);
  sec.addMetadata(N_sec);
  return &MI;
}

/// Split LDRxxx/STRxxx<c><q> <Rt>, [<Rn>, +/-<Rm>{, <shift>}]! to:
/// Rm shift #imm, but write result to VReg.
/// Add Rn, Rm
/// LDRxxx/STRxxx Rt, [Rn]
MachineInstr *AArch64InstructionSplitting::splitLDRSTRPre(MachineBasicBlock &MBB,
                                                      MachineInstr &MI) {
  // TODO: PAUL
  /*
  unsigned Simm = MI.getOperand(4).getImm();
  unsigned SOffSet = AArch64_AM::getAM2Offset(Simm);
  AArch64_AM::ShiftOpc SOpc = AArch64_AM::getAM2ShiftOpc(Simm);
  unsigned SVReg = MRI->createVirtualRegister(&AArch64::GPRnopcRegClass);

  MachineOperand &Rd = MI.getOperand(0);
  MachineOperand &Rn = MI.getOperand(1);
  MachineOperand &Rm = MI.getOperand(2);
  MachineOperand &Rs = MI.getOperand(3);
  unsigned ShiftOpc = getShiftOpcode(SOpc, SOffSet);

  // Get Metadata for the first instruction.
  ConstantAsMetadata *CMD_fst = ConstantAsMetadata::get(
      ConstantInt::get(*CTX, llvm::APInt(64, 0, false)));
  MDNode *N_fst = MDNode::get(*CTX, CMD_fst);

  // Get Metadata for the second instruction.
  ConstantAsMetadata *CMD_sec = ConstantAsMetadata::get(
      ConstantInt::get(*CTX, llvm::APInt(64, 1, false)));
  MDNode *N_sec = MDNode::get(*CTX, CMD_sec);

  // Get Metadata for the third instruction.
  ConstantAsMetadata *CMD_thd = ConstantAsMetadata::get(
      ConstantInt::get(*CTX, llvm::APInt(64, 2, false)));
  MDNode *N_thd = MDNode::get(*CTX, CMD_thd);

  unsigned newOpc = getLoadStoreOpcode(MI.getOpcode());
  int idx = MI.findRegisterUseOperandIdx(AArch64::CPSR);
  if (SOffSet > 0) {
    // LDRxxx/STRxxx<c><q> <Rt>, [<Rn>, +/-<Rm>{, <shift>}]!

    // Rs shift #imm and write result to VReg.
    MachineInstrBuilder fst =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), SVReg);
    addOperand(fst, Rs);
    fst.addImm(SOffSet);

    // Add Rm, VReg
    MachineInstrBuilder sec =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(AArch64::ADDrr));
    addOperand(sec, Rm, true);
    addOperand(sec, Rm);
    sec.addReg(SVReg);

    MachineInstrBuilder thd =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
    if (MI.mayStore())
      // STRxxx Rn, [Rm]
      addOperand(thd, Rn);
    else if (MI.mayLoad())
      // LDRxxx Rd, [Rm]
      addOperand(thd, Rd, true);
    addOperand(thd, Rm);

    // Add CPSR if the MI has.
    if (idx != -1) {
      fst.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(fst, MI.getOperand(idx));
      sec.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(sec, MI.getOperand(idx));
      thd.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(thd, MI.getOperand(idx));
    }
    fst.addMetadata(N_fst);
    sec.addMetadata(N_sec);
    thd.addMetadata(N_thd);
  } else if (ShiftOpc == AArch64::RRX) {
    // Split LDRxxx/STRxxx<c><q> <Rt>, [<Rn>, +/-<Rm>, RRX]!
    MachineInstrBuilder fst =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), SVReg);
    addOperand(fst, Rs);

    MachineInstrBuilder sec =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(AArch64::ADDrr));
    addOperand(sec, Rm, true);
    addOperand(sec, Rm);
    sec.addReg(SVReg);

    MachineInstrBuilder thd =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
    if (MI.mayStore())
      addOperand(thd, Rn);
    else if (MI.mayLoad())
      addOperand(thd, Rd, true);
    addOperand(thd, Rm);

    // Add CPSR if the MI has.
    if (idx != -1) {
      sec.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(sec, MI.getOperand(idx));
      thd.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(thd, MI.getOperand(idx));
    }
    fst.addMetadata(N_fst);
    sec.addMetadata(N_sec);
    thd.addMetadata(N_thd);
  } else {
    // Split LDRxxx/STRxxx<c><q> <Rt>, [<Rn>, +/-<Rm>]!
    MachineInstrBuilder fst =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(AArch64::ADDrr));
    addOperand(fst, Rm, true);
    addOperand(fst, Rm);
    addOperand(fst, Rs);

    MachineInstrBuilder sec =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
    if (MI.mayStore())
      addOperand(sec, Rn);
    else if (MI.mayLoad())
      addOperand(sec, Rd, true);
    addOperand(sec, Rm);

    // Add CPSR if the MI has.
    if (idx != -1) {
      fst.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(fst, MI.getOperand(idx));
      sec.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(sec, MI.getOperand(idx));
    }
    fst.addMetadata(N_fst);
    sec.addMetadata(N_sec);
  }
  */
  return &MI;
}

/// Split LDRxxx/STRxxx<c><q> <Rd>, [<Rn>, +/-<#imm>] to:
/// Add VReg, Rn, #imm
/// LDRxxx/STRxxx Rd, [VReg]
MachineInstr *AArch64InstructionSplitting::splitLDRSTRImm(MachineBasicBlock &MBB,
                                                      MachineInstr &MI) {
  unsigned VReg = MRI->createVirtualRegister(&AArch64::GPR64allRegClass);
  MachineOperand &Rd = MI.getOperand(0);
  MachineOperand &Rn = MI.getOperand(1);
  MachineOperand &Rm = MI.getOperand(2);

  // The MI is splitted into 2 instructions.
  // Get Metadata for the first instruction.
  ConstantAsMetadata *CMD_fst = ConstantAsMetadata::get(
      ConstantInt::get(*CTX, llvm::APInt(64, 0, false)));
  MDNode *N_fst = MDNode::get(*CTX, CMD_fst);

  // Get Metadata for the first instruction.
  ConstantAsMetadata *CMD_sec = ConstantAsMetadata::get(
      ConstantInt::get(*CTX, llvm::APInt(64, 1, false)));
  MDNode *N_sec = MDNode::get(*CTX, CMD_sec);

  unsigned newOpc = getLoadStoreOpcode(MI.getOpcode());
  // Add VReg, Rn, #imm
  MachineInstrBuilder fst =
      BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(AArch64::ADDXri), VReg);
  addOperand(fst, Rn);
  fst.addImm(Rm.getImm());

  // LDRxxx/STRxxx Rd, [VReg]
  MachineInstrBuilder sec =
      BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
  if (MI.mayStore())
    addOperand(sec, Rd);
  else
    addOperand(sec, Rd, true);
  sec.addReg(VReg);

  int idx = MI.findRegisterUseOperandIdx(AArch64::NZCV);
  // Add CPSR if the MI has.
  if (idx != -1) {
    fst.addImm(MI.getOperand(idx - 1).getImm());
    addOperand(fst, MI.getOperand(idx));
    sec.addImm(MI.getOperand(idx - 1).getImm());
    addOperand(sec, MI.getOperand(idx));
  }
  fst.addMetadata(N_fst);
  sec.addMetadata(N_sec);
  return &MI;
}

/// Split LDRxxx/STRxxx<c><q> <Rd>, [<Rn>, +/-<Rm>{, <shift>}] to:
/// Rm shift #imm, but write result to VReg.
/// Add VReg, Rn, Rm
/// LDRxxx/STRxxx Rd, [VReg]
MachineInstr *AArch64InstructionSplitting::splitLDRSTR(MachineBasicBlock &MBB,
                                                   MachineInstr &MI) {
  // TODO: PAUL
  /*
  unsigned Simm = MI.getOperand(3).getImm();
  unsigned SOffSet = AArch64_AM::getAM2Offset(Simm);
  AArch64_AM::ShiftOpc SOpc = AArch64_AM::getAM2ShiftOpc(Simm);
  unsigned SVReg = MRI->createVirtualRegister(&AArch64::GPRnopcRegClass);
  unsigned AVReg = MRI->createVirtualRegister(&AArch64::GPRnopcRegClass);

  MachineOperand &Rd = MI.getOperand(0);
  MachineOperand &Rn = MI.getOperand(1);
  MachineOperand &Rm = MI.getOperand(2);
  unsigned ShiftOpc = getShiftOpcode(SOpc, SOffSet);

  // Get Metadata for the fisrt insturction.
  ConstantAsMetadata *CMD_fst = ConstantAsMetadata::get(
      ConstantInt::get(*CTX, llvm::APInt(64, 0, false)));
  MDNode *N_fst = MDNode::get(*CTX, CMD_fst);

  // Get Metadata for the second insturction.
  ConstantAsMetadata *CMD_sec = ConstantAsMetadata::get(
      ConstantInt::get(*CTX, llvm::APInt(64, 1, false)));
  MDNode *N_sec = MDNode::get(*CTX, CMD_sec);

  // Get Metadata for the third insturction.
  ConstantAsMetadata *CMD_thd = ConstantAsMetadata::get(
      ConstantInt::get(*CTX, llvm::APInt(64, 2, false)));
  MDNode *N_thd = MDNode::get(*CTX, CMD_thd);

  unsigned newOpc = getLoadStoreOpcode(MI.getOpcode());
  int idx = MI.findRegisterUseOperandIdx(AArch64::CPSR);
  if (SOffSet > 0) {
    // Split LDRxxx/STRxxx Rd, [Rn, Rm, shift]
    MachineInstrBuilder fst =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), SVReg);
    addOperand(fst, Rm);
    fst.addImm(SOffSet);

    MachineInstrBuilder sec =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(AArch64::ADDrr), AVReg);
    addOperand(sec, Rn);
    sec.addReg(SVReg);

    MachineInstrBuilder thd =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
    if (MI.mayStore())
      addOperand(thd, Rd);
    else
      addOperand(thd, Rd, true);
    thd.addReg(AVReg);
    // Add CPSR if the MI has.
    if (idx != -1) {
      fst.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(fst, MI.getOperand(idx));
      sec.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(sec, MI.getOperand(idx));
      thd.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(thd, MI.getOperand(idx));
    }
    fst.addMetadata(N_fst);
    sec.addMetadata(N_sec);
    thd.addMetadata(N_thd);
  } else if (ShiftOpc == AArch64::RRX) {
    // Split LDRxxx/STRxxx Rd, [Rn, Rm, rrx]
    MachineInstrBuilder fst =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), SVReg);
    addOperand(fst, Rm);

    MachineInstrBuilder sec =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(AArch64::ADDrr), AVReg);
    addOperand(sec, Rn);
    sec.addReg(SVReg);

    MachineInstrBuilder thd =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
    if (MI.mayStore())
      addOperand(thd, Rd);
    else
      addOperand(thd, Rd, true);
    thd.addReg(AVReg);
    // Add CPSR if the MI has.
    if (idx != -1) {
      sec.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(sec, MI.getOperand(idx));
      thd.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(thd, MI.getOperand(idx));
    }
    fst.addMetadata(N_fst);
    sec.addMetadata(N_sec);
    thd.addMetadata(N_thd);
  } else {
    // Split LDRxxx/STRxxx Rd, [Rn, Rm]
    MachineInstrBuilder fst =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(AArch64::ADDrr), AVReg);
    addOperand(fst, Rn);
    addOperand(fst, Rm);

    MachineInstrBuilder sec =
        BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
    if (MI.mayStore())
      addOperand(sec, Rd);
    else
      addOperand(sec, Rd, true);
    sec.addReg(AVReg);
    // Add CPSR if the MI has.
    if (idx != -1) {
      fst.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(fst, MI.getOperand(idx));
      sec.addImm(MI.getOperand(idx - 1).getImm());
      addOperand(sec, MI.getOperand(idx));
    }
    fst.addMetadata(N_fst);
    sec.addMetadata(N_sec);
  }
  */
  return &MI;
}

/// Split 'Opcode Rd, Rn, Rm, shift' except LDRxxx/STRxxx.
MachineInstr *AArch64InstructionSplitting::splitCommon(MachineBasicBlock &MBB,
                                                   MachineInstr &MI,
                                                   unsigned newOpc) {
  MachineInstr *mi = nullptr;
  // TODO: PAUL
  /*
  for (unsigned i = 0; i < MI.getNumOperands(); i++) {
    if (MI.getOperand(i).isImm()) {
      unsigned Simm = MI.getOperand(i).getImm();
      unsigned SOffSet = AArch64_AM::getSORegOffset(Simm);
      AArch64_AM::ShiftOpc SOpc = AArch64_AM::getSORegShOp(Simm);
      unsigned ShiftOpc = getShiftOpcode(SOpc, SOffSet);

      unsigned VReg = MRI->createVirtualRegister(&AArch64::GPRnopcRegClass);
      if (ShiftOpc) {
        MachineOperand &Rd = MI.getOperand(0);
        MachineOperand &Rn = MI.getOperand(i - 2);
        MachineOperand &Rm = MI.getOperand(i - 1);

        ConstantAsMetadata *CMD_fst = ConstantAsMetadata::get(
            ConstantInt::get(*CTX, llvm::APInt(64, 0, false)));
        MDNode *N_fst = MDNode::get(*CTX, CMD_fst);

        ConstantAsMetadata *CMD_sec = ConstantAsMetadata::get(
            ConstantInt::get(*CTX, llvm::APInt(64, 1, false)));
        MDNode *N_sec = MDNode::get(*CTX, CMD_sec);

        if (SOffSet) {
          // Split Opcode Rd, Rn, Rm, shift #imm

          // Rm shifts SOffset and writes result to VReg.
          MachineInstrBuilder fst =
              BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
          addOperand(fst, Rm);
          fst.addImm(SOffSet);
          fst.addMetadata(N_fst);

          // Build 'opcode Rd, Rn, VReg'
          MachineInstrBuilder sec =
              BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
          addOperand(sec, Rd, true);
          for (unsigned n = 1; n < (i - 1); n++) {
            addOperand(sec, MI.getOperand(n));
          }
          sec.addReg(VReg);
          sec.addMetadata(N_sec);
        } else {
          if (ShiftOpc == AArch64::RRX) {
            // Split 'opcode Rd, Rn, Rm, RRX'
            MachineInstrBuilder fst =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
            addOperand(fst, Rm);
            fst.addMetadata(N_fst);

            MachineInstrBuilder sec =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
            addOperand(sec, Rd, true);

            for (unsigned n = 1; n < i - 1; n++) {
              addOperand(sec, MI.getOperand(n));
            }
            sec.addReg(VReg);
            sec.addMetadata(N_sec);
          } else {
            // Split 'opcode Rd, Rn, Rm, shift Rs'

            // Build 'ShiftOpc VReg, Rn, Rm'
            MachineInstrBuilder fst =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
            addOperand(fst, Rn);
            addOperand(fst, Rm);
            fst.addMetadata(N_fst);

            // Build 'opcode Rd, Rn, VReg'
            MachineInstrBuilder sec =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
            addOperand(sec, Rd, true);

            for (unsigned n = 1; n < (i - 2); n++) {
              addOperand(sec, MI.getOperand(n));
            }
            sec.addReg(VReg);
            sec.addMetadata(N_sec);
          }
        }
        mi = &MI;
        break;
      }
    }
  }
  */
  return mi;
}

/// Split 'opcode<s> Rd, Rn, Rm, shift' except LDRxxx/STRxxx.
MachineInstr *AArch64InstructionSplitting::splitS(MachineBasicBlock &MBB,
                                              MachineInstr &MI, unsigned newOpc,
                                              int idx) {
  MachineInstr *mi = nullptr;
  // TODO: PAUL
  /*
  for (unsigned i = 0; i < MI.getNumOperands(); i++) {
    if (MI.getOperand(i).isImm()) {
      unsigned Simm = MI.getOperand(i).getImm();
      unsigned SOffSet = AArch64_AM::getSORegOffset(Simm);
      AArch64_AM::ShiftOpc SOpc = AArch64_AM::getSORegShOp(Simm);
      unsigned ShiftOpc = getShiftOpcode(SOpc, SOffSet);
      unsigned VReg = MRI->createVirtualRegister(&AArch64::GPRnopcRegClass);

      if (ShiftOpc) {
        ConstantAsMetadata *CMD_fst = ConstantAsMetadata::get(
            ConstantInt::get(*CTX, llvm::APInt(64, 0, false)));
        MDNode *N_fst = MDNode::get(*CTX, CMD_fst);

        ConstantAsMetadata *CMD_sec = ConstantAsMetadata::get(
            ConstantInt::get(*CTX, llvm::APInt(64, 1, false)));
        MDNode *N_sec = MDNode::get(*CTX, CMD_sec);

        MachineOperand &Rd = MI.getOperand(0);
        MachineOperand &Rn = MI.getOperand(i - 2);
        MachineOperand &Rm = MI.getOperand(i - 1);

        // C flag is affected by Shift_c() if isShift_C is true.
        if (isShift_C(MI.getOpcode())) {
          if (SOffSet) {
            // Split opcode<s> Rd, Rn, Rm, shift #imm.

            // Rm shift #imm and  the new MI updates CPSR.
            MachineInstrBuilder fst =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
            addOperand(fst, Rm);
            fst.addImm(SOffSet);
            fst.addImm(AArch64CC::AL);
            addOperand(fst, MI.getOperand(idx));
            fst.addMetadata(N_fst);

            // Build 'opcode<s> Rd, Rn, VReg'
            // The new MI updates CPSR.
            MachineInstrBuilder sec =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
            addOperand(sec, Rd, true);
            for (unsigned n = 1; n < (i - 1); n++) {
              addOperand(sec, MI.getOperand(n));
            }
            sec.addReg(VReg);
            sec.addImm(AArch64CC::AL);
            addOperand(sec, MI.getOperand(idx));
            sec.addMetadata(N_sec);
          } else {
            if (ShiftOpc == AArch64::RRX) {
              // Split opcode<s> Rd, Rn, Rm, RRX.
              MachineInstrBuilder fst =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
              addOperand(fst, Rm);
              fst.addMetadata(N_fst);
              // XXX: RRX implicit CPSR, how to add cpsr?

              // Build base instructions
              MachineInstrBuilder sec =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
              addOperand(sec, Rd, true);

              for (unsigned n = 1; n < (i - 1); n++) {
                addOperand(sec, MI.getOperand(n));
              }
              sec.addReg(VReg);
              sec.addImm(AArch64CC::AL);
              addOperand(sec, MI.getOperand(idx));
              sec.addMetadata(N_sec);
            } else {
              // Split opcode<s> Rd, Rn, Rm, shift Rs.
              // The new MI updates CPSR.
              MachineInstrBuilder fst =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
              addOperand(fst, Rn);
              addOperand(fst, Rm);
              fst.addImm(AArch64CC::AL);
              addOperand(fst, MI.getOperand(idx));
              fst.addMetadata(N_fst);

              MachineInstrBuilder sec =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
              addOperand(sec, Rd, true);

              for (unsigned n = 1; n < (i - 2); n++) {
                addOperand(sec, MI.getOperand(n));
              }
              sec.addReg(VReg);
              sec.addImm(AArch64CC::AL);
              addOperand(sec, MI.getOperand(idx));
              sec.addMetadata(N_sec);
            }
          }
        } else {
          if (SOffSet) {
            // Split opcode<s> Rd, Rn, Rm, shift #imm.

            // Rm shift #imm,  and the new MI doesn't update CPSR.
            MachineInstrBuilder fst =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
            addOperand(fst, Rm);
            fst.addImm(SOffSet);
            fst.addMetadata(N_fst);

            // Build 'opcode<s> Rd, Rn, VReg'
            // The new MI updates CPSR.
            MachineInstrBuilder sec =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
            addOperand(sec, Rd, true);
            for (unsigned n = 1; n < (i - 1); n++) {
              addOperand(sec, MI.getOperand(n));
            }
            sec.addReg(VReg);
            sec.addImm(AArch64CC::AL);
            addOperand(sec, MI.getOperand(idx));
            sec.addMetadata(N_sec);
          } else {
            if (ShiftOpc == AArch64::RRX) {
              // Split opcode<s> Rd, Rn, Rm, rrx.
              MachineInstrBuilder fst =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
              addOperand(fst, Rm);
              fst.addMetadata(N_fst);
              // RRX implicit CPSR, how to add cpsr?

              MachineInstrBuilder sec =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
              addOperand(sec, Rd, true);

              for (unsigned n = 1; n < (i - 1); n++) {
                addOperand(sec, MI.getOperand(n));
              }
              sec.addReg(VReg);
              sec.addImm(AArch64CC::AL);
              addOperand(sec, MI.getOperand(idx));
              sec.addMetadata(N_sec);
            } else {
              // Split opcode<s> Rd, Rn, Rm, shift Rs.

              // Rm shift reg,  and the new MI doesn't update CPSR.
              MachineInstrBuilder fst =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
              addOperand(fst, Rn);
              addOperand(fst, Rm);
              fst.addMetadata(N_fst);

              // Build 'opcode<s> Rd, Rn, VReg'
              // The new MI updates CPSR.
              MachineInstrBuilder sec =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
              addOperand(sec, Rd, true);

              for (unsigned n = 1; n < (i - 2); n++) {
                addOperand(sec, MI.getOperand(n));
              }
              sec.addReg(VReg);
              sec.addImm(AArch64CC::AL);
              addOperand(sec, MI.getOperand(idx));
              sec.addMetadata(N_sec);
            }
          }
        }
        mi = &MI;
        break;
      }
    }
  }
  */
  return mi;
}

/// Split 'opcode<c> Rd, Rn, Rm, shift' except LDRxxx/STRxxx.
MachineInstr *AArch64InstructionSplitting::splitC(MachineBasicBlock &MBB,
                                              MachineInstr &MI, unsigned newOpc,
                                              int idx) {
  MachineInstr *mi = nullptr;
  // TODO: PAUL
  /*
  for (unsigned i = 0; i < MI.getNumOperands(); i++) {
    if (MI.getOperand(i).isImm()) {
      unsigned Simm = MI.getOperand(i).getImm();
      unsigned SOffSet = AArch64_AM::getSORegOffset(Simm);
      AArch64_AM::ShiftOpc SOpc = AArch64_AM::getSORegShOp(Simm);
      unsigned ShiftOpc = getShiftOpcode(SOpc, SOffSet);
      unsigned VReg = MRI->createVirtualRegister(&AArch64::GPRnopcRegClass);

      if (ShiftOpc) {
        MachineOperand &Rd = MI.getOperand(0);
        MachineOperand &Rn = MI.getOperand(i - 2);
        MachineOperand &Rm = MI.getOperand(i - 1);

        ConstantAsMetadata *CMD_fst = ConstantAsMetadata::get(
            ConstantInt::get(*CTX, llvm::APInt(64, 0, false)));
        MDNode *N_fst = MDNode::get(*CTX, CMD_fst);

        ConstantAsMetadata *CMD_sec = ConstantAsMetadata::get(
            ConstantInt::get(*CTX, llvm::APInt(64, 1, false)));
        MDNode *N_sec = MDNode::get(*CTX, CMD_sec);

        if (SOffSet) {
          // Split opcode<c> Rd, Rn, Rm, shift #imm
          // The new MI checks CondCode.

          MachineInstrBuilder fst =
              BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
          addOperand(fst, Rm);
          fst.addImm(SOffSet);
          fst.addImm(MI.getOperand(idx - 1).getImm());
          addOperand(fst, MI.getOperand(idx));
          fst.addMetadata(N_fst);

          MachineInstrBuilder sec =
              BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
          addOperand(sec, Rd, true);
          for (unsigned n = 1; n < (i - 1); n++) {
            addOperand(sec, MI.getOperand(n));
          }
          sec.addReg(VReg);
          sec.addImm(MI.getOperand(idx - 1).getImm());
          addOperand(sec, MI.getOperand(idx));
          sec.addMetadata(N_sec);
        } else {
          if (ShiftOpc == AArch64::RRX) {
            // Split opcode<c> Rd, Rn, Rm, RRX
            MachineInstrBuilder fst =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
            addOperand(fst, Rm);
            fst.addMetadata(N_fst);
            // XXX: RRX implicit CPSR, how to add cpsr?

            // Build base instructions
            MachineInstrBuilder sec =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
            addOperand(sec, Rd, true);

            for (unsigned n = 1; n < (i - 1); n++) {
              addOperand(sec, MI.getOperand(n));
            }
            sec.addReg(VReg);
            sec.addImm(MI.getOperand(idx - 1).getImm());
            addOperand(sec, MI.getOperand(idx));
            sec.addMetadata(N_sec);
          } else {
            // Split opcode<c> Rd, Rn, Rm, shift Rs
            // The new MI checks CondCode.

            MachineInstrBuilder fst =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
            addOperand(fst, Rn);
            addOperand(fst, Rm);
            fst.addImm(MI.getOperand(idx - 1).getImm());
            addOperand(fst, MI.getOperand(idx));
            fst.addMetadata(N_fst);

            MachineInstrBuilder sec =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
            addOperand(sec, Rd, true);

            for (unsigned n = 1; n < (i - 2); n++) {
              addOperand(sec, MI.getOperand(n));
            }
            sec.addReg(VReg);
            sec.addImm(MI.getOperand(idx - 1).getImm());
            addOperand(sec, MI.getOperand(idx));
            sec.addMetadata(N_sec);
          }
        }
        mi = &MI;
        break;
      }
    }
  }
  */
  return mi;
}

/// Split 'opcode<s><c> Rd, Rn, Rm, shift' except LDRxxx/STRxxx.
MachineInstr *AArch64InstructionSplitting::splitCS(MachineBasicBlock &MBB,
                                               MachineInstr &MI,
                                               unsigned newOpc, int idx) {
  MachineInstr *mi = nullptr;
  for (unsigned i = 0; i < MI.getNumOperands(); i++) {
    if (MI.getOperand(i).isImm()) {
      unsigned Simm = MI.getOperand(i).getImm();
      unsigned SOffSet = AArch64_AM::getShiftValue(Simm);
      AArch64_AM::ShiftExtendType SOpc = AArch64_AM::getShiftType(Simm);
      unsigned ShiftOpc = getShiftOpcode(SOpc, SOffSet);
      unsigned VReg = MRI->createVirtualRegister(&AArch64::GPR64allRegClass);

      if (ShiftOpc) {
        MachineOperand &Rd = MI.getOperand(0);
        MachineOperand &Rn = MI.getOperand(i - 2);
        MachineOperand &Rm = MI.getOperand(i - 1);

        ConstantAsMetadata *CMD_fst = ConstantAsMetadata::get(
            ConstantInt::get(*CTX, llvm::APInt(64, 0, false)));
        MDNode *N_fst = MDNode::get(*CTX, CMD_fst);

        ConstantAsMetadata *CMD_sec = ConstantAsMetadata::get(
            ConstantInt::get(*CTX, llvm::APInt(64, 1, false)));
        MDNode *N_sec = MDNode::get(*CTX, CMD_sec);

        // C flag is affected by Shift_c() if isShift_C is true.
        if (isShift_C(MI.getOpcode())) {
          if (SOffSet) {
            // Split opcode<s><c> Rd, Rn, Rm, shift #imm

            // The new MI both updates CPSR and checks CondCode.
            MachineInstrBuilder fst =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
            addOperand(fst, Rm);
            fst.addImm(SOffSet);
            fst.addImm(MI.getOperand(idx - 1).getImm());
            addOperand(fst, MI.getOperand(idx));
            addOperand(fst, MI.getOperand(idx + 1));
            fst.addMetadata(N_fst);

            MachineInstrBuilder sec =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
            addOperand(sec, Rd, true);
            for (unsigned n = 1; n < (i - 1); n++) {
              addOperand(sec, MI.getOperand(n));
            }
            sec.addReg(VReg);
            sec.addImm(MI.getOperand(idx - 1).getImm());
            addOperand(sec, MI.getOperand(idx));
            addOperand(sec, MI.getOperand(idx + 1));
            sec.addMetadata(N_sec);
          } /* TODO: Paul
          else {
            
            if (ShiftOpc == AArch64::RRX) {
              // Split opcode<s><c> Rd, Rn, Rm, RRX
              MachineInstrBuilder fst =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
              addOperand(fst, Rm);
              fst.addMetadata(N_fst);
              // RRX implicit CPSR, how to add cpsr?

              MachineInstrBuilder sec =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
              addOperand(sec, Rd, true);

              for (unsigned n = 1; n < (i - 1); n++) {
                addOperand(sec, MI.getOperand(n));
              }
              sec.addReg(VReg);
              sec.addImm(MI.getOperand(idx - 1).getImm());
              addOperand(sec, MI.getOperand(idx));
              addOperand(sec, MI.getOperand(idx + 1));
              sec.addMetadata(N_sec);
            } else {
              // Split opcode<s><c> Rd, Rn, Rm, shift Rs

              // The new MI both updates CPSR and checks CondCode.
              MachineInstrBuilder fst =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
              addOperand(fst, Rn);
              addOperand(fst, Rm);
              fst.addImm(MI.getOperand(idx - 1).getImm());
              addOperand(fst, MI.getOperand(idx));
              addOperand(fst, MI.getOperand(idx + 1));
              fst.addMetadata(N_fst);

              MachineInstrBuilder sec =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
              addOperand(sec, Rd, true);

              for (unsigned n = 1; n < (i - 2); n++) {
                addOperand(sec, MI.getOperand(n));
              }
              sec.addReg(VReg);
              sec.addImm(MI.getOperand(idx - 1).getImm());
              addOperand(sec, MI.getOperand(idx));
              addOperand(sec, MI.getOperand(idx + 1));
              sec.addMetadata(N_sec);
            }
            
          }*/
        } else {
          // Shifter doesn't update cpsr
          if (SOffSet) {
            // Split 'opcode<s><c> Rd, Rn, Rm, shift #imm'

            // Rm shifts #imm
            // The new MI checks CondCode, doesn't update CPSR.
            MachineInstrBuilder fst =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
            addOperand(fst, Rm);
            fst.addImm(SOffSet);
            fst.addImm(MI.getOperand(idx - 1).getImm());
            addOperand(fst, MI.getOperand(idx));
            fst.addMetadata(N_fst);

            // Build 'newOpc<s><c> Rd, Rn, VReg'
            // The new MI both updates CPSR and checks CondCode.
            MachineInstrBuilder sec =
                BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
            addOperand(sec, Rd, true);
            for (unsigned n = 1; n < (i - 1); n++) {
              addOperand(sec, MI.getOperand(n));
            }
            sec.addReg(VReg);
            sec.addImm(MI.getOperand(idx - 1).getImm());
            addOperand(sec, MI.getOperand(idx));
            addOperand(sec, MI.getOperand(idx + 1));
            sec.addMetadata(N_sec);
          } /* TODP: Paul
          else {
            if (ShiftOpc == AArch64::RRX) {
              // Split opcode<s><c> Rd, Rn, Rm, RRX
              MachineInstrBuilder fst =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
              addOperand(fst, Rm);
              fst.addMetadata(N_fst);
              // RRX implicit CPSR, how to add cpsr?

              MachineInstrBuilder sec =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
              addOperand(sec, Rd, true);

              for (unsigned n = 1; n < (i - 1); n++) {
                addOperand(sec, MI.getOperand(n));
              }
              sec.addReg(VReg);
              sec.addImm(MI.getOperand(idx - 1).getImm());
              addOperand(sec, MI.getOperand(idx));
              addOperand(sec, MI.getOperand(idx + 1));
              sec.addMetadata(N_sec);
            } else {
              // Split opcode<s><c> Rd, Rn, Rm, shift Rs

              // Rm shift #imm.
              // The new MI checks CondCode, doesn't update CPSR.
              MachineInstrBuilder fst =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(ShiftOpc), VReg);
              addOperand(fst, Rn);
              addOperand(fst, Rm);
              fst.addImm(MI.getOperand(idx - 1).getImm());
              addOperand(fst, MI.getOperand(idx));
              fst.addMetadata(N_fst);

              // Build 'newOpc<s><c> Rd, Rn, VReg'
              // The new MI both updates CPSR and checks CondCode.
              MachineInstrBuilder sec =
                  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(newOpc));
              addOperand(sec, Rd, true);

              for (unsigned n = 1; n < (i - 2); n++) {
                addOperand(sec, MI.getOperand(n));
              }
              sec.addReg(VReg);
              sec.addImm(MI.getOperand(idx - 1).getImm());
              addOperand(sec, MI.getOperand(idx));
              addOperand(sec, MI.getOperand(idx + 1));
              sec.addMetadata(N_sec);
            }
          } */
        }
        mi = &MI;
        break;
      }
    }
  }

  return mi;
}

bool AArch64InstructionSplitting::split() {
  if (PrintPass)
    dbgs() << "AArch64InstructionSplitting start.\n";

  std::vector<MachineInstr *> removelist;
  for (MachineBasicBlock &MBB : *MF) {
    for (MachineBasicBlock::iterator I = MBB.begin(), E = MBB.end(); I != E;
         ++I) {
      MachineInstr &MI = *I;
      MachineInstr *removeMI = nullptr;

      unsigned Opcode, newOpc;
      Opcode = MI.getOpcode();
      newOpc = checkisShifter(Opcode);

      // Need to split
      if (getLoadStoreOpcode(Opcode)) {
        // Split the MI about Load and Store.

        // TODO: LDRSH/LDRSB/LDRH/LDRD split.
        if (isLDRSTRPre(Opcode)) {
          if (MI.getOperand(3).isReg())
            removeMI = splitLDRSTRPre(MBB, MI);
          else if (MI.getOperand(3).isImm() && MI.getOperand(3).getImm() != 0)
            removeMI = splitLDRSTRPreImm(MBB, MI);
          if (removeMI)
            removelist.push_back(removeMI);
        } else if (MI.getOperand(1).isReg() &&
                   MI.getOperand(1).getReg() != AArch64::SP // &&
                   /*MI.getOperand(1).getReg() != AArch64::PC*/) {
          if (MI.getOperand(2).isReg())
            removeMI = splitLDRSTR(MBB, MI);
          else if (MI.getOperand(2).isImm() && MI.getOperand(2).getImm() != 0)
            removeMI = splitLDRSTRImm(MBB, MI);
          if (removeMI)
            removelist.push_back(removeMI);
        }
      } else if (newOpc) {
        // Split the MI except Load and Store.

        bool UpdateCPSR = false;
        bool CondCode = false;
        int idx = MI.findRegisterUseOperandIdx(AArch64::NZCV);

        // Check if MI contains CPSR
        if (idx != -1) {
          if (MI.getOperand(idx + 1).isReg() &&
              MI.getOperand(idx + 1).getReg() == AArch64::NZCV) {
            UpdateCPSR = true;
            CondCode = true;
          } else if (MI.getOperand(idx - 1).isImm() &&
                     MI.getOperand(idx - 1).getImm() != AArch64CC::AL) {
            CondCode = true;
          } else
            UpdateCPSR = true;
        }

        if (!UpdateCPSR && !CondCode)
          // Split the MI has no cpsr.
          removeMI = splitCommon(MBB, MI, newOpc);
        else if (UpdateCPSR && !CondCode)
          // Split the MI updates cpsr.
          removeMI = splitS(MBB, MI, newOpc, idx);
        else if (!UpdateCPSR && CondCode)
          // Split the MI checks CondCode.
          removeMI = splitC(MBB, MI, newOpc, idx);
        else
          // Split the MI both updates cpsr and check CondCode
          removeMI = splitCS(MBB, MI, newOpc, idx);

        if (removeMI)
          removelist.push_back(removeMI);
      }
    }
  }

  // Remove old MI.
  for (MachineInstr *mi : removelist)
    mi->removeFromParent();

  // For debugging.
  if (PrintPass) {
    LLVM_DEBUG(MF->dump());
    LLVM_DEBUG(getCRF()->dump());
    dbgs() << "AArch64InstructionSplitting end.\n";
  }

  return true;
}

bool AArch64InstructionSplitting::runOnMachineFunction(MachineFunction &mf) {
  bool rtn = false;
  init();
  rtn = split();
  return rtn;
}

#ifdef __cplusplus
extern "C" {
#endif

FunctionPass *InitializeAArch64InstructionSplitting(AArch64ModuleRaiser &mr) {
  return new AArch64InstructionSplitting(mr);
}

#ifdef __cplusplus
}
#endif
