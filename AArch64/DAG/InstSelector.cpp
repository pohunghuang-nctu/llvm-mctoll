//===- InstSelector.cpp - Binary raiser utility llvm-mctoll ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementaion of InstSelector class for use
// by llvm-mctoll.
//
//===----------------------------------------------------------------------===//

#include "InstSelector.h"
#include "AArch64.h"
#include "AArch64Subtarget.h"
#include "SelectionCommon.h"

using namespace llvm;

/// Replace all uses of F with T, then remove F from the DAG.
void AArch64InstSelector::replaceNode(SDNode *F, SDNode *T) {
  if (MachineSDNode::classof(F)) {
    NodePropertyInfo *np = DAGInfo->NPMap[F];
    DAGInfo->NPMap.erase(F);
    DAGInfo->NPMap[T] = np;
  }

  CurDAG->ReplaceAllUsesWith(F, T);
  CurDAG->RemoveDeadNode(F);
}

/// Checks the SDNode is a function argument or not.
bool AArch64InstSelector::isArgumentNode(SDNode *node) {
  if (!FrameIndexSDNode::classof(node))
    return false;

  return FuncInfo->isArgumentIndex(
      dyn_cast<FrameIndexSDNode>(node)->getIndex());
}

/// Checks the SDNode is a function return or not.
bool AArch64InstSelector::isReturnNode(SDNode *node) {
  if (!FrameIndexSDNode::classof(node))
    return false;

  return FuncInfo->isReturnIndex(dyn_cast<FrameIndexSDNode>(node)->getIndex());
}

/// Record the new defined Node, it uses to map the register number to Node.
/// In DAG emitter, emitter get a value of use base on this defined Node.
void AArch64InstSelector::recordDefinition(SDNode *oldNode, SDNode *newNode) {
  assert(newNode != nullptr &&
         "The new SDNode ptr is null when record define!");

  if (oldNode == nullptr) {
    outs() << "Warning: RecordDefine error, the SDNode ptr is null!\n";
    return;
  }

  if (RegisterSDNode::classof(oldNode)) {
    unsigned opReg = static_cast<RegisterSDNode *>(oldNode)->getReg();
    FuncInfo->setValueByRegister(opReg, SDValue(newNode, 0));
    FuncInfo->NodeRegMap[newNode] = opReg;
  }

  if (isReturnNode(oldNode)) {
    FuncInfo->setRetValue(SDValue(newNode, 0));
    FuncInfo->setValueByRegister(AArch64::X0, SDValue(newNode, 0));
    FuncInfo->NodeRegMap[newNode] = AArch64::X0;
  }
}

/// Gets the Metadata of given SDNode.
SDValue AArch64InstSelector::getMDOperand(SDNode *N) {
  for (auto &sdv : N->ops()) {
    if (MDNodeSDNode::classof(sdv.getNode())) {
      return sdv.get();
    }
  }
  assert(false && "Should not run at here!");
  return SDValue();
}

/// Instruction opcode selection.
void AArch64InstSelector::selectCode(SDNode *N) {
  SDLoc dl(N);

  switch (N->getMachineOpcode()) {
  default:
    break;
  /* ADC */
  // TODO:Paul
  /*
  case AArch64::ADCrr:
  case AArch64::ADCri:
  case AArch64::ADCrsr:
  case AArch64::ADCrsi:
  case AArch64::tADC:
  case AArch64::t2ADCrr:
  case AArch64::t2ADCri:
  case AArch64::t2ADCrs: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      // ADCS <Rdn>,<Rm>
      // ADC<c> <Rdn>,<Rm>
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node =
          CurDAG
              ->getNode(ISD::ADDC, dl, getDefaultEVT(), Rd, Rn, getMDOperand(N))
              .getNode();
    } else {
      // ADC{S}<c> <Rd>,<Rn>,#<const>
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);
      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node = CurDAG
                 ->getNode(ISD::ADDC, dl, getDefaultEVT(), Rn, op2,
                           getMDOperand(N))
                 .getNode();
    }

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* ADD */
  // TODO:Paul
  /*
  case AArch64::ADDri:
  case AArch64::ADDrr:
  case AArch64::ADDrsi:
  case AArch64::ADDrsr:
  case AArch64::tADDspi:
  case AArch64::tADDrSP:
  case AArch64::tADDi3:
  case AArch64::tADDrSPi:
  case AArch64::tADDi8:
  case AArch64::tADDhirr:
  case AArch64::tADDrr:
  case AArch64::tADDspr:
  case AArch64::t2ADDrs:
  case AArch64::t2ADDri:
  case AArch64::t2ADDrr:
  case AArch64::t2ADDri12: {
    // TODO:
    // 1. Check out MI is two-address or three-address
    // 2. Do with the displacement operation.(not yet implement.)
    // Judge the MI address module, then check out whether has the imm.
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    // <opcode>   {<cond>}{s}<Rd>，<Rn>{，<OP2>}
    SDNode *Node = nullptr;
    if (FrameIndexSDNode::classof(N->getOperand(1).getNode())) {
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::LOAD, dl, getDefaultEVT(), Rn,
                           getMDOperand(N))
                 .getNode();
    } else {
      if (isTwoAddressMode(Rd.getNode())) {
        if (RegisterSDNode::classof(N->getOperand(1).getNode()))
          Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

        SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
        Node = CurDAG
                   ->getNode(ISD::ADD, dl, getDefaultEVT(), Rd, Rn,
                             getMDOperand(N))
                   .getNode();
      } else {
        SDValue op2 = N->getOperand(2);
        if (RegisterSDNode::classof(op2.getNode()))
          op2 = FuncInfo->getValFromRegMap(op2);

        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
        Node = CurDAG
                   ->getNode(ISD::ADD, dl, getDefaultEVT(), Rn, op2,
                             getMDOperand(N))
                   .getNode();
      }
    }

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* SUB */
  // TODO:Paul
  /*
  case AArch64::SUBri:
  case AArch64::SUBrr:
  case AArch64::SUBrsi:
  case AArch64::SUBrsr:
  case AArch64::tSUBi3:
  case AArch64::tSUBi8:
  case AArch64::tSUBrr:
  case AArch64::tSUBspi:
  case AArch64::t2SUBri:
  case AArch64::t2SUBri12:
  case AArch64::t2SUBrr:
  case AArch64::t2SUBrs:
  case AArch64::t2SUBS_PC_LR: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node =
          CurDAG
              ->getNode(ISD::SUB, dl, getDefaultEVT(), Rd, Rn, getMDOperand(N))
              .getNode();
    } else {
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);

      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node =
          CurDAG
              ->getNode(ISD::SUB, dl, getDefaultEVT(), Rn, op2, getMDOperand(N))
              .getNode();
    }
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* MOV */
  // TODO:Paul
  /*  
  case AArch64::MOVi16:
  case AArch64::t2MOVi16:
  case AArch64::MOVi32imm:
  case AArch64::tMOVr:
  case AArch64::MOVr:
  case AArch64::t2MOVi:
  case AArch64::t2MOVr:
  case AArch64::MOVCCr:
  case AArch64::t2MOVCCr:
  case AArch64::t2MOVi32imm:
  case AArch64::MOVTi16:
  case AArch64::MOVi: {
    // Dispalcement operation need do.
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    if (RegisterSDNode::classof(Rn.getNode()))
      Rn = FuncInfo->getValFromRegMap(Rn);

    SDNode *Node = CurDAG
                       ->getNode(AArch64ISD::CMOV, dl, getDefaultEVT(), Rn,
                                 CurDAG->getConstant(0, dl, getDefaultEVT()))
                       .getNode();

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* STR */
// TODO:Paul
    /*
  case AArch64::STRi12:
  case AArch64::STRrs:
  case AArch64::STRD:
  case AArch64::STRD_POST:
  case AArch64::STRD_PRE:
  case AArch64::t2STREXD:
  case AArch64::STREXB:
  case AArch64::STREXD:
  case AArch64::STREXH:
  case AArch64::STREX:
  case AArch64::STR_PRE_IMM:
  case AArch64::STR_PRE_REG:
  case AArch64::STR_POST_IMM:
  case AArch64::STR_POST_REG: {
    SDValue Val = N->getOperand(0);
    SDValue Ptr = N->getOperand(1); // This is a pointer.

    if (RegisterSDNode::classof(Val.getNode()))
      Val = FuncInfo->getValFromRegMap(Val);

    if (RegisterSDNode::classof(Ptr.getNode()))
      Ptr = FuncInfo->getValFromRegMap(Ptr);

    SDNode *Node = CurDAG
                       ->getNode(EXT_AArch64ISD::STORE, dl, getDefaultEVT(), Val,
                                 Ptr, getMDOperand(N))
                       .getNode();
    replaceNode(N, Node);
  } break;
  case AArch64::STRH:
  case AArch64::STRH_PRE:
  case AArch64::STRH_POST: {
    EVT InstTy = EVT::getEVT(Type::getInt16Ty(*CurDAG->getContext()));
    SDValue Val = N->getOperand(0);
    SDValue Op1 = N->getOperand(1);
    SDNode *Node = nullptr;

    if (RegisterSDNode::classof(Val.getNode()))
      Val = FuncInfo->getValFromRegMap(Val);

    if (RegisterSDNode::classof(Op1.getNode()))
      Op1 = FuncInfo->getValFromRegMap(Op1);

    if (N->getNumOperands() < 5)
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::STORE, dl, InstTy, Val, Op1,
                           getMDOperand(N))
                 .getNode();
    else {
      SDValue Op2 = N->getOperand(2);
      Op2 = FuncInfo->getValFromRegMap(Op2);
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::STORE, dl, InstTy, Val, Op1, Op2,
                           getMDOperand(N))
                 .getNode();
    }

    replaceNode(N, Node);
  } break;
  case AArch64::STRBi12:
  case AArch64::STRBrs:
  case AArch64::STRB_PRE_IMM:
  case AArch64::STRB_PRE_REG:
  case AArch64::STRB_POST_IMM:
  case AArch64::STRB_POST_REG: {
    EVT InstTy = EVT::getEVT(Type::getInt8Ty(*CurDAG->getContext()));
    SDValue Val = N->getOperand(0);
    SDValue Op1 = N->getOperand(1);
    SDNode *Node = nullptr;

    if (RegisterSDNode::classof(Val.getNode()))
      Val = FuncInfo->getValFromRegMap(Val);

    if (RegisterSDNode::classof(Op1.getNode()))
      Op1 = FuncInfo->getValFromRegMap(Op1);

    if (N->getNumOperands() < 5)
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::STORE, dl, InstTy, Val, Op1,
                           getMDOperand(N))
                 .getNode();
    else {
      SDValue Op2 = N->getOperand(2);
      Op2 = FuncInfo->getValFromRegMap(Op2);
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::STORE, dl, InstTy, Val, Op1, Op2,
                           getMDOperand(N))
                 .getNode();
    }

    replaceNode(N, Node);
  } break;
  */
  /* LDR */
  // TODO:Paul
    /*
  case AArch64::LDRi12:
  case AArch64::LDRrs:
  case AArch64::t2LDR_PRE:
  case AArch64::t2LDR_POST:
  case AArch64::tLDR_postidx:
  case AArch64::LDR_PRE_IMM:
  case AArch64::LDR_PRE_REG:
  case AArch64::LDR_POST_IMM:
  case AArch64::LDR_POST_REG: {
    EVT InstTy = EVT::getEVT(Type::getInt32Ty(*CurDAG->getContext()));
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (RegisterSDNode::classof(Rn.getNode()))
      Rn = FuncInfo->getValFromRegMap(Rn);

    Node = CurDAG->getNode(EXT_AArch64ISD::LOAD, dl, InstTy, Rn, getMDOperand(N))
               .getNode();

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  case AArch64::LDRH:
  case AArch64::LDRSH:
  case AArch64::t2LDRSH_PRE:
  case AArch64::t2LDRSH_POST:
  case AArch64::t2LDRH_PRE:
  case AArch64::t2LDRH_POST:
  case AArch64::LDRSH_PRE:
  case AArch64::LDRSH_POST: {
    EVT InstTy = EVT::getEVT(Type::getInt16Ty(*CurDAG->getContext()));
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;

    if (RegisterSDNode::classof(Rn.getNode()))
      Rn = FuncInfo->getValFromRegMap(Rn);
    Node = CurDAG->getNode(EXT_AArch64ISD::LOAD, dl, InstTy, Rn, getMDOperand(N))
               .getNode();

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  case AArch64::LDRBi12:
  case AArch64::LDRBrs:
  case AArch64::t2LDRSB_PRE:
  case AArch64::t2LDRSB_POST:
  case AArch64::t2LDRB_PRE:
  case AArch64::t2LDRB_POST:
  case AArch64::LDRSB_PRE:
  case AArch64::LDRSB_POST:
  case AArch64::LDRB_PRE_IMM:
  case AArch64::LDRB_POST_IMM:
  case AArch64::LDRB_PRE_REG:
  case AArch64::LDRB_POST_REG: {
    EVT InstTy = EVT::getEVT(Type::getInt8Ty(*CurDAG->getContext()));
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;

    if (RegisterSDNode::classof(Rn.getNode()))
      Rn = FuncInfo->getValFromRegMap(Rn);
    Node = CurDAG->getNode(EXT_AArch64ISD::LOAD, dl, InstTy, Rn, getMDOperand(N))
               .getNode();

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* Branch */
    // TODO:Paul
    /*
  case AArch64::Bcc:
  case AArch64::tBcc:
  case AArch64::t2Bcc: {
    SDValue Iftrue = N->getOperand(0);
    SDValue Cond = N->getOperand(1);
    SDNode *Node = nullptr;

    if (DAGInfo->NPMap[N]->HasCPSR)
      Node = CurDAG
                 ->getNode(ISD::BRCOND, dl, getDefaultEVT(), Iftrue, Cond,
                           getMDOperand(N))
                 .getNode();
    else
      Node =
          CurDAG->getNode(ISD::BR, dl, getDefaultEVT(), Iftrue, getMDOperand(N))
              .getNode();

    const MachineBasicBlock *LMBB = DAGInfo->NPMap[N]->MI->getParent();
    if (LMBB->succ_size() == 0) {
      FuncInfo->setValueByRegister(AArch64::R0, SDValue(Node, 0));
      FuncInfo->NodeRegMap[Node] = AArch64::R0;
    }
    replaceNode(N, Node);
  } break;
  case AArch64::B:
  case AArch64::tB:
  case AArch64::t2B: {
    SDValue BrBlock = N->getOperand(0);
    SDNode *Node =
        CurDAG->getNode(ISD::BR, dl, getDefaultEVT(), BrBlock, getMDOperand(N))
            .getNode();

    replaceNode(N, Node);
  } break;
  case AArch64::BL:
  case AArch64::BL_pred:
  case AArch64::tBL: {
    SDValue Func = N->getOperand(0);
    SDNode *Node = nullptr;
    if (RegisterSDNode::classof(Func.getNode())) {
      Func = FuncInfo->getValFromRegMap(Func);
      Node =
          CurDAG
              ->getNode(ISD::BRIND, dl, getDefaultEVT(), Func, getMDOperand(N))
              .getNode();
    } else {
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::BRD, dl, getDefaultEVT(), Func,
                           getMDOperand(N))
                 .getNode();
    }

    FuncInfo->setValueByRegister(AArch64::R0, SDValue(Node, 0));
    FuncInfo->NodeRegMap[Node] = AArch64::R0;
    replaceNode(N, Node);
  } break;
  case AArch64::BLX:
  case AArch64::BLXi:
  case AArch64::BLX_pred:
  case AArch64::tBLXi:
  case AArch64::tBLXr: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::BR_JTr: {
    SDNode *Node = nullptr;
    SDValue Rd = N->getOperand(0);
    Node = CurDAG->getNode(ISD::BR_JT, dl, getDefaultEVT(), Rd, getMDOperand(N))
               .getNode();
    replaceNode(N, Node);
  } break;
  case AArch64::BX:
  case AArch64::BX_CALL:
  case AArch64::BX_pred:
  case AArch64::tBX:
  case AArch64::tBX_CALL: {
    SDValue CallReg = N->getOperand(0);
    if (RegisterSDNode::classof(CallReg.getNode()))
      CallReg = FuncInfo->getValFromRegMap(CallReg);

    SDNode *Node =
        CurDAG
            ->getNode(ISD::BRIND, dl, getDefaultEVT(), CallReg, getMDOperand(N))
            .getNode();
    replaceNode(N, Node);
  } break;
  case AArch64::BX_RET:
  case AArch64::tBX_RET:
    // assert(0 && "Branch instructions are removed in previous stage. should
    // not get here!");
    break;
  case AArch64::tCMPhir:
  case AArch64::CMPrr:
  case AArch64::t2CMPri:
  case AArch64::CMPri:
  case AArch64::tCMPi8:
  case AArch64::t2CMPrr:
  case AArch64::tCMPr: {
    SDValue cmpl = N->getOperand(0);
    SDValue cmph = N->getOperand(1);
    if (RegisterSDNode::classof(cmph.getNode()))
      cmph = FuncInfo->getValFromRegMap(N->getOperand(1));
    cmpl = FuncInfo->getValFromRegMap(cmpl);

    // Create condition SDValuleR
    // TODO: It should be verified why this type node can not be added Metadata
    // Operand.
    SDNode *Node = CurDAG
                       ->getNode(ISD::SETCC, dl, getDefaultEVT(), cmpl, cmph
                                 )  // , getMDOperand(N) 
                       .getNode();

    replaceNode(N, Node);
  } break;
  */
  /* AND */
  // TODO:Paul 
   /*
  case AArch64::ANDri:
  case AArch64::ANDrr:
  case AArch64::ANDrsi:
  case AArch64::ANDrsr:
  case AArch64::tAND:
  case AArch64::t2ANDri:
  case AArch64::t2ANDrr:
  case AArch64::t2ANDrs: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;

    if (isTwoAddressMode(Rd.getNode())) {
      // AND<c> <Rdn>,<Rm>
      // ANDS <Rdn>,<Rm>
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node =
          CurDAG
              ->getNode(ISD::AND, dl, getDefaultEVT(), Rd, Rn, getMDOperand(N))
              .getNode();
    } else {
      // AND{S}<c> <Rd>,<Rn>,#<const>
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);

      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node =
          CurDAG
              ->getNode(ISD::AND, dl, getDefaultEVT(), Rn, op2, getMDOperand(N))
              .getNode();
    }

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
    // TODO:
    // AND{S}<c>.W <Rd>,<Rn>,<Rm>{,<shift>}
    // AND{S}<c> <Rd>,<Rn>,<Rm>{,<shift>}
    // AND{S}<c> <Rd>,<Rn>,<Rm>,<type> <Rs>
  } break;
  */
  /* ASR */
    //TODO:Paul
  /*
  case AArch64::ASRr:
  case AArch64::ASRi:
  case AArch64::tASRrr:
  case AArch64::tASRri:
  case AArch64::t2ASRrr:
  case AArch64::t2ASRri: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node =
          CurDAG
              ->getNode(ISD::SRA, dl, getDefaultEVT(), Rd, Rn, getMDOperand(N))
              .getNode();
    } else {
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);

      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node =
          CurDAG
              ->getNode(ISD::SRA, dl, getDefaultEVT(), Rn, op2, getMDOperand(N))
              .getNode();
    }

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* CMN */
  //TODO:Paul
    /*
  case AArch64::CMNri:
  case AArch64::CMNzrr:
  case AArch64::tCMNz:
  case AArch64::t2CMNri:
  case AArch64::t2CMNzrr:
  case AArch64::t2CMNzrs: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;

    if (RegisterSDNode::classof(N->getOperand(1).getNode()))
      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
    Rd = FuncInfo->getValFromRegMap(Rd);
    Node =
        CurDAG
            ->getNode(AArch64ISD::CMN, dl, getDefaultEVT(), Rd, Rn, getMDOperand(N))
            .getNode();

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* EOR */
    //TODO:Paul
    /*
  case AArch64::EORri:
  case AArch64::EORrr:
  case AArch64::EORrsi:
  case AArch64::EORrsr:
  case AArch64::tEOR:
  case AArch64::t2EORrr:
  case AArch64::t2EORrs:
  case AArch64::t2EORri: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      // EORS <Rdn>,<Rm>
      // EOR<c> <Rdn>,<Rm>
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node =
          CurDAG
              ->getNode(ISD::XOR, dl, getDefaultEVT(), Rd, Rn, getMDOperand(N))
              .getNode();
    } else {
      // EOR{S}<c> <Rd>,<Rn>,#<const>
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);
      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node =
          CurDAG
              ->getNode(ISD::XOR, dl, getDefaultEVT(), Rn, op2, getMDOperand(N))
              .getNode();
    }
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
    // TODO:
    // EOR{S}<c>.W <Rd>,<Rn>,<Rm>{,<shift>}
    // EOR{S}<c> <Rd>,<Rn>,<Rm>{,<shift>}
    // EOR{S}<c> <Rd>,<Rn>,<Rm>,<type> <Rs>
  } break;
  */
  /* MSR */
    //TODO:Paul
    /*
  case AArch64::MSR:
  case AArch64::MSRi:
  case AArch64::MSRbanked:
  case AArch64::t2MSR_M:
  case AArch64::t2MSR_AR:
  case AArch64::t2MSRbanked: {
    // Update the CPSR.
    SDValue Cond = N->getOperand(1);
    SDNode *Node = nullptr;
    if (RegisterSDNode::classof(N->getOperand(1).getNode()))
      Cond = FuncInfo->getValFromRegMap(N->getOperand(1));

    Node = CurDAG
               ->getNode(EXT_AArch64ISD::MSR, dl, getDefaultEVT(), Cond,
                         getMDOperand(N))
               .getNode();

    replaceNode(N, Node);
  } break;
  */
  /* MUL */
    //TODO:Paul
    /*
  case AArch64::MUL:
  case AArch64::tMUL:
  case AArch64::t2MUL: {

    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    SDValue op2 = N->getOperand(2);
    op2 = FuncInfo->getValFromRegMap(op2);
    Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
    Node =
        CurDAG->getNode(ISD::MUL, dl, getDefaultEVT(), Rn, op2, getMDOperand(N))
            .getNode();

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* MVN */
    //TODO:Paul
    /*
  case AArch64::MVNi:
  case AArch64::MVNr:
  case AArch64::MVNsi:
  case AArch64::MVNsr:
  case AArch64::tMVN:
  case AArch64::t2MVNi:
  case AArch64::t2MVNr:
  case AArch64::t2MVNs: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (RegisterSDNode::classof(N->getOperand(1).getNode()))
      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

    Node = CurDAG
               ->getNode(ISD::XOR, dl, getDefaultEVT(), Rn,
                         CurDAG->getConstant(-1, dl, getDefaultEVT()))
               .getNode();

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* LSL */
    //TODO:Paul
    /*
  case AArch64::LSLi:
  case AArch64::LSLr:
  case AArch64::tLSLri:
  case AArch64::tLSLrr:
  case AArch64::t2LSLri:
  case AArch64::t2LSLrr: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

      Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node =
          CurDAG
              ->getNode(ISD::SHL, dl, getDefaultEVT(), Rd, Rn, getMDOperand(N))
              .getNode();
    } else {
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);

      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node =
          CurDAG
              ->getNode(ISD::SHL, dl, getDefaultEVT(), Rn, op2, getMDOperand(N))
              .getNode();
    }
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* LSR */
    //TODO:Paul
    /*
  case AArch64::LSRi:
  case AArch64::LSRr:
  case AArch64::tLSRri:
  case AArch64::tLSRrr:
  case AArch64::t2LSRri:
  case AArch64::t2LSRrr: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node =
          CurDAG
              ->getNode(ISD::SRL, dl, getDefaultEVT(), Rd, Rn, getMDOperand(N))
              .getNode();
    } else {
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);
      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node =
          CurDAG
              ->getNode(ISD::SRL, dl, getDefaultEVT(), Rn, op2, getMDOperand(N))
              .getNode();
    }
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* ORR */
    //TODO:Paul
    /*
  case AArch64::ORRri:
  case AArch64::ORRrr:
  case AArch64::ORRrsi:
  case AArch64::ORRrsr:
  case AArch64::tORR:
  case AArch64::t2ORRri:
  case AArch64::t2ORRrr:
  case AArch64::t2ORRrs: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    // <opcode>   {<cond>}{s}<Rd>，<Rn>{，<OP2>}
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node =
          CurDAG->getNode(ISD::OR, dl, getDefaultEVT(), Rd, Rn, getMDOperand(N))
              .getNode();
    } else {
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);

      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node =
          CurDAG
              ->getNode(ISD::OR, dl, getDefaultEVT(), Rn, op2, getMDOperand(N))
              .getNode();
    }
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* ROR */
    //TODO:Paul
    /*
  case AArch64::RORi:
  case AArch64::RORr:
  case AArch64::tROR:
  case AArch64::t2RORri:
  case AArch64::t2RORrr: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node =
          CurDAG
              ->getNode(ISD::ROTR, dl, getDefaultEVT(), Rd, Rn, getMDOperand(N))
              .getNode();
    } else {
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);
      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node = CurDAG
                 ->getNode(ISD::ROTR, dl, getDefaultEVT(), Rn, op2,
                           getMDOperand(N))
                 .getNode();
    }
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* RRX */
    //TODO:Paul
    /*
  case AArch64::RRX: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
    SDNode *Node = nullptr;
    Node =
        CurDAG->getNode(AArch64ISD::RRX, dl, getDefaultEVT(), Rn, getMDOperand(N))
            .getNode();

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* RSB */
    //TODO:Paul
    /*
  case AArch64::RSBri:
  case AArch64::RSBrr:
  case AArch64::RSBrsi:
  case AArch64::RSBrsr:
  case AArch64::tRSB:
  case AArch64::t2RSBri:
  case AArch64::t2RSBrr:
  case AArch64::t2RSBrs: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::RSB, dl, getDefaultEVT(), Rd, Rn,
                           getMDOperand(N))
                 .getNode();
    } else {
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);
      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::RSB, dl, getDefaultEVT(), op2, Rn,
                           getMDOperand(N))
                 .getNode();
    }
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* RSC */
    //TODO:Paul
    /*
  case AArch64::RSCri:
  case AArch64::RSCrr:
  case AArch64::RSCrsi:
  case AArch64::RSCrsr: {
    // RSC{S}<c> <Rd>,<Rn>, #0
    // RSC{S}<c>.W <Rd>,<Rn>,#<const>
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::RSC, dl, getDefaultEVT(), Rd, Rn,
                           getMDOperand(N))
                 .getNode();
    } else {
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);
      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::RSC, dl, getDefaultEVT(), Rn, op2,
                           getMDOperand(N))
                 .getNode();
    }
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* CLZ */
    //TODO:Paul
    /*
  case AArch64::CLZ:
  case AArch64::t2CLZ: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
    SDNode *Node = nullptr;
    Node = CurDAG->getNode(ISD::CTLZ, dl, getDefaultEVT(), Rn, getMDOperand(N))
               .getNode();
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* SBC */
    //TODO:Paul
    /*
  case AArch64::SBCrr:
  case AArch64::SBCri:
  case AArch64::tSBC: {
    SDValue Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
    SDValue Operand2 = FuncInfo->getValFromRegMap(N->getOperand(2));
    SDNode *Node = CurDAG
                       ->getNode(EXT_AArch64ISD::SBC, dl, getDefaultEVT(), Rn,
                                 Operand2, getMDOperand(N))
                       .getNode();

    recordDefinition(Rn.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* TEQ */
    //TODO:Paul
    /*
  case AArch64::TEQri:
  case AArch64::TEQrr:
  case AArch64::TEQrsi:
  case AArch64::TEQrsr:
  case AArch64::t2TEQri:
  case AArch64::t2TEQrr:
  case AArch64::t2TEQrs: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;

    if (RegisterSDNode::classof(N->getOperand(1).getNode()))
      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

    Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
    Node = CurDAG
               ->getNode(EXT_AArch64ISD::TEQ, dl, getDefaultEVT(), Rd, Rn,
                         getMDOperand(N))
               .getNode();

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* TST */
    //TODO:Paul
    /*
  case AArch64::TSTrsi:
  case AArch64::TSTrr:
  case AArch64::TSTri:
  case AArch64::TSTrsr:
  case AArch64::tTST:
  case AArch64::t2TSTri:
  case AArch64::t2TSTrr:
  case AArch64::t2TSTrs: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;

    if (RegisterSDNode::classof(N->getOperand(1).getNode()))
      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

    Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
    Node = CurDAG
               ->getNode(EXT_AArch64ISD::TST, dl, getDefaultEVT(), Rd, Rn,
                         getMDOperand(N))
               .getNode();

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break; */
  /* BIC */
    /*
  case AArch64::BICri:
  case AArch64::BICrr:
  case AArch64::BICrsi:
  case AArch64::BICrsr:
  case AArch64::tBIC:
  case AArch64::t2BICri:
  case AArch64::t2BICrr:
  case AArch64::t2BICrs: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = N->getOperand(1);
    SDNode *Node = nullptr;
    if (isTwoAddressMode(Rd.getNode())) {
      if (RegisterSDNode::classof(N->getOperand(1).getNode()))
        Rn = FuncInfo->getValFromRegMap(N->getOperand(1));

      SDValue Rd = FuncInfo->getValFromRegMap(N->getOperand(0));
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::BIC, dl, getDefaultEVT(), Rd, Rn,
                           getMDOperand(N))
                 .getNode();
    } else {
      SDValue op2 = N->getOperand(2);
      if (RegisterSDNode::classof(op2.getNode()))
        op2 = FuncInfo->getValFromRegMap(op2);

      Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
      Node = CurDAG
                 ->getNode(EXT_AArch64ISD::BIC, dl, getDefaultEVT(), Rn, op2,
                           getMDOperand(N))
                 .getNode();
    }

    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* MLA */
    //TODO:Paul
    /*
  case AArch64::MLA:
  case AArch64::t2MLA: {
    SDValue Rd = N->getOperand(0);
    SDValue Rn = FuncInfo->getValFromRegMap(N->getOperand(1));
    SDValue Rm = FuncInfo->getValFromRegMap(N->getOperand(2));
    SDValue Ra = FuncInfo->getValFromRegMap(N->getOperand(3));
    SDNode *Node = nullptr;
    Node = CurDAG
               ->getNode(EXT_AArch64ISD::MLA, dl, getDefaultEVT(), Rn, Rm, Ra,
                         getMDOperand(N))
               .getNode();
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  */
  /* UXTB */
    //TODO:Paul
    /*
  case AArch64::UXTB: {
    SDValue Rd = N->getOperand(0);
    SDValue Rm = N->getOperand(1);
    SDValue Rotation = N->getOperand(2);
    SDNode *Node = nullptr;

    if (RegisterSDNode::classof(N->getOperand(1).getNode()))
      Rm = FuncInfo->getValFromRegMap(N->getOperand(1));
    Node = CurDAG
               ->getNode(EXT_AArch64ISD::UXTB, dl, getDefaultEVT(), Rd, Rm,
                         Rotation, getMDOperand(N))
               .getNode();
    recordDefinition(Rd.getNode(), Node);
    replaceNode(N, Node);
  } break;
  case AArch64::MCR:
  case AArch64::MCRR:
  case AArch64::t2MCR:
  case AArch64::t2MCRR:
  case AArch64::VMSR:
  case AArch64::VMSR_FPEXC:
  case AArch64::VMSR_FPSID:
  case AArch64::VMSR_FPINST:
  case AArch64::VMSR_FPINST2: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::MRS:
  case AArch64::MRSsys:
  case AArch64::t2MRS_AR:
  case AArch64::t2MRSsys_AR: {
    SDValue Rn = N->getOperand(0);
    if (RegisterSDNode::classof(Rn.getNode()))
      Rn = FuncInfo->getValFromRegMap(Rn);

    SDNode *Node =
        CurDAG
            ->getNode(EXT_AArch64ISD::MRS, dl, getDefaultEVT(), Rn, getMDOperand(N))
            .getNode();
    replaceNode(N, Node);
  } break;
  */
  /* ABS */
    //TODO:Paul
    /*
  case AArch64::ABS:
  case AArch64::t2ABS: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::tLDRpci:
  case AArch64::LDRcp: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::t2SBFX:
  case AArch64::SBFX:
  case AArch64::t2UBFX:
  case AArch64::UBFX: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::t2UMAAL:
  case AArch64::UMAAL: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::t2UMLAL:
  case AArch64::UMLAL:
  case AArch64::UMLALv5: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::t2SMLAL:
  case AArch64::SMLAL:
  case AArch64::SMLALv5: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::t2SMMLS:
  case AArch64::SMMLS: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::VZIPd8:
  case AArch64::VZIPd16:
  case AArch64::VZIPq8:
  case AArch64::VZIPq16:
  case AArch64::VZIPq32: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::VUZPd8:
  case AArch64::VUZPd16:
  case AArch64::VUZPq8:
  case AArch64::VUZPq16:
  case AArch64::VUZPq32: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
  case AArch64::VTRNd8:
  case AArch64::VTRNd16:
  case AArch64::VTRNd32:
  case AArch64::VTRNq8:
  case AArch64::VTRNq16:
  case AArch64::VTRNq32: {
    outs() << "WARNING: Not yet implemented!\n";
  } break;
    // TODO: Need to add other pattern matching here.
    */
  }
   
}

void AArch64InstSelector::select(SDNode *N) {
  if (!N->isMachineOpcode()) {
    N->setNodeId(-1);
    return; // Already selected.
  }

  selectCode(N);
}
