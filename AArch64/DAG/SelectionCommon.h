//===- SelectionCommon.h ----------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains some declarations, and defines some structures, which are
// use to DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_LLVM_MCTOLL_AARCH64_DAG_SELECTIONCOMMON_H
#define LLVM_TOOLS_LLVM_MCTOLL_AARCH64_DAG_SELECTIONCOMMON_H

#include "AArch64ISelLowering.h"
#include "llvm/CodeGen/ISDOpcodes.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"

/// This is the start index of EXT_AArch64ISD. Because node types which start
/// from AArch64ISD::LD2post (Next to AArch64ISD::UUNPKLO) are identified as
/// TARGET_MEMORY_OPCODE, we set EXTAArch64ISD_OP_BEGIN index after AArch64ISD::UUNPKLO,
/// plugs 40 to keep long time with no confliction.
#define EXTAArch64ISD_OP_BEGIN (AArch64ISD::UUNPKLO + 40)

namespace llvm {
namespace EXT_AArch64ISD {

enum NodeType {
  BX_RET = EXTAArch64ISD_OP_BEGIN,
  BRD, // Direct branch
  LOAD,
  STORE,
  MSR,
  MRS,
  RSB,
  RSC,
  SBC,
  TEQ,
  TST,
  BIC,
  MLA,
  UXTB,

  EXT_AArch64ISD_OP_END
};

} // namespace EXT_AArch64ISD
} // namespace llvm

using namespace llvm;

/// This structure is to extend SDNode properties, some additional SDNode
/// properties which are used by llvm-mctoll will be kept at here.
typedef struct NodeProperty {
  bool HasCPSR;
  bool Special;
  bool UpdateCPSR;
  unsigned Cond;
  bool IsTwoAddress;
  Value *Val;
  const MachineInstr *MI;
} NodePropertyInfo;

#endif // LLVM_TOOLS_LLVM_MCTOLL_AARCH64_DAG_SELECTIONCOMMON_H
