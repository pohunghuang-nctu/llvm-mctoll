# Added by Paul Huang 20200317
include_directories(
  ${LLVM_MAIN_SRC_DIR}/lib/Target/AArch64
  ${LLVM_BINARY_DIR}/lib/Target/AArch64
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_SOURCE_DIR}/.
  ${CMAKE_CURRENT_SOURCE_DIR}/DAG
)

if(NOT LLVM_MCTOLL_BUILT_STANDALONE)
  set(LLVM_MCTOLL_DEPS intrinsics_gen AArch64CommonTableGen)
endif()

add_llvm_library(AArch64Raiser
  AArch64ModuleRaiser.cpp
  AArch64FunctionPrototype.cpp
  AArch64MIRevising.cpp
  AArch64EliminatePrologEpilog.cpp
  AArch64CreateJumpTable.cpp
  AArch64ArgumentRaiser.cpp
  AArch64FrameBuilder.cpp
  AArch64InstructionSplitting.cpp
  DAG/FunctionRaisingInfo.cpp
  DAG/DAGRaisingInfo.cpp
  DAG/DAGBuilder.cpp
  DAG/InstSelector.cpp
  DAG/IREmitter.cpp
  AArch64SelectionDAGISel.cpp
  AArch64MachineInstructionRaiser.cpp

  DEPENDS
    ${LLVM_MCTOLL_DEPS}
  )
