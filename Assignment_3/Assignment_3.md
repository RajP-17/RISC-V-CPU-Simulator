
# Computer Architecture (ECGR 4181/5181)

## Assignment 3: RISC-V Decoder Design [50 points]

Consider the Decoder of a processor using the von Neumann architecture (discussed in class). The processor uses a subset of the RISC-V ISA (RV32I, RV32F and RV32M) from the "p15 RISCV Table" in Lectures. The double words, ecall, ebreak, fence, and "cs" ops are to be ignored. We are considering UNPIPELINED design for this assignment.

### Tasks

1. **Draw the schematic of the architecture** [30 points]
   
   Draw the schematic of the architecture where the different components (in the fetch, decode, execute, writeback stages) are shown as black boxes, their ports are clearly defined, and connections between these ports detailed. Explain the purpose of these ports and connections.

2. **Write the Decoder code** [20 points]
   
   Write the Decoder code (in C++) and comment in the code to highlight the executions for every instruction.

### Notes

- Processor: von Neumann architecture
- ISA Subset: RV32I, RV32F, RV32M
- Reference: "p15 RISCV Table" from Lectures
- Ignore: double words, ecall, ebreak, fence, "cs" ops
- Design Type: UNPIPELINED