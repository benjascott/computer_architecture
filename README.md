# computer_architecture

This folder contains code that simulates a RISCV processor. If downloaded, it can be run as a project in VIVADO.
The simulated processor is structured as a 5 stage pipeline (Instruction Fetch, Instruction Decode, Execute, Memory, and Write Back stages).
The project also contains code that simulates an ALU as well as a register file containing 32 registers that hold 32 bit words. 
The program contains all the logic to implement the datapath flow as well as total control flow.
The simulated processor also implements forwarding techniques to keep from stalls in the pipeline due to data hazards.
It also implements hazard detection techniques to keep from exceptions caused by control hazards by detecting branch hazards as well as load-use hazards. Implements necessary stalls and nops to ensure the pipeline implements only the correct instructions. 

List of RISCV instructions that the simulated processor handles:

BEQ

ADD/ADDI

SUB/SUBI

AND/ANDI

OR/ORI

XOR/XORI

LT

STORE

LOAD
