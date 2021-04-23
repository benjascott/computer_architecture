`ifndef RISCV_ALU_CONSTANTS
`define RISCV_ALU_CONSTANTS

localparam[3:0] ALUOP_AND = 4'b0000;
localparam[3:0] ALUOP_OR = 4'b0001;
localparam[3:0] ALUOP_ADD = 4'b0010;
localparam[3:0] ALUOP_SUB = 4'b0011;
localparam[3:0] ALUOP_LT = 4'b0111;
localparam[3:0] ALUOP_XOR = 4'b1101;

`endif // RISCV_ALU_CONSTANTS