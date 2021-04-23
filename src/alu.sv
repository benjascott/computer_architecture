// This timescale statement indicates that each time tick of the simulator
// is 1 nanosecond and the simulator has a precision of 1 picosecond. This 
// is used for simulation and all of your SystemVerilog files should have 
// this statement at the top. 
`timescale 1 ns / 1 ps 
//include constants for slecting alu operations
`include "riscv_alu_constants.sv"

/***************************************************************************
* 
* Module: alu.sv
*
* Author: Ben Scott
* Class: ECEN 323, Winter Semester 2021
* Date: 1/20/2021
*
* Description:
*    This module implements the ALU for the micro processor we are building 
*    in EcEn 323 this semester 
*
****************************************************************************/

module alu(op1, op2, alu_op, result);
    input [31:0] op1, op2;
    input [3:0] alu_op;
    output [31:0] result;
    
    //internal variable to store operation values, then put into result
    logic [31:0] math;
    
    //comment for always comb block alu operations
    always_comb begin
        math = op1 + op2;
        case (alu_op)
            ALUOP_AND: 
                math = op1 & op2;
            ALUOP_OR: 
                math = op1 | op2;
            ALUOP_SUB: 
                math = op1 - op2;
            ALUOP_LT: 
                math = $signed(op1) < $signed(op2);
            ALUOP_XOR: 
                math = op1 ^ op2;
            default: 
                math = op1 + op2;
        endcase
    end
    
    assign result = math;
endmodule