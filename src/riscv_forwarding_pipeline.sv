// This timescale statement indicates that each time tick of the simulator
// is 1 nanosecond and the simulator has a precision of 1 picosecond. This 
// is used for simulation and all of your SystemVerilog files should have 
// this statement at the top. 
`timescale 1 ns / 1 ps 
`include "riscv_alu_constants.sv"

/***************************************************************************
* 
* Module: riscv_forwarding_pipeline.sv
*
* Author: Ben Scott
* Class: ECEN 323, Winter Semester 2021
* Date: 3/8/2021
*
* Description:
*    This module implements a pipeline for executing riscv instructions using
*     forwarding and stalling techniques, also implements hazard detection
*     
*
****************************************************************************/
//`default_nettype none

 ///////////////// Datapath Constants ///////////////////////
    
    localparam [31:0] PC_INIT = 32'h00400000;
    localparam [19:0] IMM_ONES = 20'hfffff;
    localparam [19:0] IMM_ZEROS = 20'h00000;
    localparam ZERO_HI = 1'b1;
    localparam ZERO_LOW = 1'b0;
    localparam LOAD_PC_HIGH = 1'b1;
    
    //opCode constants
    localparam [6:0] BRANCH_OPCODE = 7'b1100011;
    localparam [6:0] R_TYPE_OPCODE = 7'b0110011;
    localparam [6:0] LOAD_OPCODE = 7'b0000011;
    localparam [6:0] IMM_OPCODE = 7'b0010011;
    localparam [6:0] STORE_OPCODE = 7'b0100011;
    
    //ALUOp constants
    localparam[3:0] FUNC3_AND = 3'b111;
    localparam[3:0] FUNC3_OR = 3'b110;
    localparam[3:0] FUNC3_ADD = 3'b000;
    localparam[3:0] FUNC3_LT = 3'b010;
    localparam[3:0] FUNC3_XOR = 3'b100;
    localparam[6:0] FUNC7_SUB = 7'b0100000;
    localparam[6:0] FUNC7_ADD = 7'b0000000;
    
    localparam[31:0] NOP_INSTRUCTION = 32'h00000013;
    
    localparam [4:0] REG_ZERO = 5'b00000;

module riscv_forwarding_pipeline #(parameter INITIAL_PC = PC_INIT)
(clk, rst, instruction, dReadData, PC, iMemRead, ALUResult, dAddress, dWriteData, MemRead, MemWrite, WriteBackData);

    input wire logic clk, rst;
    input wire logic [31:0] instruction, dReadData;
    
    output logic iMemRead, MemRead, MemWrite;
    output logic [31:0] PC, ALUResult, dAddress, dWriteData, WriteBackData;
    
    ///////////// IF stage vars ///////////////////////////////////////
    logic [31:0] if_PC;
    
    ///////////// ID stage vars ///////////////////////////////////////
    logic [31:0] id_PC;
    logic [6:0] id_opCode, id_func7;
    logic [2:0] id_func3;
    logic [31:0] id_s_imm_data, id_i_imm_data, id_imm_data, id_branch_offset;
    logic [3:0] id_ALUCtrl;
    logic id_s_type, id_branch, id_ALUSrc, id_MemWrite, id_MemRead, id_RegWrite, id_MemtoReg;
    logic [4:0] id_regWriteAddr, id_rs1, id_rs2;
    
    
    ///////////// EX stage vars ///////////////////////////////////////
    logic [31:0] ex_PC, ex_imm_data, ex_op2, ex_op1, ex_op2_forwarding, ex_result;
    logic [3:0] ex_ALUCtrl;
    logic [31:0] ex_regReadDataA, ex_regReadDataB, ex_branch_target, ex_branch_offset;
    logic ex_Zero, ex_branch, ex_ALUSrc, ex_MemWrite, ex_MemRead, ex_RegWrite, ex_MemtoReg;
    logic [4:0] ex_regWriteAddr, ex_rs1, ex_rs2;
    
    
    ///////////// MEM stage vars //////////////////////////////////////
    logic [31:0] mem_PC, mem_result;
    logic mem_PCSrc, mem_MemWrite, mem_MemRead;
    logic [31:0] mem_branch_target, mem_regReadDataB, mem_dWriteData, mem_dAddress, mem_op2_forwarding;
    logic mem_Zero, mem_branch, mem_RegWrite, mem_MemtoReg;    
    logic [4:0] mem_regWriteAddr;
    
    
    ///////////// WB stage vars ///////////////////////////////////////
    logic [31:0] wb_PC;
    logic wb_MemtoReg, wb_RegWrite, wb_Zero, wb_branch;
    logic [31:0] wb_regWriteData, wb_result;
    logic [4:0] wb_regWriteAddr;
    
    ///////////Forwarding vars ////////////////////////////////////////
    logic load_use_hazard, branch_mem_taken, branch_wb_taken;
    
    
    
    ///////////////////////////////////////////////////////////////////////////////
    // IF: Instruction Fetch
    ///////////////////////////////////////////////////////////////////////////////
    
    //program counter
    //register to pipeline signals from if stage to id stage
    always_ff@(posedge clk) begin
        if (rst)
            if_PC <= INITIAL_PC;
        else if (load_use_hazard)
            if_PC <= if_PC;
        else if (mem_PCSrc)
            if_PC <= mem_branch_target;//advance PC to branch destination
        else
            if_PC <= if_PC + 4;  
    end
    
    //pipeline register from IF stage to ID stage
    always_ff@(posedge clk) begin
        if(rst)
            id_PC <= PC_INIT;
        else if (load_use_hazard)
            id_PC <= id_PC;
        else
            id_PC <= if_PC;
    end 
    
    ///////////////////////////////////////////////////////////////////////////////
    // ID: Instruction Decode
    ///////////////////////////////////////////////////////////////////////////////
    
    //load-use stalling logic
    assign iMemRead = !load_use_hazard;
    
    //instruction decode
    assign id_opCode = instruction[6:0];
    assign id_func3 = instruction[14:12];
    assign id_func7 = instruction[31:25];    
    
    assign id_rs1 = instruction[19:15];
    assign id_rs2 = instruction[24:20];
    
    //MemRead and MemWrite logic
    assign id_MemRead = (id_opCode == LOAD_OPCODE);
    assign id_MemWrite = (id_opCode == STORE_OPCODE);
    
    //branch option
    assign id_branch = (id_opCode == BRANCH_OPCODE);
    
    //MemtoReg
    assign id_MemtoReg = (id_opCode == LOAD_OPCODE);
    
    //RegWrite
    assign id_RegWrite = ((id_opCode == R_TYPE_OPCODE) || (id_opCode == LOAD_OPCODE) || (id_opCode == IMM_OPCODE));
        
    //ALUSrc control logic
    assign id_ALUSrc = ((id_opCode == R_TYPE_OPCODE) || (id_opCode == BRANCH_OPCODE))?1'b0:1'b1;
    
    // 1) decode instruction
    always_comb begin
                    
        //combinational logic for ALUCtrl
        case (id_func3)
                FUNC3_AND: begin
                    id_ALUCtrl = ALUOP_AND;
            end FUNC3_OR: begin
                    id_ALUCtrl = ALUOP_OR;
            end FUNC3_ADD: begin 
                    //if subract (func7), ALUCtrl = Subtract, else, Add
                    if ((id_func7 == FUNC7_SUB) && (id_opCode == R_TYPE_OPCODE))
                        id_ALUCtrl = ALUOP_SUB;
                    else 
                        id_ALUCtrl = ALUOP_ADD;
            end FUNC3_LT: begin
                    id_ALUCtrl = ALUOP_LT;
            end FUNC3_XOR: begin
                    id_ALUCtrl = ALUOP_XOR;
            end default: begin
                    //default ALUCtrl is ADD
                    id_ALUCtrl = ALUOP_ADD;
            end
        endcase
        
        //if it is a branch instruction do subtract
        if (id_opCode == BRANCH_OPCODE)
          id_ALUCtrl = ALUOP_SUB;
        
        //ALUCtrl for opcode
        if ((id_opCode == LOAD_OPCODE) || (id_opCode == STORE_OPCODE))
          id_ALUCtrl = ALUOP_ADD;    
          
        //regWrite Addr
        id_regWriteAddr = instruction[11:7];   
    end
    
    // 2) immediate generation
    //construction of I type instruction immediate value
    assign id_i_imm_data[11:0] = instruction[31:20];
    assign id_i_imm_data[31:12] = id_i_imm_data[11]?IMM_ONES:IMM_ZEROS;
    
    //construction of S type instruction immediate value
    assign id_s_imm_data[4:0] = instruction[11:7];
    assign id_s_imm_data[11:5] = instruction[31:25];
    assign id_s_imm_data[31:12] = id_s_imm_data[11]?IMM_ONES:IMM_ZEROS;
    assign id_s_type = (instruction[7:0] == STORE_OPCODE)?1'b1:1'b0;
    
    //assign actual immediate data
    assign id_imm_data = id_s_type?id_s_imm_data:id_i_imm_data;
    
    //create branch offset value
    assign id_branch_offset[0] = ZERO_LOW;
    assign id_branch_offset[4:1] = instruction[11:8];
    assign id_branch_offset[10:5] = instruction[30:25];
    assign id_branch_offset[11] = instruction[7]; 
    assign id_branch_offset[12] = instruction[31];
    assign id_branch_offset[31:13] = id_branch_offset[12]?19'b1111111111111111111:19'b0000000000000000000;
    
    // 3) register file
    //call regfile module
    regfile my_regfile(.clk(clk), .regAddrA(instruction[19:15]), .regAddrB(instruction[24:20]), .regAddrWrite(wb_regWriteAddr),
                        .regWriteData(wb_regWriteData), .regWrite(wb_RegWrite), .regReadDataA(ex_regReadDataA), .regReadDataB(ex_regReadDataB));
    
    //register to pipeline signals from id stage to ex stage
    always_ff@(posedge clk) begin
        if (rst || load_use_hazard || branch_mem_taken || branch_wb_taken) begin 
            //reset control signals
            ex_PC <= PC_INIT; 
            ex_ALUSrc <= 1'b0;
            ex_RegWrite <= 0;
            ex_MemtoReg <= 0;
            ex_ALUCtrl <= ALUOP_ADD;
            ex_MemRead <= 0;
            ex_MemWrite <= 0;
            ex_branch <= 0;
            ex_branch_offset <= 0;
            ex_regWriteAddr <= 0;
            ex_imm_data <= 0;
            ex_rs1 <= 0;
            ex_rs2 <= 0;
        end
        else begin
            ex_PC <= id_PC;
            //ex_instruction <= instruction;
            ex_ALUCtrl <= id_ALUCtrl;
            ex_imm_data <= id_imm_data;
            ex_ALUSrc <= id_ALUSrc;
            ex_MemWrite <= id_MemWrite;
            ex_MemRead <= id_MemRead;
            ex_RegWrite <= id_RegWrite;
            ex_MemtoReg <= id_MemtoReg;
            ex_regWriteAddr <= id_regWriteAddr;
            ex_branch_offset <= id_branch_offset;
            ex_branch <= id_branch;
            ex_rs1 <= id_rs1;
            ex_rs2 <= id_rs2;
        end
    end
        
        
    ///////////////////////////////////////////////////////////////////////////////
    // EX: Execution
    ///////////////////////////////////////////////////////////////////////////////
    //multiplexer for second operand in alu mod
    
    //forwarding logic
    always_comb begin
        //defaults
        ex_op1 = ex_regReadDataA;
        ex_op2_forwarding = ex_regReadDataB;
        ex_op2 = ex_op2_forwarding;
        
        //forwarding for op1 (rs1)
        if(!(ex_rs1 == REG_ZERO)) begin
            if ((mem_regWriteAddr == ex_rs1) && mem_RegWrite)
                ex_op1 = mem_result;
            else if ((wb_regWriteAddr == ex_rs1) && wb_RegWrite)
                ex_op1 = wb_regWriteData;   
        end
        
        //first mux for rs2 (forwarding)
        if(!(ex_rs2 == REG_ZERO)) begin
            if ((mem_regWriteAddr == ex_rs2) && mem_RegWrite)
                ex_op2_forwarding = mem_result;
            else if ((wb_regWriteAddr == ex_rs2) && wb_RegWrite)
                ex_op2_forwarding = wb_regWriteData;  
        end
        //second mux for rs2 (immediate data or nah)
        if(ex_ALUSrc)
            ex_op2 = ex_imm_data;
        else 
            ex_op2 = ex_op2_forwarding;
    end
    
    //instance alu module
    alu my_alu(.op1(ex_op1), .op2(ex_op2), .alu_op(ex_ALUCtrl), .result(ex_result));
    
    //assign Zero signal
    assign ex_Zero = ex_result?1'b0:1'b1;
    
    //assign branch target
    assign ex_branch_target = ex_PC + $signed(ex_branch_offset);
    
    //register to pipeline signals from ex stage to mem stage
    always_ff@(posedge clk) begin
        if (rst || branch_mem_taken || branch_wb_taken) begin
            mem_PC <= PC_INIT;
            mem_result <= 0;
            mem_Zero <= 0;
            mem_branch <= 0;
            mem_branch_target <= 0;
            mem_MemWrite <= 0;
            mem_MemRead <= 0;
            mem_MemtoReg <= 0;
            mem_RegWrite <= 0;
            mem_regWriteAddr <= 0;
            mem_regReadDataB <= 0;
        end
        else begin 
            mem_PC <= ex_PC;
            mem_result <= ex_result;
            mem_Zero <= ex_Zero;
            mem_branch <= ex_branch;
            mem_branch_target <= ex_branch_target;
            mem_MemWrite <= ex_MemWrite;
            mem_MemRead <= ex_MemRead;
            mem_MemtoReg <= ex_MemtoReg;
            mem_RegWrite <= ex_RegWrite;
            mem_regWriteAddr <= ex_regWriteAddr;
            mem_regReadDataB <= ex_regReadDataB;
            mem_op2_forwarding <= ex_op2_forwarding;
        end
    end
    
    
    ///////////////////////////////////////////////////////////////////////////////
    // MEM: Memory Stage
    ///////////////////////////////////////////////////////////////////////////////
    //assign PC source signal
    assign mem_PCSrc = (mem_Zero && mem_branch);
    assign branch_mem_taken = (mem_Zero && mem_branch);
    assign mem_dWriteData = mem_op2_forwarding;
    assign mem_dAddress = mem_result;
    
    assign MemRead = mem_MemRead;
    assign MemWrite = mem_MemWrite;
    
    //register to pipeline values from mem stage to wb stage
    always_ff@(posedge clk) begin
        if(rst || branch_wb_taken) begin
            wb_PC <= PC_INIT;
            wb_result <= 0;
            wb_MemtoReg <= 0;
            wb_RegWrite <= 0;
            wb_regWriteAddr <= 0;
            wb_Zero <= 0;
            wb_branch <= 0;
            
        end
        else begin
            wb_PC <= mem_PC;
            wb_result <= mem_result;
            wb_MemtoReg <= mem_MemtoReg;
            wb_RegWrite <= mem_RegWrite;
            wb_regWriteAddr <= mem_regWriteAddr;
            wb_Zero <= mem_Zero;
            wb_branch <= mem_branch;
        end
    end
    
    
    ///////////////////////////////////////////////////////////////////////////////
    // WB: Write Back
    ///////////////////////////////////////////////////////////////////////////////
    //assign regWriteData signal multiplexer
    assign wb_regWriteData = wb_MemtoReg?dReadData:wb_result;
    assign branch_wb_taken = (wb_Zero && wb_branch);
    
    
    ///////////////////////////////////////////////////
    // Drive top level ports
    ///////////////////////////////////////////////////
    
    assign PC = if_PC; //if stage check
    assign ALUResult = ex_result; //ex stage check
    assign dWriteData = mem_dWriteData; //mem stage check
    assign dAddress = mem_dAddress;
    assign WriteBackData = wb_regWriteData; //wb stage check

    
    //////////////////////////////////////////////////////
    // load-use detection logic
    //////////////////////////////////////////////////////
    assign load_use_hazard = (ex_MemRead && ((ex_regWriteAddr == id_rs1) || (ex_regWriteAddr == id_rs2)) && !(mem_Zero && mem_branch));
    
    
    
endmodule