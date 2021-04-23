// This timescale statement indicates that each time tick of the simulator
// is 1 nanosecond and the simulator has a precision of 1 picosecond. This 
// is used for simulation and all of your SystemVerilog files should have 
// this statement at the top. 
`timescale 1 ns / 1 ps 

/***************************************************************************
* 
* Module: regfile.sv
*
* Author: Ben Scott
* Class: ECEN 323, Winter Semester 2021
* Date: 1/25/2021
*
* Description:
*   creates a module to read anfd write from register memory
*   
****************************************************************************/

module regfile(clk, regAddrA, regAddrB, regAddrWrite, regWriteData, regWrite, regReadDataA, regReadDataB);

    input clk, regWrite;
    input [4:0] regAddrA, regAddrB, regAddrWrite;
    input [31:0] regWriteData;
    
    output logic [31:0] regReadDataA, regReadDataB;
    
    localparam regZero = 5'b00000;
    
    logic [31:0] register[31:0];
    
    integer i;
    initial
        for (i=0;i<32;i=i+1)
            register[i] = 0;   
            
    always_ff@(posedge clk) begin
        regReadDataA <= register[regAddrA];
        regReadDataB <= register[regAddrB];
        if(regWrite && !(regAddrWrite == regZero)) begin
            register[regAddrWrite] <= regWriteData;
            if (regAddrA == regAddrWrite)
                regReadDataA <= regWriteData;
            if (regAddrB == regAddrWrite)
                regReadDataB <= regWriteData;
        end
    end
    
endmodule