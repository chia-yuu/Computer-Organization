`timescale 1ns / 1ps
// 111550108, copy from lab 3

/* Copy your ALU Control (if you have one) from Lab 2 */
module alu_control (
    input  [1:0] alu_op,    // ALUOp
    input  [5:0] funct,     // Funct field
    output [3:0] operation  // Operation
);

    assign operation[3] = 1'b0;
    assign operation[2] = (funct[1] & alu_op[1]) | alu_op[0];       // (f1 * op1) + op0
    assign operation[1] = ~funct[2] | ~alu_op[1];               // !f2 + !op1
    assign operation[0] = (funct[3] | funct[0]) & alu_op[1];      // (f3+f0) * op1
    
endmodule
