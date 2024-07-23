`timescale 1ns / 1ps
// 111550108, copy from lab 3, add parameter lui, ori, jump

/* Copy your Control (and its components) from Lab 2 */
module control (
    input  [5:0] opcode,      // the opcode field of a instruction is [?:?]
    output       reg_dst,     // select register destination: rt(0), rd(1)
    output       alu_src,     // select 2nd operand of ALU: rt(0), sign-extended(1)
    output       mem_to_reg,  // select data write to register: ALU(0), memory(1)
    output       reg_write,   // enable write to register file
    output       mem_read,    // enable read form data memory
    output       mem_write,   // enable write to data memory
    output       branch,      // this is a branch instruction or not (work with alu.zero)
    output [1:0] alu_op,      // ALUOp passed to ALU Control unit
    output       lui,         // doesn't used in lab3
    output       ori,         // doesn't used in lab3
    output       jump,        // not sure
    output       addi         // used in lab3
);

    assign lui = opcode[0] & opcode[1] & opcode[2] & opcode[3] & ~opcode[4] & ~opcode[5];
    assign ori = opcode[0] & ~opcode[1] & opcode[2] & opcode[3] & ~opcode[4] & ~opcode[5];
    assign jump = ~opcode[0] & opcode[1] & ~opcode[2] & ~opcode[3] & ~opcode[4] & ~opcode[5];
    assign addi = ~opcode[0] & ~opcode[1] & ~opcode[2] & opcode[3] & ~opcode[4] & ~opcode[5];   // 001000

    wire r, lw, sw, beq;
    assign r = ~opcode[0] & ~opcode[1] & ~opcode[2] & ~opcode[3] & ~opcode[4] & ~opcode[5];
    assign lw = (opcode[0] & opcode[1] & ~opcode[2] & ~opcode[3] & ~opcode[4] & opcode[5]) | lui | ori | addi;
    assign sw = opcode[0] & opcode[1] & ~opcode[2] & opcode[3] & ~opcode[4] & opcode[5];
    assign beq = ~opcode[0] & ~opcode[1] & opcode[2] & ~opcode[3] & ~opcode[4] & ~opcode[5];

    assign reg_dst = r;
    assign alu_src = lw || sw || addi;
    assign mem_to_reg = lw;
    assign reg_write = r || lw;
    assign mem_read = lw;
    assign mem_write = sw;
    assign branch = beq;
    assign alu_op[1] = r;
    assign alu_op[0] = beq;

endmodule
