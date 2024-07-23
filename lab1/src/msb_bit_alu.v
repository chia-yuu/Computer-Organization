`timescale 1ns / 1ps
// 111550108

/* checkout FIGURE C.5.10 (Bottom) */
/* [Prerequisite] complete bit_alu.v */
module msb_bit_alu (
    input        a,          // 1 bit, a
    input        b,          // 1 bit, b
    input        less,       // 1 bit, Less
    input        a_invert,   // 1 bit, Ainvert
    input        b_invert,   // 1 bit, Binvert
    input        carry_in,   // 1 bit, CarryIn
    input  [1:0] operation,  // 2 bit, Operation
    output reg   result,     // 1 bit, Result (Must it be a reg?)
    output       set,        // 1 bit, Set
    output       overflow    // 1 bit, Overflow
);

    /* Try to implement the most significant bit ALU by yourself! */
    // invert input
    wire ai, bi;
    assign ai = (a_invert==0)? a : !a;
    assign bi = (b_invert==0)? b : !b;

    // full adder
    wire sum;
    assign carry_out = (ai & bi) | (bi & carry_in) | (ai & carry_in);
    assign sum       = ai ^ bi ^ carry_in;

    // mux to get reslut
    always @(*) begin  // `*` auto captures sensitivity ports, now it's combinational logic
        case (operation)  // `case` is similar to `switch` in C
            2'b00:   result <= ai&bi;  // AND
            2'b01:   result <= ai|bi;  // OR
            2'b10:   result <= sum;  // ADD
            2'b11:   result <= less; // SLT
            default: result <= 0;  // should not happened
        endcase
    end

    // set and overflow
    // assign set = sum;
    // assign set = (ai & ~bi)? 1 : (~ai & bi)? 0 : sum;
    assign set = sum ^ overflow;
    // assign overflow = carry_out;
    // assign overflow = (!ai & !bi & result) | (ai & bi & !result);
    assign overflow = carry_in ^ carry_out;

endmodule
