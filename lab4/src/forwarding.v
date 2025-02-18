`timescale 1ns / 1ps
// 111550108, finish in lab 4

/** [Reading] 4.7 p.363-371
 * Understand when and how to forward
 */

/* checkout FIGURE 4.55 for definition of mux control signals */
/* checkout FIGURE 4.56/60 for how this unit should be connected */
module forwarding (
    input            ID_EX_mem_write,
    input      [4:0] ID_EX_rs,          // inputs are pipeline registers relate to forwarding
    input      [4:0] ID_EX_rt,
    input            EX_MEM_reg_write,
    input      [4:0] EX_MEM_rd,
    input            MEM_WB_reg_write,
    input      [4:0] MEM_WB_rd,
    output reg [1:0] forward_A,         // ALU operand is from: 00:ID/EX, 10: EX/MEM, 01:MEM/WB
    output reg [1:0] forward_B
);
    /** [step 1] Forwarding
     * 1. EX hazard (p.366)(PPT p.82)
     * 2. MEM hazard (p.369)(PPt p.84)
     * 3. Solve potential data hazards between:
          the result of the instruction in the WB stage,
          the result of the instruction in the MEM stage,
          and the source operand of the instruction in the ALU stage.
          Hint: Be careful that the textbook is wrong here!
          Hint: Which of EX & MEM hazard has higher priority? ans: EX
     */
    always @(*) begin
        // forward_A
        // EX hazard, from last inst
        if (EX_MEM_reg_write && EX_MEM_rd != 5'b0 && EX_MEM_rd == ID_EX_rs) begin
            forward_A <= 2'b10;
        end
        // MEM hazard, from last two inst
        else if (MEM_WB_reg_write && MEM_WB_rd != 5'b0 &&
            !(EX_MEM_reg_write && EX_MEM_rd != 5'b0 && EX_MEM_rd == ID_EX_rs) &&
            MEM_WB_rd == ID_EX_rs) begin
            forward_A <= 2'b01;
        end
        // no hazard, from reg file
        else begin
            forward_A <= 2'b00;
        end

        // forward_B
        // EX hazard, from last inst
        if (!ID_EX_mem_write && EX_MEM_reg_write && EX_MEM_rd != 5'b0 && EX_MEM_rd == ID_EX_rt) begin
            forward_B <= 2'b10;
        end
        // MEM hazard, from last two inst
        else if (MEM_WB_reg_write && MEM_WB_rd != 5'b0 &&
            !(!ID_EX_mem_write && EX_MEM_reg_write && EX_MEM_rd != 5'b0 && EX_MEM_rd == ID_EX_rt) &&
            MEM_WB_rd == ID_EX_rt) begin
            forward_B <= 2'b01;
        end
        // no hazard, from reg file
        else begin
            forward_B <= 2'b00;
        end
    end

endmodule
