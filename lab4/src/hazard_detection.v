`timescale 1ns / 1ps
// 111550108

/** [Reading] 4.7 p.372-375
 * Understand when and how to detect stalling caused by data hazards.
 * When read a reg right after it was load from memory,
 * it is impossible to solve the hazard just by forwarding.
 */

/* checkout FIGURE 4.59 to understand why a stall is needed */
/* checkout FIGURE 4.60 for how this unit should be connected */
module hazard_detection (
    input        control_branch,       // add by myself
    input        control_mem_write, // add by myself
    input        EX_MEM_mem_read,   // add by myself
    input        MEM_WB_mem_to_reg, // add by myself
    input        EX_MEM_reg_write,  // add by myself
    input        ID_EX_mem_read,
    input  [4:0] ID_EX_rt,
    input  [4:0] IF_ID_rs,
    input  [4:0] IF_ID_rt,
    input  [4:0] EX_MEM_rd,       // add by myself
    input  [4:0] MEM_WB_rd,       // add by meself
    input  [4:0] ID_EX_rd,
    output       pc_write,        // only update PC when this is set
    output       IF_ID_write,     // only update IF/ID stage registers when this is set
    output       stall            // insert a stall (bubble) in ID/EX when this is set
);

    /** [step 3] Stalling
     * 1. calculate stall by equation from textbook.
     * 2. Should pc be written when stall?
     * 3. Should IF/ID stage registers be updated when stall?
     */
    // wrong because only consider lw hazard
    // always @(*) begin
    //     // stall
    //     if(ID_EX_mem_read && (ID_EX_rt==IF_ID_rs || ID_EX_rt==IF_ID_rt))begin
    //         pc_write    <= 1'b0;
    //         IF_ID_write <= 1'b0;
    //         stall       <= 1'b1;
    //     end
    //     // no stall
    //     else begin
    //         pc_write    <= 1'b1;
    //         IF_ID_write <= 1'b1;
    //         stall       <= 1'b0;
    //     end
    // end

    wire lw_hazard = ID_EX_mem_read && ~control_mem_write && (ID_EX_rt==IF_ID_rs || ID_EX_rt==IF_ID_rt);

    wire branch_hazard1 = control_branch && (EX_MEM_reg_write && (EX_MEM_rd == IF_ID_rs || EX_MEM_rd == IF_ID_rt) ||
                                             (MEM_WB_mem_to_reg && (MEM_WB_rd == IF_ID_rs || MEM_WB_rd == IF_ID_rt)));
    // wire branch_hazard2 = control_branch && (ID_EX_mem_read || EX_MEM_mem_read);
    wire branch_hazard2 = control_branch && ((EX_MEM_mem_read && (EX_MEM_rd == IF_ID_rs || EX_MEM_rd == IF_ID_rt)) ||
                                             (ID_EX_mem_read && (ID_EX_rd == IF_ID_rs || ID_EX_rd == IF_ID_rt)));

    wire hazard = lw_hazard || branch_hazard1 || branch_hazard2;
    assign stall = hazard;
    assign pc_write = ~hazard;
    assign IF_ID_write = ~hazard;


endmodule