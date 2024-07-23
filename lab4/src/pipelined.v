`timescale 1ns / 1ps
// 111550108

/** [Prerequisite] pipelined (Lab 3), forwarding, hazard_detection
 * This module is the pipelined MIPS processor "similar to" FIGURE 4.60 (control hazard is not solved).
 * You can implement it by any style you want, as long as it passes testbench.
 */

/* checkout FIGURE 4.51 */
module pipelined #(
    parameter integer TEXT_BYTES = 1024,        // size in bytes of instruction memory
    parameter integer TEXT_START = 'h00400000,  // start address of instruction memory
    parameter integer DATA_BYTES = 1024,        // size in bytes of data memory
    parameter integer DATA_START = 'h10008000   // start address of data memory
) (
    input clk,  // clock
    input rstn  // negative reset
);

    /** [step 0] Copy from Lab 3
     * You should modify your pipelined processor from Lab 3, so copy to here first.
     */

    /* Instruction Memory */
    wire [31:0] instr_mem_address, instr_mem_instr;
    instr_mem #(
        .BYTES(TEXT_BYTES),
        .START(TEXT_START)
    ) instr_mem (
        .address(instr_mem_address),
        .instr  (instr_mem_instr)
    );

    /* Register Rile */
    wire [4:0] reg_file_read_reg_1, reg_file_read_reg_2, reg_file_write_reg;
    wire reg_file_reg_write;
    wire [31:0] reg_file_write_data, reg_file_read_data_1, reg_file_read_data_2;
    reg_file reg_file (
        .clk        (~clk),                  // only write when negative edge
        .rstn       (rstn),
        .read_reg_1 (reg_file_read_reg_1),
        .read_reg_2 (reg_file_read_reg_2),
        .reg_write  (reg_file_reg_write),
        .write_reg  (reg_file_write_reg),
        .write_data (reg_file_write_data),
        .read_data_1(reg_file_read_data_1),
        .read_data_2(reg_file_read_data_2)
    );

    /* ALU */
    wire [31:0] alu_a, alu_b, alu_result;
    wire [3:0] alu_ALU_ctl;
    wire alu_zero, alu_overflow;
    alu alu (
        .a       (alu_a),
        .b       (alu_b),
        .ALU_ctl (alu_ALU_ctl),
        .result  (alu_result),
        .zero    (alu_zero),
        .overflow(alu_overflow)
    );

    /* Data Memory */
    wire data_mem_mem_read, data_mem_mem_write;
    wire [31:0] data_mem_address, data_mem_write_data, data_mem_read_data;
    data_mem #(
        .BYTES(DATA_BYTES),
        .START(DATA_START)
    ) data_mem (
        .clk       (~clk),                 // only write when negative edge
        .mem_read  (data_mem_mem_read),
        .mem_write (data_mem_mem_write),
        .address   (data_mem_address),
        .write_data(data_mem_write_data),
        .read_data (data_mem_read_data)
    );

    /* ALU Control */
    wire [1:0] alu_control_alu_op;
    wire [5:0] alu_control_funct;
    wire [3:0] alu_control_operation;
    alu_control alu_control (
        .alu_op   (alu_control_alu_op),
        .funct    (alu_control_funct),
        .operation(alu_control_operation)
    );

    /* (Main) Control */
    wire [5:0] control_opcode;
    // Execution/address calculation stage control lines
    wire control_reg_dst, control_alu_src;
    wire [1:0] control_alu_op;
    // Memory access stage control lines
    wire control_branch, control_mem_read, control_mem_write;
    // Wire-back stage control lines
    wire control_reg_write, control_mem_to_reg;
    // Parameter add by myself (lui, ori, jump, addi)
    wire control_lui, control_ori, control_jump, control_addi;
    control control (
        .opcode    (control_opcode),
        .reg_dst   (control_reg_dst),
        .alu_src   (control_alu_src),
        .mem_to_reg(control_mem_to_reg),
        .reg_write (control_reg_write),
        .mem_read  (control_mem_read),
        .mem_write (control_mem_write),
        .branch    (control_branch),
        .alu_op    (control_alu_op),
        .lui       (control_lui),
        .ori       (control_ori),
        .jump      (control_jump),
        .addi      (control_addi)
    );

/***** [step 2] Connect Forwarding unit *****
     * 1. add `ID_EX_rs` into ID/EX stage registers
     * 2. Use a mux to select correct ALU operands according to forward_A/B
     *    Hint don't forget that alu_b might be sign-extended immediate!
     */
    // ID_EX_M = {control_branch, control_mem_read, control_mem_write, control_jump}
    wire [1:0] forward_A, forward_B;
    reg  [4:0] ID_EX_rs, ID_EX_rt, EX_MEM_rd, MEM_WB_rd, ID_EX_rd;
    wire       EX_MEM_reg_write, MEM_WB_reg_write;
    forwarding forwarding (
        .ID_EX_mem_write (ID_EX_M[1]),
        .ID_EX_rs        (ID_EX_rs),
        .ID_EX_rt        (ID_EX_rt),
        .EX_MEM_reg_write(EX_MEM_WB[0]),
        .EX_MEM_rd       (EX_MEM_rd),
        .MEM_WB_reg_write(MEM_WB_WB[0]),
        .MEM_WB_rd       (MEM_WB_rd),
        .forward_A       (forward_A),
        .forward_B       (forward_B)
    );

    // reg cannot assign?
    // assign ID_EX_rs = ID_EX_instr_25[25:21];
    // assign ID_EX_rt = ID_EX_instr_25[20:16];
    // assign EX_MEM_rd = EX_MEM_write_reg;
    // assign MEM_WB_rd = reg_file_write_reg;

    // WB = {mem_to_reg, reg_write}
    reg [1:0]  EX_MEM_WB;   // EX
    reg [1:0]  MEM_WB_WB;   // MEM
    // assign EX_MEM_reg_write = EX_MEM_WB[0];
    // assign MEM_WB_reg_write = MEM_WB_WB[0];
  
    // forward 1st operand
    wire [31:0] alu_a_src, alu_b_src;   // EX
    reg  [31:0] EX_MEM_alu_result;
    assign alu_a = (forward_A==2'b00)? alu_a_src :
                   (forward_A==2'b01)? reg_file_write_data : EX_MEM_alu_result;
    // forward 2nd operand
    assign alu_b = (forward_B==2'b00)? alu_b_src :
                   (forward_B==2'b01)? reg_file_write_data : EX_MEM_alu_result;

    /** [step 4] Connect Hazard Detection unit
     * 1. use `pc_write` when updating PC
     * 2. use `IF_ID_write` when updating IF/ID stage registers
     * 3. use `stall` when updating ID/EX stage registers
     */
    /** WB = {mem_to_reg, reg_write}
     *  M  = {branch, mem_read, mem_write, jump}
     *  EX = {reg_dst, alu_op[1], alu_op[0], alu_src, lui, ori, addi}
    **/
    wire pc_write, IF_ID_write, stall;
    hazard_detection hazard_detection (
        .control_branch(control_branch),
        .control_mem_write(control_mem_write),
        .EX_MEM_mem_read(EX_MEM_M[2]),
        .MEM_WB_mem_to_reg(MEM_WB_WB[1]),
        .EX_MEM_reg_write(EX_MEM_WB[0]),
        .ID_EX_mem_read(ID_EX_M[2]),
        .ID_EX_rt      (ID_EX_rt),
        .IF_ID_rs      (IF_ID_instr[25:21]),
        .IF_ID_rt      (IF_ID_instr[20:16]),
        .EX_MEM_rd     (EX_MEM_write_reg),
        .MEM_WB_rd     (MEM_WB_write_reg),
        .ID_EX_rd      (ID_EX_rd),
        .pc_write      (pc_write),            // implicitly declared
        .IF_ID_write   (IF_ID_write),         // implicitly declared
        .stall         (stall)                // implicitly declared
    );

// Instruction fetch (IF)
    // 1.
    reg [31:0] pc;  // DO NOT change this line
    // 2.
    assign instr_mem_address = pc;
    // 3.
    wire [31:0] pc_4 = pc + 32'd4;
    // 4.
    reg [31:0] IF_ID_instr, IF_ID_pc_4;
    always @(posedge clk)
        if (rstn) begin
            IF_ID_instr <= IF_ID_write? instr_mem_instr : IF_ID_instr;  // a. (me)
            IF_ID_pc_4  <= IF_ID_write? pc_4 : IF_ID_pc_4;  // b. (me)
        end
    always @(negedge rstn) begin
        IF_ID_instr <= 0;  // a. (TA)
        IF_ID_pc_4  <= 0;  // b. (TA)
    end

// Instruction decode and register file read (ID)
    // 1.
    assign control_opcode = IF_ID_instr[31:26];
    // 2.
    assign reg_file_read_reg_1 = IF_ID_instr[25:21];
    assign reg_file_read_reg_2 = IF_ID_instr[20:16];
    // 3.
    wire [31:0] sign_extend;
    assign sign_extend = {{16{IF_ID_instr[15]}}, IF_ID_instr[15:0]};

    // calculate branch target (lab 4 step 5)
    reg [1:0] beq_A, beq_B;
    wire[31:0] branch_target;
    assign branch_target = IF_ID_pc_4 + (sign_extend << 2);
    wire A = (beq_A == 2'b01)? reg_file_write_data :
             (beq_A == 2'b10)? EX_MEM_alu_result : reg_file_read_data_1;
    wire B = (beq_B == 2'b01)? reg_file_write_data :
             (beq_B == 2'b10)? EX_MEM_alu_result : reg_file_read_data_2;
    wire take_branch;
    assign take_branch = control_branch? ((A==B)? 1 : 0) : 0;
    
    // update pc (branch or pc+4)
    reg [31:0] pc_next;
    always @(*) begin
        pc_next <= take_branch? branch_target : pc_4;
    end
    always @(posedge clk) begin
        if(rstn)begin
            if(pc_write) pc <= pc_next;
            // pc <= pc_next;
        end
        else begin
            pc <= 32'h00400000;
        end
    end
    always @(negedge rstn) begin
        pc <= 32'h00400000;
    end

    // 4.
    reg [1:0]  ID_EX_WB;
    reg [3:0]  ID_EX_M;
    reg [6:0]  ID_EX_EX;
    reg [31:0] ID_EX_pc_4, ID_EX_read_data_1, ID_EX_read_data_2, ID_EX_sign_extend;
    reg [5:0]  ID_EX_instr_20, ID_EX_instr_15;
    reg [26:0] ID_EX_instr_25;  // jump address
    /** WB = {mem_to_reg, reg_write}
     *  M  = {branch, mem_read, mem_write, jump}
     *  EX = {reg_dst, alu_op[1], alu_op[0], alu_src, lui, ori, addi}
    **/
    always @(posedge clk)
        if (rstn) begin
            // stall, insert bubble(nop)
            if(stall) begin
                ID_EX_WB <= 0;
                ID_EX_M  <= 0;
                ID_EX_EX <= 0;
            end
            // no stall
            else begin
                ID_EX_WB <= {control_mem_to_reg, control_reg_write};               // a.
                ID_EX_M  <= {control_branch, control_mem_read, control_mem_write, control_jump}; // a.
                ID_EX_EX <= {control_reg_dst, control_alu_op[1], control_alu_op[0],
                             control_alu_src, control_lui, control_ori, control_addi};    // a.
            end
            ID_EX_pc_4        <= IF_ID_pc_4;            // b.
            ID_EX_read_data_1 <= reg_file_read_data_1;  // c.
            ID_EX_read_data_2 <= reg_file_read_data_2;  // c.
            ID_EX_sign_extend <= sign_extend;           // d.
            ID_EX_instr_20    <= IF_ID_instr[20:16];    // e.
            ID_EX_instr_15    <= IF_ID_instr[15:11];    // e.
            ID_EX_instr_25    <= IF_ID_instr[25:0];     // jump address
            ID_EX_rs          <= IF_ID_instr[25:21];    // lab 4
            ID_EX_rt          <= IF_ID_instr[20:16];    // lab 4
            ID_EX_rd          <= IF_ID_instr[15:11];    // lab 4
        end
    always @(negedge rstn) begin
            ID_EX_WB          <= 0;     // a.
            ID_EX_M           <= 0;     // a.
            ID_EX_EX          <= 0;     // a.
            ID_EX_pc_4        <= 0;     // b.
            ID_EX_read_data_1 <= 0;     // c.
            ID_EX_read_data_2 <= 0;     // c.
            ID_EX_sign_extend <= 0;     // d.
            ID_EX_instr_20    <= 0;     // e.
            ID_EX_instr_15    <= 0;     // e.
            ID_EX_instr_25    <= 0;     // jump address
            ID_EX_rs          <= 0;     // lab 4
            ID_EX_rt          <= 0;     // lab 4
            ID_EX_rd          <= 0;     // lab 4
    end

// Execute or address calculation (EX)
    // 0. EX = {reg_dst, alu_op[1], alu_op[0], alu_src, lui, ori, addi}
    // 1.
    wire[31:0] EX_branch_target;
    assign EX_branch_target = ID_EX_pc_4 + (ID_EX_sign_extend << 2);
    // jump
    wire [31:0] EX_jump_target;
    assign EX_jump_target = {ID_EX_pc_4[31:28], ID_EX_instr_25, 2'b00};
    // 2.
    // wire [31:0] alu_a_src, alu_b_src;
    assign alu_a_src = ID_EX_read_data_1;
    assign alu_b_src = ID_EX_EX[3]? ID_EX_sign_extend : ID_EX_read_data_2;  // alu_src = ID_EX_EX[3]
    // 3.
    assign alu_control_alu_op = (ID_EX_EX[2] | ID_EX_EX[1] | ID_EX_EX[0])?  // (lui | ori | addi)?
                                4'b0010 : {ID_EX_EX[5], ID_EX_EX[4]};       // alu_op = ID_EX_EX[5], ID_EX_EX[4]
    assign alu_control_funct  = ID_EX_sign_extend[5:0];     // sign_extend = inst[15:0], alu_ctl_func = inst[5:0];
    assign alu_ALU_ctl = alu_control_operation;
    // 4.
    wire [4:0] EX_write_reg;
    assign EX_write_reg = ID_EX_EX[6]? ID_EX_instr_15 : ID_EX_instr_20; // reg_dst = ID_EX_EX[6]
    // 5.
    reg        EX_MEM_alu_zero, EX_MEM_alu_overflow;
    // reg [1:0]  EX_MEM_WB;
    reg        EX_MEM_reg_dst;
    reg [3:0]  EX_MEM_M;
    reg [4:0]  EX_MEM_write_reg, EX_MEM_rt;
    reg [31:0] EX_MEM_branch_target, /*EX_MEM_alu_result,*/ EX_MEM_read_data_2, EX_MEM_jump_target;
    always @(posedge clk)
        if (rstn) begin
            EX_MEM_WB            <= ID_EX_WB;           // a.
            EX_MEM_M             <= ID_EX_M;            // a.
            EX_MEM_branch_target <= EX_branch_target;   // b.
            EX_MEM_alu_zero      <= alu_zero;           // c.
            EX_MEM_alu_result    <= alu_result;         // d.
            EX_MEM_alu_overflow  <= alu_overflow;       // d. (alu overflow)
            EX_MEM_read_data_2   <= ID_EX_read_data_2;  // f.
            EX_MEM_write_reg     <= EX_write_reg;       // e.
            EX_MEM_jump_target   <= EX_jump_target;     // jump
            EX_MEM_rd            <= EX_write_reg;       // lab 4
            EX_MEM_reg_dst       <= ID_EX_EX[6];
            EX_MEM_rt            <= ID_EX_rt;
        end
    always @(negedge rstn) begin
        EX_MEM_WB            <= 0;  // a.
        EX_MEM_M             <= 0;  // a.
        EX_MEM_branch_target <= 0;  // b.
        EX_MEM_alu_zero      <= 0;  // c.
        EX_MEM_alu_result    <= 0;  // d.
        EX_MEM_alu_overflow  <= 0;  // d. (alu overflow)
        EX_MEM_read_data_2   <= 0;  // f.
        EX_MEM_write_reg     <= 0;  // e.
        EX_MEM_jump_target   <= 0;  // jump
        EX_MEM_rd            <= 0;  // lab 4
        EX_MEM_reg_dst       <= 0;
        EX_MEM_rt            <= 0;
    end

// Memory access (MEM)  (lab 4 step 5, branch decision is moved from MEM to ID)
    // 0. M = {control_branch, control_mem_read, control_mem_write, control_jump}
    // 1.
    wire pc_src;
    assign pc_src = EX_MEM_M[3] & EX_MEM_alu_zero;      // control_branch = EX_MEM_M[3]
    // 2.
    reg sw_forward_ctl;
    assign data_mem_address    = EX_MEM_alu_result;
    assign data_mem_write_data = sw_forward_ctl? reg_file_write_data : EX_MEM_read_data_2;
    // 3.
    assign data_mem_mem_write = EX_MEM_M[1];
    assign data_mem_mem_read  = EX_MEM_M[2];
    // 4.
    // reg [1:0]  MEM_WB_WB;
    reg [4:0]  MEM_WB_write_reg;
    reg [32:0] MEM_WB_data_mem_read_data, MEM_WB_alu_result;
    always @(posedge clk)
        if (rstn) begin
            MEM_WB_WB                 <= EX_MEM_WB;             // 4. a
            MEM_WB_data_mem_read_data <= data_mem_read_data;    // 4. b
            MEM_WB_alu_result         <= EX_MEM_alu_result;     // 4. c
            MEM_WB_write_reg          <= EX_MEM_write_reg;      // 4. d
            MEM_WB_rd                 <= EX_MEM_write_reg;      // lab 4
        end
    always @(negedge rstn) begin
        MEM_WB_WB                 <= 0; // 4. a
        MEM_WB_data_mem_read_data <= 0; // 4. b
        MEM_WB_alu_result         <= 0; // 4. c
        MEM_WB_write_reg          <= 0; // 4. d
        MEM_WB_rd                 <= 0; // lab 4
    end

// Write-back (WB)
    // WB = {mem_to_reg, reg_write}
    // 1.
    assign reg_file_reg_write = MEM_WB_WB[0];
    // 2.
    assign reg_file_write_data = MEM_WB_WB[1]? MEM_WB_data_mem_read_data : MEM_WB_alu_result;
    // 3.
    assign reg_file_write_reg = MEM_WB_write_reg;

    /** [step 5] Control Hazard
     * This is the most difficult part since the textbook does not provide enough information.
     * By reading p.377-379 "Reducing the Delay of Branches",
     * we can disassemble this into the following steps:
     * 1. Move branch target address calculation & taken or not from EX to ID
     * 2. Move branch decision from MEM to ID
     * 3. Add forwarding for registers used in branch decision from EX/MEM
     * 4. Add stalling:
          branch read registers right after an ALU instruction writes it -> 1 stall
          branch read registers right after a load instruction writes it -> 2 stalls
     */
    // WB = {mem_to_reg, reg_write}
    // branch forward
    always @(*) begin
        if(control_branch) begin
            // operation
            if(EX_MEM_reg_dst && EX_MEM_write_reg == IF_ID_instr[25:21])
                beq_A <= 2'b10;
            // lw
            else if(MEM_WB_WB[1] && MEM_WB_write_reg == IF_ID_instr[25:21])
                beq_A <= 2'b01;
            else
                beq_A <= 2'b00;

            // operation
            if(EX_MEM_reg_dst && EX_MEM_write_reg == IF_ID_instr[20:16])
                beq_B <= 2'b10;
            // lw
            else if(MEM_WB_WB[1] && MEM_WB_write_reg == IF_ID_instr[20:16])
                beq_B <= 2'b01;
            else
                beq_B <= 2'b00;
        end
        else begin
            beq_A <= 2'b00;
            beq_B <= 2'b00;
        end
    end

    // sw forward
    // M  = {branch, mem_read, mem_write, jump}
    always @(*) begin
        if(EX_MEM_M[1] && MEM_WB_WB[1] && EX_MEM_rt != 0 && EX_MEM_rt == MEM_WB_rd)
            sw_forward_ctl <= 1;
        else
            sw_forward_ctl <= 0;
    end

    

endmodule  // pipelined


/*
wire branch_hazard_1 = take_branch &&
((EX_MEM_reg_write && (IF_ID_rs == EX_MEM_rd || IF_ID_rt == EX_MEM_rd))||
(MEM_WB_mem_to_reg && (IF_ID_rs == MEM_WB_rd || IF_ID_rt == MEM_WB_rd)));

wire branch_hazard_2 = take_branch && (ID_EX_mem_read || EX_MEM_mem_read);
*/