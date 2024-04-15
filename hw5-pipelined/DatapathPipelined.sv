`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`ifndef RISCV_FORMAL
`include "../hw2b/cla.sv"
`include "../hw3-singlecycle/RvDisassembler.sv"
`include "../hw4-multicycle/divider_unsigned_pipelined.sv"
`endif

module Disasm #(
    byte PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
  // synthesis translate_off
  // this code is only for simulation, not synthesis
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic. Also,
  // string needs to be reversed to render correctly.
  genvar i;
  for (i = 3; i < 32; i = i + 1) begin : gen_disasm
    assign disasm[((i+1-3)*8)-1-:8] = disasm_string[31-i];
  end
  assign disasm[255-:8] = PREFIX;
  assign disasm[247-:8] = ":";
  assign disasm[239-:8] = " ";
  // synthesis translate_on
endmodule

module RegFile (
    input logic [4:0] rd,
    input logic [`REG_SIZE] rd_data,
    input logic [4:0] rs1,
    output logic [`REG_SIZE] rs1_data,
    input logic [4:0] rs2,
    output logic [`REG_SIZE] rs2_data,

    input logic clk,
    input logic we,
    input logic rst
);
  localparam int NumRegs = 32;
  integer i;
  logic [`REG_SIZE] regs[NumRegs];

  // TODO: your code here
  assign regs[0] = 32'd0; 


  assign rs1_data = regs[rs1]; 
  assign rs2_data = regs[rs2]; 

  always_ff @(negedge clk) begin
    if (rst == 1'b1) begin
      for (i=0; i < NumRegs; i++) begin
        regs[i] <= 32'd0;
      end
    end else begin
      if ((we==1'b1) && (rd != 5'd0)) begin 
        regs[rd] <= rd_data;
      end
    end
  end

endmodule

/**
 * This enum is used to classify each cycle as it comes through the Writeback stage, identifying
 * if a valid insn is present or, if it is a stall cycle instead, the reason for the stall. The
 * enum values are mutually exclusive: only one should be set for any given cycle. These values
 * are compared against the trace-*.json files to ensure that the datapath is running with the
 * correct timing.
 *
 * You will need to set these values at various places within your pipeline, and propagate them
 * through the stages until they reach Writeback where they can be checked.
 */
typedef enum {
  /** invalid value, this should never appear after the initial reset sequence completes */
  CYCLE_INVALID = 0,
  /** a stall cycle that arose from the initial reset signal */
  CYCLE_RESET = 1,
  /** not a stall cycle, a valid insn is in Writeback */
  CYCLE_NO_STALL = 2,
  /** a stall cycle that arose from a taken branch/jump */
  CYCLE_TAKEN_BRANCH = 4,

  // the values below are only needed in HW5B

  /** a stall cycle that arose from a load-to-use stall */
  CYCLE_LOAD2USE = 8,
  /** a stall cycle that arose from a div/rem-to-use stall */
  CYCLE_DIV2USE = 16,
  /** a stall cycle that arose from a fence.i insn */
  CYCLE_FENCEI = 32
} cycle_status_e;

typedef struct packed {
  logic insn_lui;
  logic insn_auipc;
  logic insn_jal;
  logic insn_jalr;

  logic insn_beq;
  logic insn_bne;
  logic insn_blt;
  logic insn_bge;
  logic insn_bltu;
  logic insn_bgeu;

  logic insn_lb;
  logic insn_lh;
  logic insn_lw;
  logic insn_lbu;
  logic insn_lhu;

  logic insn_sb;
  logic insn_sh;
  logic insn_sw;

  logic insn_addi;
  logic insn_slti;
  logic insn_sltiu;
  logic insn_xori;
  logic insn_ori;
  logic insn_andi;

  logic insn_slli;
  logic insn_srli;
  logic insn_srai;

  logic insn_add;
  logic insn_sub ;
  logic insn_sll ;
  logic insn_slt;
  logic insn_sltu ;
  logic insn_xor ;
  logic insn_srl;
  logic insn_sra;
  logic insn_or;
  logic insn_and;

  logic insn_mul;
  logic insn_mulh;
  logic insn_mulhsu;
  logic insn_mulhu;
  logic insn_div;
  logic insn_divu;
  logic insn_rem;
  logic insn_remu;

  logic insn_ecall;
  logic insn_fence;
} exectue_ins;

/** state at the start of Decode stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic [4:0] rd_num;
  logic [4:0] rs1_num;
  logic [`REG_SIZE] rs1_data_temp;
  logic [4:0] rs2_num;
  logic [`REG_SIZE] rs2_data_temp;
  logic [6:0] insn_funct7;
  logic [2:0] insn_funct3;
  logic [`REG_SIZE] addr_to_dmem;
  logic [3:0] store_we_to_dmem;
  logic [`REG_SIZE] store_data_to_dmem;
  logic [`REG_SIZE] insn_imem;
  logic [`REG_SIZE] imm_i_sz_ext;
  logic [`OPCODE_SIZE] insn_opcode;
  exectue_ins exe_control;
} stage_decode_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic [4:0] rd_num;
  logic [`REG_SIZE] rd_val;
  logic [4:0] rs1_num;
  logic [`REG_SIZE] rs1_data_temp;
  logic [4:0] rs2_num;
  logic [`REG_SIZE] rs2_data_temp;
  logic [`REG_SIZE] addr_to_dmem;
  logic [3:0] store_we_to_dmem;
  logic [`REG_SIZE] store_data_to_dmem;
  logic [`REG_SIZE] insn_imem;
  logic [`REG_SIZE] imm_i_sz_ext;
  logic [`OPCODE_SIZE] insn_opcode;
  exectue_ins exe_control;
} stage_execute_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic [4:0] rd_num;
  logic [`REG_SIZE] rd_val;
  logic [4:0] rs1_num;
  logic [`REG_SIZE] rs1_data_temp;
  logic [4:0] rs2_num;
  logic [`REG_SIZE] rs2_data_temp;
  logic [`REG_SIZE] addr_to_dmem;
  logic [3:0] store_we_to_dmem;
  logic [`REG_SIZE] store_data_to_dmem;
  logic [`OPCODE_SIZE] insn_opcode;
  logic sig_halt;
  logic branch_taken;
  logic [`REG_SIZE] pc_nxt;
} stage_memory_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic [4:0] rd_num;
  logic [`REG_SIZE] rd_val;
  logic [4:0] rs1_num;
  logic [`REG_SIZE] rs1_data_temp;
  logic [4:0] rs2_num;
  logic [`REG_SIZE] rs2_data_temp;
  logic [`OPCODE_SIZE] insn_opcode;
  logic sig_halt;
} stage_writeback_t;

module DatapathPipelined (
    input wire clk,
    input wire rst,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`INSN_SIZE] insn_from_imem,
    // dmem is read/write
    output logic [`REG_SIZE] addr_to_dmem,
    input wire [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See cycle_status_e enum for valid values.
    output cycle_status_e trace_writeback_cycle_status
);

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpcodeLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpcodeMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpcodeJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpcodeRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpcodeAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpcodeLui = 7'b01_101_11;

  // cycle counter, not really part of any stage but useful for orienting within GtkWave
  // do not rename this as the testbench uses this value
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  /*****/
  /* FETCH STAGE */
  /*****/
  logic [`REG_SIZE] f_pc_current;
  logic [`REG_SIZE] pc_nxt; 
  wire [`REG_SIZE] f_insn;
  cycle_status_e f_cycle_status;

  logic div_u_first;

  // program counter
  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 32'd0;

    end else begin
        f_cycle_status <= CYCLE_NO_STALL;
        if (branch_taken == 1'b1) begin
          f_pc_current <= pc_nxt;
        end else begin
          f_pc_current <= f_pc_current + 4;
        end
    end
  end

  assign pc_to_imem = f_pc_current;
  assign f_insn = insn_from_imem;

  // Here's how to disassemble an insn into a string you can view in GtkWave.
  // Use PREFIX to provide a 1-character tag to identify which stage the insn comes from.
  wire [255:0] f_disasm;
  Disasm #(
      .PREFIX("F")
  ) disasm_0fetch (
      .insn  (f_insn),
      .disasm(f_disasm)
  );

  wire [6:0] insn_funct7;
  wire [4:0] insn_rs2;
  wire [4:0] insn_rs1;
  wire [2:0] insn_funct3;
  wire [4:0] insn_rd;
  wire [`OPCODE_SIZE] insn_opcode;


  exectue_ins exe_control_temp;
  logic [`REG_SIZE] rs1_data_temp;
  logic [`REG_SIZE] rs2_data_temp;
  logic [4:0] rs2_val_or_not;


  // split R-type instruction - see section 2.2 of RiscV spec

  assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn_from_imem;
    // B - conditionals
  wire [12:0] imm_b_temp;
  assign {imm_b_temp[12], imm_b_temp[10:5]} = insn_funct7, {imm_b_temp[4:1], imm_b_temp[11]} = insn_rd, imm_b_temp[0] = 1'b0;

  logic [`REG_SIZE] imm_b_sext_temp;
  assign imm_b_sext_temp = {{19{imm_b_temp[12]}}, imm_b_temp[12:0]};

  /******/
  /* DECODE STAGE */
  /******/

  stage_decode_t decode_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      decode_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET,
        rs1_num: 0,
        rs1_data_temp: 0,
        rs2_num: 0,
        rs2_data_temp: 0,
        rd_num: 0,
        insn_funct7: 0,
        insn_funct3: 0,
        addr_to_dmem: 0,
        store_we_to_dmem: 0,
        store_data_to_dmem: 0,
        insn_imem: 0,
        imm_i_sz_ext: 0,
        insn_opcode: 0,
        exe_control: '{default:0}
      };
    end else begin
      begin
        if (branch_taken == 1'b1) begin
          decode_state <= 0;
        end else begin
          decode_state <= '{
          pc: f_pc_current,
          insn: f_insn,
          cycle_status: f_cycle_status,
          rd_num: insn_opcode == 7'h63 ? 0 : insn_rd,
          rs1_num: insn_opcode == 7'h37 ? 0: insn_rs1,
          rs1_data_temp: rs1_data_temp,
          rs2_num: ((insn_opcode == 7'h13) || (insn_opcode == 7'h37)) ? 0: insn_rs2,
          rs2_data_temp: rs2_data_temp,
          insn_funct7: insn_opcode == 7'h37 ? 0: insn_funct7,
          insn_funct3: insn_opcode == 7'h37 ? 0: insn_funct3,
          addr_to_dmem: 0,
          store_we_to_dmem: 0,
          store_data_to_dmem: 0,
          insn_imem: insn_from_imem,
          imm_i_sz_ext: 0,
          insn_opcode: insn_opcode,
          exe_control: '{default:0}
        };
        end 
      end
    end
  end
  wire [255:0] decode_disasm;
  Disasm #(
      .PREFIX("D")
  ) disasm_1decode (
      .insn  (decode_state.insn),
      .disasm(decode_disasm)
  );

  // setup for I, S, B & J type instructions
  // I - short immediates and loads
  wire [11:0] imm_i;
  assign imm_i = decode_state.insn_imem[31:20];
  wire [ 4:0] imm_shamt = decode_state.insn_imem[24:20];

  // S - stores
  wire [11:0] imm_s;

  assign imm_s[11:5] = decode_state.insn_funct7, imm_s[4:0] = decode_state.insn_imem[11:7]; 

  // B - conditionals
  wire [12:0] imm_b;

  assign {imm_b[12], imm_b[10:5]} = decode_state.insn_funct7, {imm_b[4:1], imm_b[11]} = decode_state.insn_imem[11:7], imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {decode_state.insn_imem[31:12], 1'b0};
  
  // U - Immidiates 
  wire [19:0] imm_u; 
  assign imm_u = decode_state.insn_imem[31:12];

 
  logic [1:0] mux_val_wd;
  logic [`REG_SIZE] rs1_mux_data;
  logic [`REG_SIZE] rs2_mux_data;
  logic [4:0] wd_rd_num;
  assign wd_rd_num = write_back_state.rd_num;

  logic [`REG_SIZE] imm_i_sext;
  logic [`REG_SIZE] imm_i_ext;
  logic [`REG_SIZE] imm_s_sext;
  logic [`REG_SIZE] imm_b_sext;
  logic [`REG_SIZE] imm_j_sext;
  logic [`REG_SIZE] imm_u_ext;
  
  logic [`REG_SIZE] imm_i_sz_ext;

  assign imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  assign imm_i_ext = {{20{1'b0}}, imm_i[11:0]};
  assign imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  assign imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  assign imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};
  assign imm_u_ext = {{12{1'b0}},imm_u[19:0]};

  assign exe_control_temp.insn_lui = decode_state.insn_opcode == OpcodeLui;
  assign exe_control_temp.insn_auipc = decode_state.insn_opcode == OpcodeAuipc;
  assign exe_control_temp.insn_jal = decode_state.insn_opcode == OpcodeJal;
  assign exe_control_temp.insn_jalr = decode_state.insn_opcode == OpcodeJalr;

  assign exe_control_temp.insn_addi = decode_state.insn_opcode == OpcodeRegImm && decode_state.insn_imem[14:12] == 3'b000;
  assign exe_control_temp.insn_slti = decode_state.insn_opcode == OpcodeRegImm && decode_state.insn_imem[14:12] == 3'b010;
  assign exe_control_temp.insn_sltiu = decode_state.insn_opcode == OpcodeRegImm && decode_state.insn_imem[14:12] == 3'b011;
  assign exe_control_temp.insn_xori = decode_state.insn_opcode == OpcodeRegImm && decode_state.insn_imem[14:12] == 3'b100;
  assign exe_control_temp.insn_ori = decode_state.insn_opcode == OpcodeRegImm && decode_state.insn_imem[14:12] == 3'b110;
  assign exe_control_temp.insn_andi = decode_state.insn_opcode == OpcodeRegImm && decode_state.insn_imem[14:12] == 3'b111;

  assign exe_control_temp.insn_beq = decode_state.insn_opcode == OpcodeBranch && decode_state.insn_imem[14:12] == 3'b000;
  assign exe_control_temp.insn_bne = decode_state.insn_opcode == OpcodeBranch && decode_state.insn_imem[14:12] == 3'b001;
  assign exe_control_temp.insn_blt = decode_state.insn_opcode == OpcodeBranch && decode_state.insn_imem[14:12] == 3'b100;
  assign exe_control_temp.insn_bge = decode_state.insn_opcode == OpcodeBranch && decode_state.insn_imem[14:12] == 3'b101;
  assign exe_control_temp.insn_bltu = decode_state.insn_opcode == OpcodeBranch && decode_state.insn_imem[14:12] == 3'b110;
  assign exe_control_temp.insn_bgeu = decode_state.insn_opcode == OpcodeBranch && decode_state.insn_imem[14:12] == 3'b111;

  assign exe_control_temp.insn_lb = decode_state.insn_opcode == OpcodeLoad && decode_state.insn_imem[14:12] == 3'b000;
  assign exe_control_temp.insn_lh = decode_state.insn_opcode == OpcodeLoad && decode_state.insn_imem[14:12] == 3'b001;
  assign exe_control_temp.insn_lw = decode_state.insn_opcode == OpcodeLoad && decode_state.insn_imem[14:12] == 3'b010;
  assign exe_control_temp.insn_lbu = decode_state.insn_opcode == OpcodeLoad && decode_state.insn_imem[14:12] == 3'b100;
  assign exe_control_temp.insn_lhu = decode_state.insn_opcode == OpcodeLoad && decode_state.insn_imem[14:12] == 3'b101;

  assign exe_control_temp.insn_sb = decode_state.insn_opcode == OpcodeStore && decode_state.insn_imem[14:12] == 3'b000;
  assign exe_control_temp.insn_sh = decode_state.insn_opcode == OpcodeStore && decode_state.insn_imem[14:12] == 3'b001;
  assign exe_control_temp.insn_sw = decode_state.insn_opcode == OpcodeStore && decode_state.insn_imem[14:12] == 3'b010;

  
  assign exe_control_temp.insn_slli = decode_state.insn_opcode == OpcodeRegImm && decode_state.insn_imem[14:12] == 3'b001 && decode_state.insn_imem[31:25] == 7'd0;
  assign exe_control_temp.insn_srli = decode_state.insn_opcode == OpcodeRegImm && decode_state.insn_imem[14:12] == 3'b101 && decode_state.insn_imem[31:25] == 7'd0;
  assign exe_control_temp.insn_srai = decode_state.insn_opcode == OpcodeRegImm && decode_state.insn_imem[14:12] == 3'b101 && decode_state.insn_imem[31:25] == 7'b0100000;

  
  assign exe_control_temp.insn_mul    = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[31:25] == 7'd1 && decode_state.insn_imem[14:12] == 3'b000;
  assign exe_control_temp.insn_mulh   = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[31:25] == 7'd1 && decode_state.insn_imem[14:12] == 3'b001;
  assign exe_control_temp.insn_mulhsu = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[31:25] == 7'd1 && decode_state.insn_imem[14:12] == 3'b010;
  assign exe_control_temp.insn_mulhu  = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[31:25] == 7'd1 && decode_state.insn_imem[14:12] == 3'b011;
  assign exe_control_temp.insn_div    = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[31:25] == 7'd1 && decode_state.insn_imem[14:12] == 3'b100;
  assign exe_control_temp.insn_divu   = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[31:25] == 7'd1 && decode_state.insn_imem[14:12] == 3'b101;
  assign exe_control_temp.insn_rem    = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[31:25] == 7'd1 && decode_state.insn_imem[14:12] == 3'b110;
  assign exe_control_temp.insn_remu   = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[31:25] == 7'd1 && decode_state.insn_imem[14:12] == 3'b111;

  assign exe_control_temp.insn_add = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[14:12] == 3'b000 && decode_state.insn_imem[31:25] == 7'd0;
  assign exe_control_temp.insn_sub  = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[14:12] == 3'b000 && decode_state.insn_imem[31:25] == 7'b0100000;
  assign exe_control_temp.insn_sll = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[14:12] == 3'b001 && decode_state.insn_imem[31:25] == 7'd0;
  assign exe_control_temp.insn_slt = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[14:12] == 3'b010 && decode_state.insn_imem[31:25] == 7'd0;
  assign exe_control_temp.insn_sltu = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[14:12] == 3'b011 && decode_state.insn_imem[31:25] == 7'd0;
  assign exe_control_temp.insn_xor = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[14:12] == 3'b100 && decode_state.insn_imem[31:25] == 7'd0;
  assign exe_control_temp.insn_srl = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[14:12] == 3'b101 && decode_state.insn_imem[31:25] == 7'd0;
  assign exe_control_temp.insn_sra  = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[14:12] == 3'b101 && decode_state.insn_imem[31:25] == 7'b0100000;
  assign exe_control_temp.insn_or = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[14:12] == 3'b110 && decode_state.insn_imem[31:25] == 7'd0;
  assign exe_control_temp.insn_and = decode_state.insn_opcode == OpcodeRegReg && decode_state.insn_imem[14:12] == 3'b111 && decode_state.insn_imem[31:25] == 7'd0;

  assign exe_control_temp.insn_ecall = decode_state.insn_opcode == OpcodeEnviron && decode_state.insn_imem[31:7] == 25'd0;
  assign exe_control_temp.insn_fence = decode_state.insn_opcode == OpcodeMiscMem;

  always_comb begin
    imm_i_sz_ext = 0;
    if (exe_control_temp.insn_jalr || exe_control_temp.insn_addi || exe_control_temp.insn_slti || exe_control_temp.insn_sltiu || exe_control_temp.insn_xori || exe_control_temp.insn_ori || exe_control_temp.insn_andi || (decode_state.insn_opcode == OpcodeLoad)) begin
      imm_i_sz_ext = imm_i_sext;
    end else if (exe_control_temp.insn_slli || exe_control_temp.insn_srli || exe_control_temp.insn_srai) begin
      imm_i_sz_ext = imm_i_ext;
    end else if (decode_state.insn_opcode == OpcodeStore) begin
      imm_i_sz_ext = imm_s_sext;
    end else if (decode_state.insn_opcode == OpcodeBranch) begin
      imm_i_sz_ext = imm_b_sext;
    end else if (decode_state.insn_opcode == OpcodeJal) begin
      imm_i_sz_ext = imm_j_sext;
    end else if ((decode_state.insn_opcode == OpcodeLui) || (decode_state.insn_opcode == OpcodeAuipc)) begin
      imm_i_sz_ext = imm_u_ext;
    end

    rs1_mux_data = rs1_data_temp;
    rs2_mux_data = rs2_data_temp;

    mux_val_wd = 2'b0;

    if (wd_rd_num !=0) begin  
      if (wd_rd_num == decode_state.rs1_num) begin
        mux_val_wd = 2'b01;
        rs1_mux_data = write_back_state.rd_val;
      end else if (wd_rd_num == decode_state.rs2_num) begin
        mux_val_wd = 2'b10;
        rs2_mux_data = write_back_state.rd_val;
      end
    end

    x_state_temp = '{
    pc: decode_state.pc,
    insn: decode_state.insn,
    cycle_status: decode_state.cycle_status,
    rs1_num: decode_state.rs1_num,
    rs1_data_temp: rs1_mux_data,
    rs2_num: decode_state.rs2_num,
    rs2_data_temp: rs2_mux_data, 
    rd_num: decode_state.rd_num,
    rd_val: 0,
    addr_to_dmem: decode_state.addr_to_dmem,
    store_we_to_dmem: decode_state.store_we_to_dmem,
    store_data_to_dmem: decode_state.store_data_to_dmem,
    insn_imem: decode_state.insn_imem,
    imm_i_sz_ext: imm_i_sz_ext,
    insn_opcode: decode_state.insn_opcode,
    exe_control: exe_control_temp
    };
    
  end

  /******/
  /* EXECUTE STAGE */
  /******/
  stage_execute_t x_state;
  stage_execute_t x_state_temp;

  always_ff @(posedge clk) begin
    if (rst) begin
      x_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET,
        rs1_num: 0,
        rs1_data_temp: 0,
        rs2_num: 0,
        rs2_data_temp: 0,
        rd_num: 0,
        rd_val: 0,
        addr_to_dmem: 0,
        store_we_to_dmem: 0,
        store_data_to_dmem: 0,
        insn_imem: 0,
        imm_i_sz_ext: 0,
        insn_opcode: 0,
        exe_control: '{default:0}
      };
    end else begin
      begin
        if (branch_taken == 1'b1) begin
            x_state <= 0;
        end else begin
            x_state <= x_state_temp;
        end 
      end
    end
  end

  wire [255:0] e_disasm;
  Disasm #(
      .PREFIX("E")
  ) disasm_1execute (
      .insn  (x_state.insn),
      .disasm(e_disasm)
  );


  logic we1;
  logic [63:0] result_mul;
  logic [31:0] result_mul_signed;
  logic [63:0] result_mul_store;
  logic sig_halt;
  logic sig_halt_temp;
  logic [31:0] temp_dividend;
  logic [31:0] divisor_temp;
  logic [31:0] rem_temp_o;
  logic [31:0] quo_temp_o;
  logic c_in;
  logic [`REG_SIZE] final_sum;
  logic [`REG_SIZE] sum_a, sum_b;
  logic illegal_insn;
  logic [4:0] rd; 
  logic [`REG_SIZE] rd_data;
  logic [4:0] rs1; 
  logic [4:0] rs2; 
  logic [`REG_SIZE] store_temp_dmem_addr;
  logic [3:0] dmem_temp_store_we_to_;
  logic [`REG_SIZE] temp_pcCurrent;
  


  logic [31:0] bits_address;
  logic [`REG_SIZE] temp_dmem_addr;



  
  logic [`REG_SIZE] inc_pc;

  logic rs1_temp;
  logic rs2_temp;
  logic branch_taken;
  logic [`REG_SIZE] rd_temp;

  logic [`OPCODE_SIZE] opcode_x;

  logic [4:0] rd_mem_num;
  assign rd_mem_num = memory_state.rd_num;
  
  logic [4:0] writeback_rd_num;
  assign writeback_rd_num = write_back_state.rd_num;

  logic [4:0] rs1_num_exec;
  assign rs1_num_exec = x_state.rs1_num;

  logic [4:0] rs2_num_exec;
  assign rs2_num_exec = x_state.rs2_num;

  logic [`REG_SIZE] rs1_data_exec;

  logic [`REG_SIZE] rs2_data_exec;

  logic [3:0] mx_wx_multiplex;

  assign opcode_x = x_state.insn_opcode;

  RegFile rf(.rd(write_back_state.rd_num), .rd_data(write_back_state.rd_val), .rs1(decode_state.rs1_num), .rs1_data(rs1_data_temp), 
  .rs2(decode_state.rs2_num), .rs2_data(rs2_data_temp), .clk(clk), .we(we1), .rst(rst));

  cla alu (.a(sum_a), .b(sum_b), .cin(c_in), .sum(final_sum));

  divider_unsigned_pipelined div(.clk(clk), .rst(rst), .i_dividend(temp_dividend),
    .i_divisor(divisor_temp), .o_quotient(quo_temp_o),
    .o_remainder(rem_temp_o));
  always_comb begin
  rs1_data_exec = x_state.rs1_data_temp;
  rs2_data_exec = x_state.rs2_data_temp;

  mx_wx_multiplex = 0;

    if (rd_mem_num != 0 || writeback_rd_num != 0) begin
      if (rd_mem_num == rs1_num_exec && rd_mem_num != rs2_num_exec && rd_mem_num != 0) begin
        mx_wx_multiplex = 1;
        rs1_data_exec = memory_state.rd_val;
      end if (rd_mem_num == rs2_num_exec && rd_mem_num != rs1_num_exec  && rd_mem_num != 0) begin
        mx_wx_multiplex = 2;
        rs2_data_exec = memory_state.rd_val;
      end if (rd_mem_num == rs1_num_exec && rd_mem_num == rs2_num_exec && rd_mem_num != 0) begin
        mx_wx_multiplex = 3;
        rs1_data_exec = memory_state.rd_val;
        rs2_data_exec = memory_state.rd_val;
      end if (writeback_rd_num == rs1_num_exec && writeback_rd_num != rs2_num_exec && writeback_rd_num != rd_mem_num && writeback_rd_num != 0) begin
        mx_wx_multiplex = 4;
        rs1_data_exec = write_back_state.rd_val;
      end if (writeback_rd_num == rs2_num_exec && writeback_rd_num != rs1_num_exec && writeback_rd_num != rd_mem_num && writeback_rd_num != 0) begin
        mx_wx_multiplex = 5;
        rs2_data_exec = write_back_state.rd_val;
      end if (writeback_rd_num == rs1_num_exec && writeback_rd_num == rs2_num_exec && writeback_rd_num != rd_mem_num && writeback_rd_num != 0) begin
        mx_wx_multiplex = 6;
        rs2_data_exec = write_back_state.rd_val;
        rs1_data_exec = write_back_state.rd_val;
      end if (rd_mem_num == rs1_num_exec && writeback_rd_num == rs2_num_exec && rs1_num_exec != rs2_num_exec && (writeback_rd_num != 0 && rd_mem_num != 0)) begin
        mx_wx_multiplex = 7;
        rs1_data_exec = memory_state.rd_val;
        rs2_data_exec = write_back_state.rd_val;
      end if (rd_mem_num == rs2_num_exec && writeback_rd_num == rs1_num_exec && rs1_num_exec != rs2_num_exec && (writeback_rd_num != 0 && rd_mem_num != 0)) begin
        mx_wx_multiplex = 8;
        rs2_data_exec = memory_state.rd_val;
        rs1_data_exec = write_back_state.rd_val;
      end

    end

  c_in = 1'b0;
  branch_taken = 1'b0;
  illegal_insn = 1'b0;
  result_mul = 64'b0;
  result_mul_signed = 32'b0;
  result_mul_store = 64'b0;
  pc_nxt = 0;
  rd_temp = 32'd0;
  divisor_temp = 32'b0;
  temp_dividend = 32'b0;
  sig_halt_temp = 1'b0;
  sum_a = $signed(rs1_data_exec);
  sum_b = $signed(rs2_data_exec);

  case (opcode_x)
      OpcodeMiscMem: begin
          if(x_state.exe_control.insn_fence) begin 
          end else begin
            illegal_insn = 1'b1;
          end 
    end
		
	  OpcodeEnviron: begin
          if(x_state.exe_control.insn_ecall) begin
            sig_halt_temp = 1'b1;
          end
       end

    OpcodeLui: begin
      if(x_state.rd_num == 5'b0)
        rd_temp = 32'b0;
      else begin
        rd_temp = {x_state.insn_imem[31:12], 12'd0};
      end
		end

    OpcodeBranch: begin
      if(x_state.exe_control.insn_beq) begin 
        if(rs1_data_exec == rs2_data_exec) begin 
          pc_nxt = x_state.pc + x_state.imm_i_sz_ext;
          branch_taken = 1'b1;
        end
        else begin 
          branch_taken = 1'b0;
        end 
      end else
      if(x_state.exe_control.insn_bne)begin
        if (rs1_data_exec != rs2_data_exec) begin
          pc_nxt = x_state.pc + x_state.imm_i_sz_ext;
          branch_taken = 1'b1;
          end
        else begin 
          branch_taken = 1'b0;
        end 
      end  
      else if(x_state.exe_control.insn_blt)begin 
        if($signed(rs1_data_exec) < $signed(rs2_data_exec)) begin
          pc_nxt = x_state.pc + x_state.imm_i_sz_ext;
          branch_taken = 1'b1;
        end 
        else begin 
          branch_taken = 1'b0;
        end
      end
      else if(x_state.exe_control.insn_bge)begin 
        if($signed(rs1_data_exec) >= $signed(rs2_data_exec)) begin
          pc_nxt = x_state.pc + x_state.imm_i_sz_ext;
          branch_taken = 1'b1;
        end
        else begin 
          branch_taken = 1'b0;
        end 
      end 
      else if(x_state.exe_control.insn_bltu)begin 
        if($signed(rs1_data_exec) < $unsigned(rs2_data_exec)) begin
          pc_nxt = x_state.pc + x_state.imm_i_sz_ext;
          branch_taken = 1'b1;
        end
        else begin 
          branch_taken = 1'b0;
        end
      end
      else if(x_state.exe_control.insn_bgeu)begin 
        if($signed(rs1_data_exec) >= $unsigned(rs2_data_exec)) begin
          pc_nxt = x_state.pc + x_state.imm_i_sz_ext;
          branch_taken = 1'b1;
        end
        else begin 
          branch_taken = 1'b0;
        end
      end 
      else begin 
      end       
    end 

    OpcodeRegImm: begin 
      if(x_state.exe_control.insn_addi) begin 
          sum_a = rs1_data_exec;
          sum_b = x_state.imm_i_sz_ext;
          rd_temp = final_sum;
      end
      else if (x_state.exe_control.insn_slti) begin 
        if($signed(x_state.imm_i_sz_ext) > $signed(rs1_data_exec))
          rd_temp = 32'b1;
        else
          rd_temp = 32'b0;
      end
      else if(x_state.exe_control.insn_sltiu) begin
        if($signed(rs1_data_exec) < $unsigned(x_state.imm_i_sz_ext))
          rd_temp = 32'b1;
        else
          rd_temp = 32'b0;
      end  

      else if(x_state.exe_control.insn_srai) begin
        rd_temp = ($signed(rs1_data_exec) >>> (x_state.imm_i_sz_ext[4:0]));
      end
     else if(x_state.exe_control.insn_xori) begin 
        rd_temp = $signed(rs1_data_exec) ^ x_state.imm_i_sz_ext;
      end 
      else if(x_state.exe_control.insn_ori) begin
        rd_temp = $signed(rs1_data_exec) | x_state.imm_i_sz_ext;
      end
      else if(x_state.exe_control.insn_andi) begin
        rd_temp = $signed(rs1_data_exec) & x_state.imm_i_sz_ext;
      end
      else if(x_state.exe_control.insn_slli) begin
        rd_temp = (rs1_data_exec << (x_state.imm_i_sz_ext[4:0]));
      end
      else if(x_state.exe_control.insn_srli) begin
        rd_temp = (rs1_data_exec >> (x_state.imm_i_sz_ext[4:0]));
      end
      else begin 
        illegal_insn = 1'b1;
      end 
    end

    OpcodeRegReg: begin
      if(x_state.exe_control.insn_add) begin 
        sum_a = rs1_data_exec;
        sum_b = rs2_data_exec;
        rd_temp = final_sum;
      end
      else if(x_state.exe_control.insn_sub) begin 
        sum_a = rs1_data_exec;
        sum_b = ~rs2_data_exec;
        c_in = 1'b1;
        rd_temp = final_sum;
      end
      else if(x_state.exe_control.insn_sll) begin 
        rd_temp = rs1_data_exec << rs2_data_exec[4:0];
      end
      else if(x_state.exe_control.insn_slt) begin  
        if($signed(rs1_data_exec) < $signed(rs2_data_exec)) 
          rd_temp = 32'b1;
        else 
          rd_temp = 32'b0;
      end
      else if(x_state.exe_control.insn_sltu) begin 
        rd_temp = (rs1_data_exec < $unsigned(rs2_data_exec))? 32'b1:32'b0;
      end
      else if(x_state.exe_control.insn_xor) begin 
        rd_temp = rs1_data_exec ^ rs2_data_exec;
      end
      else if(x_state.exe_control.insn_srl) begin 
        rd_temp = rs1_data_exec >> (rs2_data_exec[4:0]);
      end
      else if(x_state.exe_control.insn_sra) begin 
        rd_temp = $signed(rs1_data_exec) >>> (rs2_data_exec[4:0]);
      end
      else if(x_state.exe_control.insn_or) begin 
        rd_temp = rs1_data_exec | rs2_data_exec;
      end
      else if(x_state.exe_control.insn_and) begin 
        rd_temp = rs1_data_exec & rs2_data_exec;
      end
      else if(x_state.exe_control.insn_mul)begin 
        result_mul = (rs1_data_exec * rs2_data_exec);
        rd_temp = result_mul[31:0];
      end 
      else if(x_state.exe_control.insn_mulh)begin 
        result_mul = ($signed(rs1_data_exec) * $signed(rs2_data_exec));
        rd_temp = result_mul[63:32];
      end  
      else if(x_state.exe_control.insn_mulhsu)begin 
        result_mul_signed = (rs1_data_exec[31]) ? (~rs1_data_exec + 32'b1) : rs1_data_exec;
        result_mul = (result_mul_signed * $unsigned(rs2_data_exec));
        if(rs1_data_exec[31]) begin
          result_mul_store = ~result_mul + 64'b1;
        end 
        else begin
          result_mul_store = result_mul;
        end 
        rd_temp = result_mul_store[63:32];                     
      end
      else if(x_state.exe_control.insn_mulhu)begin 
        result_mul = ($unsigned(rs1_data_exec) *  $unsigned(rs2_data_exec));
        rd_temp = result_mul[63:32];
      end
      else if(x_state.exe_control.insn_div)begin 
        temp_dividend = (rs1_data_exec[31]) ? (~rs1_data_exec + 32'b1) : rs1_data_exec; 
        divisor_temp = (rs2_data_exec[31]) ? (~rs2_data_exec + 32'b1) : rs2_data_exec;
        if(( rs1_data_exec == 0 | rs2_data_exec == 0)) begin  
            rd_temp = $signed(32'hFFFF_FFFF);             
        end 
        else if(rs1_data_exec[31] != rs2_data_exec[31]) begin
          rd_temp = (~quo_temp_o + 32'b1);
        end 
        else begin 
          rd_temp = quo_temp_o;
        end 
      end
      else if(x_state.exe_control.insn_divu)begin 
        temp_dividend = $signed(rs1_data_exec); 
        divisor_temp =  $unsigned(rs2_data_exec);
        rd_temp = quo_temp_o;
      end
      else if (x_state.exe_control.insn_rem)begin 
        temp_dividend = (rs1_data_exec[31]) ? (~rs1_data_exec + 32'b1) : rs1_data_exec; 
        divisor_temp = (rs2_data_exec[31]) ? (~rs2_data_exec + 32'b1) : rs2_data_exec;
        if(rs1_data_exec == 32'b0) begin  
            rd_temp = (rs2_data_exec[31]) ? (~rs2_data_exec + 32'b1) : rs2_data_exec;             
        end 
        else if((rs1_data_exec[31])) begin
          rd_temp = (~rem_temp_o + 32'b1);
        end 
        else begin 
          rd_temp = rem_temp_o;
        end
      end 
      else if(x_state.exe_control.insn_remu)begin
        temp_dividend = $signed(rs1_data_exec); 
        divisor_temp =  $unsigned(rs2_data_exec);
        rd_temp = rem_temp_o;
      end  
      else begin 
        illegal_insn = 1'b1;
      end                  
    end
      OpcodeJal: begin
      if (x_state.exe_control.insn_jal) begin
        rd_temp = x_state.pc + 32'd4;
        pc_nxt = x_state.pc + x_state.imm_i_sz_ext;
        branch_taken = 1'b1;
      end 
      else begin 
        branch_taken = 1'b0;
      end
    end

    OpcodeJalr: begin
      if (x_state.exe_control.insn_jalr) begin 
        rd_temp = x_state.pc + 32'd4;
        pc_nxt = (($signed(rs1_data_exec) + $signed(x_state.imm_i_sz_ext)) & 32'hFFFFFFFE);
        branch_taken = 1'b1;
      end 
      else begin 
        branch_taken = 1'b0;
      end
    end 

    OpcodeStore: begin
    end

    OpcodeLoad: begin
    end

    default: begin
      illegal_insn = 1'b1;
    end 

    endcase
  end

  /******/
  /* MEMORY STAGE */
  /******/
  stage_memory_t memory_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      memory_state <= '{
        addr_to_dmem: 0,
        store_we_to_dmem: 0,
        store_data_to_dmem: 0,
        insn_opcode: 0,
        sig_halt: 0,
        branch_taken: 0,
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET,
        rs1_num: 0,
        rs1_data_temp: 0,
        rs2_num: 0,
        rs2_data_temp: 0,
        rd_num: 0,
        rd_val: 0,
        pc_nxt: 0
      };
    end else begin
      begin
        memory_state <= '{
          addr_to_dmem: x_state.addr_to_dmem,
          store_we_to_dmem: x_state.store_we_to_dmem,
          store_data_to_dmem: x_state.store_data_to_dmem,
          insn_opcode: x_state.insn_opcode,
          sig_halt: sig_halt_temp,
          branch_taken: branch_taken,
          pc: x_state.pc,
          insn: x_state.insn,
          cycle_status: x_state.cycle_status,
          rs1_num: x_state.rs1_num,
          rs1_data_temp: rs1_data_exec,
          rs2_num: x_state.rs2_num,
          rs2_data_temp: rs2_data_exec,
          rd_num: x_state.rd_num,
          rd_val: rd_temp,
          pc_nxt: pc_nxt
        };
      end
    end
  end
  wire [255:0] memory_disasm;
  Disasm #(
      .PREFIX("M")
  ) disasm_1memory (
      .insn  (memory_state.insn),
      .disasm(memory_disasm)
  );

  /******/
  /* WRITEBACK STAGE */
  /******/

  stage_writeback_t write_back_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      write_back_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET,
        rd_num: 0,
        rd_val: 0,
        rs1_num: 0,
        rs1_data_temp: 0,
        rs2_num: 0,
        rs2_data_temp: 0,
        insn_opcode: 0,
        sig_halt: 0
      };
    end else begin
      begin
        write_back_state <= '{
          pc: memory_state.pc,
          insn: memory_state.insn,
          cycle_status: memory_state.cycle_status,
          rs1_num: memory_state.rs1_num,
          rs1_data_temp: memory_state.rs1_data_temp,
          rs2_num: memory_state.rs2_num,
          rs2_data_temp: memory_state.rs2_data_temp,
          rd_num: memory_state.rd_num,
          rd_val: memory_state.rd_val,
          insn_opcode: memory_state.insn_opcode,
          sig_halt: memory_state.sig_halt
        };
      end
    end
  end

  wire [255:0] write_disasm;
  Disasm #(
      .PREFIX("W")
  ) disasm_1writeback (
      .insn  (write_back_state.insn),
      .disasm(write_disasm)
  );

  assign we1 = (write_back_state.insn_opcode == 7'h63 || write_back_state.insn_opcode == 7'h23) || (write_back_state.rd_num == 0)? 1'b0 : 1'b1;
  assign halt = write_back_state.sig_halt; 

  assign trace_writeback_cycle_status = write_back_state.cycle_status;



endmodule

module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. The memory reads/writes on @(negedge clk)
    input wire clk,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] pc_to_imem,

    // the value at memory location pc_to_imem
    output logic [`REG_SIZE] insn_from_imem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] addr_to_dmem,

    // the value at memory location addr_to_dmem
    output logic [`REG_SIZE] load_data_from_dmem,

    // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
    input wire [`REG_SIZE] store_data_to_dmem,

    // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
    // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
    input wire [3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  logic [`REG_SIZE] mem[NUM_WORDS];

  initial begin
    $readmemh("mem_initial_contents.hex", mem, 0);
  end

  always_comb begin
    // memory addresses should always be 4B-aligned
    assert (pc_to_imem[1:0] == 2'b00);
    assert (addr_to_dmem[1:0] == 2'b00);
  end

  localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam int AddrLsb = 2;

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      if (store_we_to_dmem[0]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
      end
      if (store_we_to_dmem[1]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
      end
      if (store_we_to_dmem[2]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
      end
      if (store_we_to_dmem[3]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
      end
      // dmem is "read-first": read returns value before the write
      load_data_from_dmem <= mem[{addr_to_dmem[AddrMsb:AddrLsb]}];
    end
  end
endmodule

/* This design has just one clock for both processor and memory. */
module RiscvProcessor (
    input  wire  clk,
    input  wire  rst,
    output logic halt,
    output wire [`REG_SIZE] trace_writeback_pc,
    output wire [`INSN_SIZE] trace_writeback_insn,
    output cycle_status_e trace_writeback_cycle_status
);

  wire [`INSN_SIZE] insn_from_imem;
  wire [`REG_SIZE] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) the_mem (
      .rst                (rst),
      .clk                (clk),
      // imem is read-only
      .pc_to_imem         (pc_to_imem),
      .insn_from_imem     (insn_from_imem),
      // dmem is read-write
      .addr_to_dmem       (mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem   (mem_data_we)
  );

  DatapathPipelined datapath (
      .clk(clk),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule
