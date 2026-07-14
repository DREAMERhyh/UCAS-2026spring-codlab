`timescale 10ns / 1ns
module custom_cpu(
    input         clk,
    input         rst,
    // 指令请求通道
    output [31:0] PC,
    output        Inst_Req_Valid,
    input         Inst_Req_Ready,
    // 指令应答通道
    input  [31:0] Instruction,
    input         Inst_Valid,
    output        Inst_Ready,
    // 数据请求通道
    output [31:0] Address,
    output        MemWrite,
    output [31:0] Write_data,
    output [ 3:0] Write_strb,
    output        MemRead,
    input         Mem_Req_Ready,
    // 数据应答通道
    input  [31:0] Read_data,
    input         Read_data_Valid,
    output        Read_data_Ready,
    input         intr,
    output [31:0] cpu_perf_cnt_0,
    output [31:0] cpu_perf_cnt_1,
    output [31:0] cpu_perf_cnt_2,
    output [31:0] cpu_perf_cnt_3,
    output [31:0] cpu_perf_cnt_4,
    output [31:0] cpu_perf_cnt_5,
    output [31:0] cpu_perf_cnt_6,
    output [31:0] cpu_perf_cnt_7,
    output [31:0] cpu_perf_cnt_8,
    output [31:0] cpu_perf_cnt_9,
    output [31:0] cpu_perf_cnt_10,
    output [31:0] cpu_perf_cnt_11,
    output [31:0] cpu_perf_cnt_12,
    output [31:0] cpu_perf_cnt_13,
    output [31:0] cpu_perf_cnt_14,
    output [31:0] cpu_perf_cnt_15,
    output [69:0] inst_retire
);

// ====================== 全局控制信号定义 ======================
wire stall_if_id;    
wire stall_id_ex;    
wire stall_ex_mem;   
wire stall_mem_wb;

wire flush_if_id;    
wire flush_id_ex;    
wire flush_ex_mem;
wire flush_mem_wb;

wire mem_wait_stall;
wire data_hazard;        
wire branch_flush;       

// ====================== 1. IF级：取指阶段 ======================
reg [31:0] pc_reg;
reg        req_accepted;

// ================== AXI地址强保护与幽灵请求追踪 ==================
wire req_fire = Inst_Req_Valid && Inst_Req_Ready; // 请求握手成功
wire resp_fire = Inst_Valid && Inst_Ready;        // 应答握手成功

reg redirect_pending;
reg [31:0] redirect_target;
always @(posedge clk) begin
    if (rst) begin
        redirect_pending <= 1'b0;
        redirect_target <= 32'h0;
    end else if (branch_flush) begin
        if (Inst_Req_Valid && !Inst_Req_Ready) begin // 发生跳转分支，但是上一个请求已经发出且请求未握手，总先忙
            redirect_pending <= 1'b1; // 记录有个重定向还未进行
            redirect_target <= ex_branch_target; // 锁存目标地址
        end
    end else if (redirect_pending && Inst_Req_Ready) begin
        redirect_pending <= 1'b0; // 第一次握手成功，总线空闲，若有重定向就执行
    end
end

reg [2:0] outstanding_req; // 空中请求数
reg [2:0] discard_cnt; // 需要丢弃的应答数
always @(posedge clk) begin
    if (rst) begin
        outstanding_req <= 3'd0;
        discard_cnt <= 3'd0;
    end else begin
        outstanding_req <= outstanding_req + {2'b0, req_fire} - {2'b0, resp_fire};
        // 请求握手成功表示空中请求数加一，应答握手成功表示空中请求数减一，简单的进出计数
        if (branch_flush) begin
            // 分支发生瞬间拍一个快照，此刻正在空中的请求要扔掉
            discard_cnt <= outstanding_req + {2'b0, req_fire} - {2'b0, resp_fire};
        end else if (redirect_pending && req_fire) begin
            // 重定向期间，又有新请求，这个请求也要作废，计入丢弃数，同时返回的数据作丢弃
            discard_cnt <= discard_cnt + 3'd1 - {2'b0, resp_fire};
        end else if (resp_fire && discard_cnt > 0) begin
            // 应答握手成功消耗丢弃数
            discard_cnt <= discard_cnt - 3'd1;
        end
    end
end

// 判断指令是否是垃圾，为1则为垃圾指令
wire stale_inst = (discard_cnt > 0) || (branch_flush && Inst_Valid) || (redirect_pending && Inst_Valid);
// 真正有效的指令
wire real_inst_valid = Inst_Valid && !stale_inst;

// ================== 核心修复：取指缓存区(打破总线死锁) ==================
reg [31:0] inst_buf; // 指令缓存，暂存一条指令
reg        inst_buf_valid; // 缓存里是否有数据

// 垃圾指令直接收反正要扔掉，缓存空着或者不停顿也直接收
assign Inst_Ready = stale_inst ? 1'b1 : (!inst_buf_valid || !stall_if_id);
// 取得了真正有效的指令才算握手成功
wire inst_handshake = real_inst_valid && Inst_Ready;

always @(posedge clk) begin
    if (rst || branch_flush) begin
        inst_buf_valid <= 1'b0;
        inst_buf <= 32'h0;
    end else if (inst_handshake && stall_if_id) begin
        // 流水线被卡住，但总线送来了数据，先暂存到缓存里释放总线
        inst_buf_valid <= 1'b1;
        inst_buf <= Instruction;
    end else if (!stall_if_id && inst_buf_valid) begin
        // 流水线恢复前进，消耗掉缓存
        inst_buf_valid <= 1'b0;
    end
end

// 请求保持逻辑
reg inst_req_holding;
always @(posedge clk) begin
    if (rst) 
        inst_req_holding <= 1'b0;
    else if (Inst_Req_Valid && Inst_Req_Ready) // 握手成功清除保持
        inst_req_holding <= 1'b0;
    else if (Inst_Req_Valid && !Inst_Req_Ready) // 尚未握手记住保持
        inst_req_holding <= 1'b1;
end

// 缓存满了或者请求已经受理时，不再发新请求
assign Inst_Req_Valid = inst_req_holding        // 上次请求没处理，继续举手
                        || (!rst
                            && !inst_buf_valid  // 缓存空闲
                            && !req_accepted    // 没有等应答
                            && !stall_if_id);   // 流水线没停顿

always @(posedge clk) begin
    if (rst || branch_flush) begin
        req_accepted <= 1'b0;
    end else if (Inst_Req_Valid && Inst_Req_Ready && !redirect_pending) begin
        req_accepted <= 1'b1; // 总线受理了，并且不是重定向期间，标记等待
    end else if (inst_handshake) begin
        req_accepted <= 1'b0;
    end
end

// PC 更新逻辑
always @(posedge clk) begin
    if (rst)
        pc_reg <= 32'h0;
    else if (branch_flush && !(Inst_Req_Valid && !Inst_Req_Ready))
        pc_reg <= ex_branch_target; // 没有卡在总线上的请求，直接跳
    else if (redirect_pending && Inst_Req_Ready)
        pc_reg <= redirect_target; // 总线刚好闲下来，用锁存的地址跳转
    else if (!stall_if_id && (inst_buf_valid || (inst_handshake && req_accepted)))
        pc_reg <= pc_reg + 32'd4; // 要么从缓存里取一条，要么正好从总线接受一条
end
assign PC = pc_reg;

// ====================== 2. IF/ID流水线寄存器 ======================
reg [31:0] ifid_pc;
reg [31:0] ifid_instruction;
reg        ifid_valid;

always @(posedge clk) begin
    if (rst || flush_if_id) begin
        ifid_valid <= 1'b0;
        ifid_pc <= 32'h0;
        ifid_instruction <= 32'h0;
    end else if (!stall_if_id) begin
        ifid_valid <= inst_buf_valid || (inst_handshake && req_accepted);
        ifid_pc <= pc_reg;
        // 如果缓存里有数据就用缓存，否则用总线当拍进来的数据
        ifid_instruction <= inst_buf_valid ? inst_buf : Instruction;
    end
end

// ====================== 3. ID级：译码阶段 (纯组合逻辑) ======================
wire [6:0]  opcode   = ifid_instruction[6:0];
wire [4:0]  rd       = ifid_instruction[11:7];
wire [2:0]  funct3   = ifid_instruction[14:12];
wire [6:0]  funct7   = ifid_instruction[31:25];
wire [4:0]  rs1      = ifid_instruction[19:15];
wire [4:0]  rs2      = ifid_instruction[24:20];
wire [11:0] imm_I    = ifid_instruction[31:20];
wire [11:0] imm_S    = {ifid_instruction[31:25], ifid_instruction[11:7]};
wire [12:0] imm_B    = {ifid_instruction[31], ifid_instruction[7], ifid_instruction[30:25], ifid_instruction[11:8], 1'b0};
wire [19:0] imm_U    = ifid_instruction[31:12];
wire [20:0] imm_J    = {ifid_instruction[31], ifid_instruction[19:12], ifid_instruction[20], ifid_instruction[30:21], 1'b0};
wire [31:0] imm_I_sext = {{20{imm_I[11]}}, imm_I};
wire [31:0] imm_S_sext = {{20{imm_S[11]}}, imm_S};
wire [31:0] imm_B_sext = {{19{imm_B[12]}}, imm_B};
wire [31:0] imm_U_sext = {imm_U, 12'd0};
wire [31:0] imm_J_sext = {{11{imm_J[20]}}, imm_J};

wire is_R_type = (opcode == 7'b0110011);
wire is_OP_IMM = (opcode == 7'b0010011);
wire is_LOAD   = (opcode == 7'b0000011);
wire is_I_type = is_OP_IMM || is_LOAD;
wire is_S_type = (opcode == 7'b0100011);
wire is_B_type = (opcode == 7'b1100011);
wire is_U_type = (opcode == 7'b0110111) || (opcode == 7'b0010111);
wire is_J_type = (opcode == 7'b1101111) || (opcode == 7'b1100111);
wire is_JAL    = (opcode == 7'b1101111);
wire is_JALR   = (opcode == 7'b1100111);
wire is_LUI    = (opcode == 7'b0110111);
wire is_AUIPC  = (opcode == 7'b0010111);
wire RegWrite = is_R_type || is_I_type || is_U_type || is_J_type;
wire ALUSrc = is_OP_IMM || is_LOAD || is_S_type || is_JALR || is_U_type;
wire MemtoReg = is_LOAD;
wire Branch   = is_B_type;
wire Jump     = is_J_type;

localparam F3_ADD_SUB = 3'b000;
localparam F3_SLL     = 3'b001;
localparam F3_SLT     = 3'b010;
localparam F3_SLTU    = 3'b011;
localparam F3_XOR     = 3'b100;
localparam F3_SRL_SRA = 3'b101;
localparam F3_OR      = 3'b110;
localparam F3_AND     = 3'b111;
localparam F7_NORMAL  = 7'b0000000;
localparam F7_SUB_SRA = 7'b0100000;
localparam F7_MUL     = 7'b0000001;   // 新增
localparam ALU_ADD    = 4'b0010;
localparam ALU_SUB    = 4'b0110;
localparam ALU_SLT    = 4'b0111;
localparam ALU_SLTU   = 4'b0011;
localparam ALU_XOR    = 4'b0100;
localparam ALU_OR     = 4'b0001;
localparam ALU_AND    = 4'b0000;
localparam ALU_MUL    = 4'b1000;   // 新增

wire [3:0] alu_op = 
    is_R_type ?
(
        (funct3 == F3_ADD_SUB) ?             // 仅判断 funct3，再细分 funct7
            ((funct7 == F7_SUB_SRA) ? ALU_SUB :
            (funct7 == F7_MUL)     ? ALU_MUL :
            ALU_ADD) :                        // 默认 ADD（含 F7_NORMAL）
        (funct3 == F3_SLT)     ? ALU_SLT :
        (funct3 == F3_SLTU)    ? ALU_SLTU :
        (funct3 == F3_XOR)     ? ALU_XOR :
        (funct3 == F3_OR)      ? ALU_OR :
        (funct3 == F3_AND)     ? ALU_AND :
        ALU_ADD
    ) : is_I_type ?
(
        is_OP_IMM ? (
            (funct3 == F3_ADD_SUB) ? ALU_ADD :
            (funct3 == F3_SLT)     ? ALU_SLT :
            (funct3 == F3_SLTU)    ? ALU_SLTU :
            (funct3 == F3_XOR)     ? ALU_XOR :
            (funct3 == F3_OR)      ? ALU_OR :
            (funct3 == F3_AND)     ? ALU_AND :
            ALU_ADD
        ) : ALU_ADD
    ) : is_S_type ? ALU_ADD :
      is_B_type ? ALU_SUB :
      is_U_type ? ALU_ADD :
      is_J_type ? ALU_ADD : ALU_ADD;

wire is_SLL  = is_R_type && (funct3 == F3_SLL)     && (funct7 == F7_NORMAL);
wire is_SRL  = is_R_type && (funct3 == F3_SRL_SRA) && (funct7 == F7_NORMAL);
wire is_SRA  = is_R_type && (funct3 == F3_SRL_SRA) && (funct7 == F7_SUB_SRA);
wire is_SLLI = is_OP_IMM && (funct3 == F3_SLL)     && (funct7 == F7_NORMAL);
wire is_SRLI = is_OP_IMM && (funct3 == F3_SRL_SRA) && (funct7 == F7_NORMAL);
wire is_SRAI = is_OP_IMM && (funct3 == F3_SRL_SRA) && (funct7 == F7_SUB_SRA);
wire is_shift_inst = is_SLL || is_SRL || is_SRA || is_SLLI || is_SRLI || is_SRAI;

localparam SHIFT_SLL = 2'b00;
localparam SHIFT_SRL = 2'b10;
localparam SHIFT_SRA = 2'b11;
wire [1:0] shift_op = (is_SLL || is_SLLI) ? SHIFT_SLL :
                      (is_SRL || is_SRLI) ? SHIFT_SRL :
                      (is_SRA || is_SRAI) ? SHIFT_SRA : SHIFT_SLL;

wire        RF_wen;
wire [4:0]  RF_waddr;
wire [31:0] RF_wdata;
wire [31:0] read_data_1, read_data_2;

reg_file rf_inst (
    .clk    (clk),
    .waddr  (RF_waddr),
    .raddr1 (rs1),
    .raddr2 (rs2),
    .wen    (RF_wen),
    .wdata  (RF_wdata),
    .rdata1 (read_data_1),
    .rdata2 (read_data_2)
);

// ID级输出 (纯组合逻辑 wire)
wire        id_valid     = ifid_valid;
wire        id_RegWrite  = RegWrite && ifid_valid;
wire        id_MemRead   = is_LOAD && ifid_valid;
wire        id_MemWrite  = is_S_type && ifid_valid;
wire        id_MemtoReg  = MemtoReg && ifid_valid;
wire        id_ALUSrc    = ALUSrc;
wire [3:0]  id_alu_op    = alu_op;
wire [1:0]  id_shift_op  = shift_op;
wire        id_is_shift  = is_shift_inst;
wire        id_is_R_type = is_R_type;
wire [2:0]  id_funct3    = funct3;
wire [31:0] id_read_data1 = read_data_1;
wire [31:0] id_read_data2 = read_data_2;
wire [4:0]  id_rd        = rd;
wire [4:0]  id_rs1       = rs1;
wire [4:0]  id_rs2       = rs2;
wire [31:0] id_pc_out    = ifid_pc;
wire        id_is_JAL    = is_JAL;
wire        id_is_JALR   = is_JALR;
wire        id_is_LUI    = is_LUI;
wire        id_is_AUIPC  = is_AUIPC;
wire        id_Branch    = Branch;
wire        id_Jump       = Jump;
wire        id_is_J_type = is_J_type; 

wire [31:0] id_imm_sext = is_I_type ? imm_I_sext :
                          is_S_type ? imm_S_sext :
                          is_B_type ? imm_B_sext :
                          is_U_type ? imm_U_sext :
                          is_J_type ? (is_JAL ? imm_J_sext : imm_I_sext) : 32'h0;

// ====================== 4. ID/EX流水线寄存器======================
reg        idex_RegWrite;
reg        idex_MemRead;
reg        idex_MemWrite;
reg        idex_MemtoReg;
reg        idex_ALUSrc;
reg [3:0]  idex_alu_op;
reg [1:0]  idex_shift_op;
reg        idex_is_shift;
reg        idex_is_R_type;
reg [2:0]  idex_funct3;
reg [31:0] idex_read_data1;
reg [31:0] idex_read_data2;
reg [31:0] idex_imm_sext;
reg [4:0]  idex_rd;
reg [4:0]  idex_rs1;
reg [4:0]  idex_rs2;
reg [31:0] idex_pc;
reg        idex_is_JAL;
reg        idex_is_JALR;
reg        idex_is_LUI;
reg        idex_is_AUIPC;
reg        idex_Branch;
reg        idex_Jump;
reg        idex_valid;
reg        idex_is_J_type;

always @(posedge clk) begin
    if (rst || flush_id_ex) begin  
        idex_valid <= 1'b0;
        idex_RegWrite <= 1'b0;
        idex_MemRead <= 1'b0;
        idex_MemWrite <= 1'b0;
        idex_MemtoReg <= 1'b0;
        idex_ALUSrc <= 1'b0;
        idex_alu_op <= 4'b0;
        idex_shift_op <= 2'b0;
        idex_is_shift <= 1'b0;
        idex_is_R_type <= 1'b0;
        idex_funct3 <= 3'b0;
        idex_read_data1 <= 32'h0;
        idex_read_data2 <= 32'h0;
        idex_imm_sext <= 32'h0;
        idex_rd <= 5'b0;
        idex_rs1 <= 5'b0;
        idex_rs2 <= 5'b0;
        idex_pc <= 32'h0;
        idex_is_JAL <= 1'b0;
        idex_is_JALR <= 1'b0;
        idex_is_LUI <= 1'b0;
        idex_is_AUIPC <= 1'b0;
        idex_Branch <= 1'b0;
        idex_Jump <= 1'b0;
        idex_is_J_type <= 1'b0;
    end else if (data_hazard) begin 
        idex_valid <= 1'b0;
        idex_RegWrite <= 1'b0;
        idex_MemRead <= 1'b0;
        idex_MemWrite <= 1'b0;
        idex_MemtoReg <= 1'b0;
        idex_ALUSrc <= 1'b0;
        idex_alu_op <= 4'b0;
        idex_shift_op <= 2'b0;
        idex_is_shift <= 1'b0;
        idex_is_R_type <= 1'b0;
        idex_funct3 <= 3'b0;
        idex_read_data1 <= 32'h0;
        idex_read_data2 <= 32'h0;
        idex_imm_sext <= 32'h0;
        idex_rd <= 5'b0;
        idex_rs1 <= 5'b0;
        idex_rs2 <= 5'b0;
        idex_pc <= 32'h0;
        idex_is_JAL <= 1'b0;
        idex_is_JALR <= 1'b0;
        idex_is_LUI <= 1'b0;
        idex_is_AUIPC <= 1'b0;
        idex_Branch <= 1'b0;
        idex_Jump <= 1'b0;
        idex_is_J_type <= 1'b0;
    end else if (!stall_id_ex) begin  
        idex_valid <= id_valid;
        idex_RegWrite <= id_RegWrite;
        idex_MemRead <= id_MemRead;
        idex_MemWrite <= id_MemWrite;
        idex_MemtoReg <= id_MemtoReg;
        idex_ALUSrc <= id_ALUSrc;
        idex_alu_op <= id_alu_op;
        idex_shift_op <= id_shift_op;
        idex_is_shift <= id_is_shift;
        idex_is_R_type <= id_is_R_type;
        idex_funct3 <= id_funct3;
        idex_read_data1 <= id_read_data1;
        idex_read_data2 <= id_read_data2;
        idex_imm_sext <= id_imm_sext;
        idex_rd <= id_rd;
        idex_rs1 <= id_rs1;
        idex_rs2 <= id_rs2;
        idex_pc <= id_pc_out;
        idex_is_JAL <= id_is_JAL;
        idex_is_JALR <= id_is_JALR;
        idex_is_LUI <= id_is_LUI;
        idex_is_AUIPC <= id_is_AUIPC;
        idex_Branch <= id_Branch;
        idex_Jump <= id_Jump;
        idex_is_J_type <= id_is_J_type;
    end
end

// ====================== 5. EX级：执行阶段 (组合逻辑) ======================
wire [31:0] alu_input_1 = (idex_is_AUIPC || idex_is_JAL) ? idex_pc : 
                          (idex_is_LUI ? 32'd0 : idex_read_data1);
wire [31:0] alu_input_2_normal = idex_is_JAL ? 32'd4 :
                                 (idex_ALUSrc ? idex_imm_sext : idex_read_data2);
wire [31:0] shift_input = idex_read_data1;

wire [4:0] shift_amount = idex_is_shift ? 
    (idex_is_R_type ? idex_read_data2[4:0] : idex_imm_sext[4:0]) : 5'b0;
wire [31:0] alu_result_normal;
wire        zero_flag_alu, overflow_flag, carryout_flag;
alu alu_inst (
    .A(alu_input_1), 
    .B(alu_input_2_normal), 
    .ALUop(idex_alu_op),
    .Overflow(overflow_flag), 
    .CarryOut(carryout_flag), 
    .Zero(zero_flag_alu), 
    .Result(alu_result_normal)
);
wire [31:0] shift_result;
shifter shift_inst (
    .A(shift_input), 
    .B(shift_amount), 
    .Shiftop(idex_shift_op), 
    .Result(shift_result)
);
wire [31:0] alu_result = idex_is_shift ? shift_result : alu_result_normal;
wire zero_flag = idex_is_shift ? (shift_result == 32'd0) : zero_flag_alu;
wire [31:0] return_addr = idex_pc + 32'd4;
wire [31:0] alu_result_for_wb = (idex_Jump) ? return_addr : alu_result;

wire branch_eq  = zero_flag_alu;
wire branch_ne  = ~zero_flag_alu;
wire branch_lt_signed = alu_result_normal[31] ^ overflow_flag;
wire branch_ge_signed = ~branch_lt_signed;
wire branch_lt_unsigned = carryout_flag;
wire branch_ge_unsigned = ~carryout_flag;

wire branch_taken = idex_Branch && (
    (idex_funct3 == 3'b000 && branch_eq) || 
    (idex_funct3 == 3'b001 && branch_ne) ||
    (idex_funct3 == 3'b100 && branch_lt_signed) || 
    (idex_funct3 == 3'b101 && branch_ge_signed) ||
    (idex_funct3 == 3'b110 && branch_lt_unsigned) || 
    (idex_funct3 == 3'b111 && branch_ge_unsigned));

wire [31:0] ex_branch_target = 
    (idex_Jump && idex_is_JAL)  ? (idex_pc + idex_imm_sext) :    
    (idex_Jump && idex_is_JALR) ? {alu_result[31:1], 1'b0} : 
    (idex_Branch && branch_taken) ? (idex_pc + idex_imm_sext) :    
    (idex_pc + 32'd4);

// 分支与冲刷控制信号
assign branch_flush = (idex_Jump || (idex_Branch && branch_taken)) && idex_valid;
assign flush_if_id = branch_flush;
assign flush_id_ex = branch_flush;
assign flush_ex_mem = 1'b0;  
assign flush_mem_wb = 1'b0;

wire [1:0] mem_addr_low = alu_result[1:0];
wire [4:0] mem_shift = {mem_addr_low, 3'b000};

wire is_SB = idex_MemWrite && (idex_funct3 == 3'b000);
wire is_SH = idex_MemWrite && (idex_funct3 == 3'b001);
wire is_SW = idex_MemWrite && (idex_funct3 == 3'b010);
wire [31:0] write_data_comb = 
    is_SB ? ({24'b0, idex_read_data2[7:0]}  << mem_shift) :
    is_SH ? ({16'b0, idex_read_data2[15:0]} << mem_shift) : 
    is_SW ? idex_read_data2 : 32'd0;

wire [3:0] write_strb_comb = 
    is_SB ? (4'b0001 << mem_addr_low) : 
    is_SH ? (4'b0011 << mem_addr_low) : 
    is_SW ? 4'b1111 : 4'b0000;

// ====================== 6. EX/MEM流水线寄存器======================
reg        exmem_RegWrite;
reg        exmem_MemRead;
reg        exmem_MemWrite;
reg        exmem_MemtoReg;
reg [2:0]  exmem_funct3;
reg [31:0] exmem_alu_result;
reg [31:0] exmem_write_data;
reg [3:0]  exmem_write_strb;
reg [4:0]  exmem_rd;
reg [31:0] exmem_pc;
reg        exmem_valid;

always @(posedge clk) begin
    if (rst || flush_ex_mem) begin
        exmem_valid <= 1'b0;
        exmem_RegWrite <= 1'b0;
        exmem_MemRead <= 1'b0;
        exmem_MemWrite <= 1'b0;
        exmem_MemtoReg <= 1'b0;
        exmem_funct3 <= 3'b0;
        exmem_alu_result <= 32'h0;
        exmem_write_data <= 32'h0;
        exmem_write_strb <= 4'b0;
        exmem_rd <= 5'b0;
        exmem_pc <= 32'h0;
    end else if (!stall_ex_mem) begin
        exmem_valid <= idex_valid;             
        exmem_RegWrite <= idex_RegWrite;
        exmem_MemRead <= idex_MemRead;
        exmem_MemWrite <= idex_MemWrite;
        exmem_MemtoReg <= idex_MemtoReg;
        exmem_funct3 <= idex_funct3;
        exmem_alu_result <= alu_result_for_wb;      
        exmem_write_data <= write_data_comb;
        exmem_write_strb <= write_strb_comb;
        exmem_rd <= idex_rd;
        exmem_pc <= idex_pc;
    end
end

// ====================== 7. MEM级：访存阶段 (组合逻辑) ======================
reg mem_req_sent; // 防止对同一次访存请求重复发送
always @(posedge clk) begin
    if (rst) begin
        mem_req_sent <= 1'b0;
    end else if (!stall_ex_mem) begin
        mem_req_sent <= 1'b0; 
    end else if (exmem_valid && (exmem_MemRead || exmem_MemWrite) && Mem_Req_Ready) begin
        mem_req_sent <= 1'b1; // 请求被总线受理
    end
end

assign Address = {exmem_alu_result[31:2], 2'b00};
assign MemRead = exmem_MemRead && exmem_valid && !mem_req_sent;
assign MemWrite = exmem_MemWrite && exmem_valid && !mem_req_sent;
assign Write_data = exmem_write_data;
assign Write_strb = exmem_write_strb;

reg read_ready_reg;
always @(posedge clk) begin
    if (rst || (MemRead && Mem_Req_Ready ))
        read_ready_reg <= 1'b1;
    else if (Read_data_Valid && read_ready_reg)
        read_ready_reg <= 1'b0;
end
assign Read_data_Ready = read_ready_reg;

wire [1:0] load_addr_low = exmem_alu_result[1:0];
wire [7:0] load_byte = 
    (load_addr_low == 2'b00) ? Read_data[7:0] : 
    (load_addr_low == 2'b01) ? Read_data[15:8] :
    (load_addr_low == 2'b10) ? Read_data[23:16] : Read_data[31:24];

wire [15:0] load_half = load_addr_low[1] ? Read_data[31:16] : Read_data[15:0];
wire [31:0] mem_load_data_comb =
    (exmem_funct3 == 3'b000) ? {{24{load_byte[7]}}, load_byte} : 
    (exmem_funct3 == 3'b001) ? {{16{load_half[15]}}, load_half} : 
    (exmem_funct3 == 3'b010) ? Read_data :                        
    (exmem_funct3 == 3'b100) ? {24'b0, load_byte} :               
    (exmem_funct3 == 3'b101) ? {16'b0, load_half} : Read_data;    

// 核心冒险检测逻辑和停顿逻辑
assign mem_wait_stall = (exmem_MemRead && exmem_valid && !Read_data_Valid) || 
                        // 请求还没发出去，总线现在忙 → 应该等，请求已经发出去了，总线现在忙别的事 → 不应该等
                        (exmem_MemWrite && exmem_valid && !Mem_Req_Ready && !mem_req_sent);

wire hazard_id_ex = ifid_valid && idex_valid && idex_RegWrite && (idex_rd != 5'b0) &&
                ((rs1 == idex_rd) || (rs2 == idex_rd));
wire hazard_ex_mem = ifid_valid && exmem_valid && exmem_RegWrite && (exmem_rd != 5'b0) &&
                    ((rs1 == exmem_rd) || (rs2 == exmem_rd));
wire hazard_mem_wb = ifid_valid && memwb_valid && memwb_RegWrite && (memwb_rd != 5'b0) &&
                    ((rs1 == memwb_rd) || (rs2 == memwb_rd));
assign data_hazard = hazard_id_ex || hazard_ex_mem || hazard_mem_wb;

// 数据冒险只有ifid停顿，因为要锁存当前指令，其他阶段插入空泡
assign stall_if_id = data_hazard || mem_wait_stall;
assign stall_id_ex = mem_wait_stall;
assign stall_ex_mem = mem_wait_stall;   
assign stall_mem_wb = mem_wait_stall; 

// ====================== 8. MEM/WB流水线寄存器======================
reg        memwb_RegWrite;
reg        memwb_MemtoReg;
reg [31:0] memwb_load_data;
reg [31:0] memwb_alu_result;
reg [4:0]  memwb_rd;
reg [31:0] memwb_pc;
reg        memwb_valid;

always @(posedge clk) begin
    if (rst || flush_mem_wb) begin
        memwb_valid <= 1'b0;
        memwb_RegWrite <= 1'b0;
        memwb_MemtoReg <= 1'b0;
        memwb_load_data <= 32'h0;
        memwb_alu_result <= 32'h0;
        memwb_rd <= 5'b0;
        memwb_pc <= 32'h0;
    end else if (!stall_mem_wb) begin 
        memwb_valid <= exmem_valid;
        memwb_RegWrite <= exmem_RegWrite;
        memwb_MemtoReg <= exmem_MemtoReg;
        memwb_load_data <= mem_load_data_comb; 
        memwb_alu_result <= exmem_alu_result;
        memwb_rd <= exmem_rd;
        memwb_pc <= exmem_pc;
    end
end

// ====================== 9. WB级：写回阶段 ======================
assign RF_wdata = memwb_MemtoReg ? memwb_load_data : memwb_alu_result;
assign RF_waddr = memwb_rd;
assign RF_wen = memwb_RegWrite && memwb_valid && (memwb_rd != 5'b0);

// ====================== 10. 性能计数器 ======================
reg [31:0] cycle_cnt;
reg [31:0] inst_retire_cnt;
always @(posedge clk) begin
    if (rst) begin
        cycle_cnt <= 32'h0;
        inst_retire_cnt <= 32'h0;
    end else begin
        cycle_cnt <= cycle_cnt + 1;
        if (RF_wen) begin
            inst_retire_cnt <= inst_retire_cnt + 1;
        end
    end
end

assign cpu_perf_cnt_0  = cycle_cnt;
assign cpu_perf_cnt_1  = inst_retire_cnt;
assign cpu_perf_cnt_2  = 32'h0;
assign cpu_perf_cnt_3  = 32'h0;
assign cpu_perf_cnt_4  = 32'h0;
assign cpu_perf_cnt_5  = 32'h0;
assign cpu_perf_cnt_6  = 32'h0;
assign cpu_perf_cnt_7  = 32'h0;
assign cpu_perf_cnt_8  = 32'h0;
assign cpu_perf_cnt_9  = 32'h0;
assign cpu_perf_cnt_10 = 32'h0;
assign cpu_perf_cnt_11 = 32'h0;
assign cpu_perf_cnt_12 = 32'h0;
assign cpu_perf_cnt_13 = 32'h0;
assign cpu_perf_cnt_14 = 32'h0;
assign cpu_perf_cnt_15 = 32'h0;

// ====================== 11. 指令退休信号 ======================
assign inst_retire = RF_wen ? 
    {memwb_RegWrite, memwb_rd, RF_wdata, memwb_pc} : 70'b0;

endmodule