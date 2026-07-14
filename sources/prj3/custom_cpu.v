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

// ====================== 状态机定义（9个状态，独热码） ======================
localparam S_INIT = 9'b000000001; // 初始状态
localparam S_IF   = 9'b000000010; // 取指
localparam S_IW   = 9'b000000100; // 指令等待
localparam S_ID   = 9'b000001000; // 译码
localparam S_EX   = 9'b000010000; // 执行
localparam S_LD   = 9'b000100000; // 内存读（Load请求）
localparam S_RDW  = 9'b001000000; // 读数据等待
localparam S_WB   = 9'b010000000; // 写回
localparam S_ST   = 9'b100000000; // 内存写（Store请求）

reg [8:0] current_state, next_state;

// ====================== 内部寄存器 ======================
reg [31:0] pc_reg;
reg [31:0] instruction_reg;

// 用于访存及写回的锁存信号
reg        is_load_reg;      // 当前指令是Load（用于区分访存类型及写回）
reg        is_store_reg;     // 当前指令是Store
reg [31:0] mem_addr_reg;     // 访存地址
reg [31:0] mem_wdata_reg;    // 写数据
reg [3:0]  mem_wstrb_reg;    // 写字节使能

reg        wb_en_reg;        // 写回使能
reg [4:0]  wb_addr_reg;      // 写回目标寄存器
reg [31:0] wb_data_reg;      // 写回数据

// 用于退休信息的锁存信号
reg [31:0] retire_pc_reg;

// 周期计数器
reg [31:0] cycle_cnt;

reg is_periph_load_reg;  // 标记当前 Load 指令是否是外设读

// ====================== 指令字段解析 ======================
wire [6:0]  opcode   = instruction_reg[6:0];
wire [4:0]  rd       = instruction_reg[11:7];
wire [2:0]  funct3   = instruction_reg[14:12];
wire [6:0]  funct7   = instruction_reg[31:25];
wire [4:0]  rs1      = instruction_reg[19:15];
wire [4:0]  rs2      = instruction_reg[24:20];
wire [11:0] imm_I    = instruction_reg[31:20];
wire [11:0] imm_S    = {instruction_reg[31:25], instruction_reg[11:7]};
wire [12:0] imm_B    = {instruction_reg[31], instruction_reg[7], instruction_reg[30:25], instruction_reg[11:8], 1'b0};
wire [19:0] imm_U    = instruction_reg[31:12];
wire [20:0] imm_J    = {instruction_reg[31], instruction_reg[19:12], instruction_reg[20], instruction_reg[30:21], 1'b0};

wire [31:0] imm_I_sext = {{20{imm_I[11]}}, imm_I};
wire [31:0] imm_S_sext = {{20{imm_S[11]}}, imm_S};
wire [31:0] imm_B_sext = {{19{imm_B[12]}}, imm_B};
wire [31:0] imm_U_sext = {imm_U, 12'd0};
wire [31:0] imm_J_sext = {{11{imm_J[20]}}, imm_J};

// ====================== 寄存器文件 ======================
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

// ====================== 控制信号生成（纯组合逻辑） ======================
wire is_R_type = (opcode == 7'b0110011);
wire is_OP_IMM = (opcode == 7'b0010011);
wire is_LOAD   = (opcode == 7'b0000011);
wire is_I_type = is_OP_IMM || is_LOAD;
wire is_S_type = (opcode == 7'b0100011);
wire is_B_type = (opcode == 7'b1100011);
wire is_U_type = (opcode == 7'b0110111) || (opcode == 7'b0010111);
wire is_J_type = (opcode == 7'b1101111) || (opcode == 7'b1100111);

wire is_JAL  = (opcode == 7'b1101111);
wire is_JALR = (opcode == 7'b1100111);
wire is_LUI  = (opcode == 7'b0110111);
wire is_AUIPC = (opcode == 7'b0010111);

wire RegWrite = is_R_type || is_I_type || is_U_type || is_J_type;
wire ALUSrc   = is_OP_IMM || is_LOAD || is_S_type || is_JALR;
wire MemtoReg = is_LOAD;
wire Branch   = is_B_type;
wire Jump     = is_J_type;

// ====================== ALU 与移位器控制 ======================
wire [2:0] alu_op;
wire [1:0] shift_op;
wire       is_shift_inst;

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

localparam ALU_ADD  = 3'b010;
localparam ALU_SUB  = 3'b110;
localparam ALU_SLT  = 3'b111;
localparam ALU_SLTU = 3'b011;
localparam ALU_XOR  = 3'b100;
localparam ALU_OR   = 3'b001;
localparam ALU_AND  = 3'b000;

assign alu_op = 
    is_R_type ? (
        (funct7 == F7_SUB_SRA && funct3 == F3_ADD_SUB) ? ALU_SUB :
        (funct3 == F3_ADD_SUB) ? ALU_ADD :
        (funct3 == F3_SLT)     ? ALU_SLT :
        (funct3 == F3_SLTU)    ? ALU_SLTU :
        (funct3 == F3_XOR)     ? ALU_XOR :
        (funct3 == F3_OR)      ? ALU_OR :
        (funct3 == F3_AND)     ? ALU_AND :
        ALU_ADD
    ) : is_I_type ? (
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
      is_J_type ? ALU_ADD :
      ALU_ADD;

wire is_SLL  = is_R_type && (funct3 == F3_SLL)     && (funct7 == F7_NORMAL);
wire is_SRL  = is_R_type && (funct3 == F3_SRL_SRA) && (funct7 == F7_NORMAL);
wire is_SRA  = is_R_type && (funct3 == F3_SRL_SRA) && (funct7 == F7_SUB_SRA);
wire is_SLLI = is_OP_IMM && (funct3 == F3_SLL)     && (funct7 == F7_NORMAL);
wire is_SRLI = is_OP_IMM && (funct3 == F3_SRL_SRA) && (funct7 == F7_NORMAL);
wire is_SRAI = is_OP_IMM && (funct3 == F3_SRL_SRA) && (funct7 == F7_SUB_SRA);

assign is_shift_inst = is_SLL || is_SRL || is_SRA || is_SLLI || is_SRLI || is_SRAI;

localparam SHIFT_SLL = 2'b00;
localparam SHIFT_SRL = 2'b10;
localparam SHIFT_SRA = 2'b11;

assign shift_op = (is_SLL || is_SLLI) ? SHIFT_SLL :
                  (is_SRL || is_SRLI) ? SHIFT_SRL :
                  (is_SRA || is_SRAI) ? SHIFT_SRA :
                  SHIFT_SLL;

// ====================== ALU 输入选择 ======================
wire [31:0] alu_input_1 = is_AUIPC ? pc_reg : read_data_1;
wire [31:0] alu_input_2_normal = ALUSrc ? (
    is_I_type ? imm_I_sext :
    is_S_type ? imm_S_sext :
    is_JALR   ? imm_I_sext :
    read_data_2
) : read_data_2;

wire [31:0] shift_input = read_data_1;
wire [4:0]  shift_amount = is_R_type ? read_data_2[4:0] : imm_I[4:0];

// ====================== ALU 和移位器实例化 ======================
wire [31:0] alu_result_normal;
wire        zero_flag_alu, overflow_flag, carryout_flag;
alu alu_inst (
    .A(alu_input_1), 
    .B(alu_input_2_normal), 
    .ALUop(alu_op),
    .Overflow(overflow_flag), 
    .CarryOut(carryout_flag), 
    .Zero(zero_flag_alu), 
    .Result(alu_result_normal)
);

wire [31:0] shift_result;
shifter shift_inst (
    .A(shift_input), 
    .B(shift_amount), 
    .Shiftop(shift_op), 
    .Result(shift_result)
);

wire [31:0] alu_result = is_shift_inst ? shift_result : alu_result_normal;
wire zero_flag = is_shift_inst ? (shift_result == 32'd0) : zero_flag_alu;

// ====================== 写回数据生成（组合逻辑，但仅在EX时锁存） ======================
wire [1:0] mem_addr_low = alu_result[1:0];
wire [7:0] load_byte = 
    (mem_addr_low == 2'b00) ? Read_data[7:0] : 
    (mem_addr_low == 2'b01) ? Read_data[15:8] :
    (mem_addr_low == 2'b10) ? Read_data[23:16] : 
    Read_data[31:24];
wire [15:0] load_half = mem_addr_low[1] ? Read_data[31:16] : Read_data[15:0];

wire [31:0] load_data = 
    (funct3 == 3'b000) ? {{24{load_byte[7]}}, load_byte} :  // LB
    (funct3 == 3'b001) ? {{16{load_half[15]}}, load_half} : // LH
    (funct3 == 3'b010) ? Read_data :                         // LW
    (funct3 == 3'b100) ? {24'b0, load_byte} :                // LBU
    (funct3 == 3'b101) ? {16'b0, load_half} : Read_data;     // LHU

wire [31:0] RF_wdata_comb = 
    MemtoReg ? load_data :              // Load → 从内存读出的数据
    is_LUI   ? imm_U_sext :             // LUI → 立即数 << 12
    is_AUIPC ? (pc_reg + imm_U_sext) :  // AUIPC → PC + 立即数
    Jump     ? (pc_reg + 32'd4) :       // JAL/JALR → 返回地址 PC+4
    alu_result;                         // 其他 → ALU/移位器结果

// ====================== 访存信号生成 ======================
wire is_SB = is_S_type && (funct3 == 3'b000);
wire is_SH = is_S_type && (funct3 == 3'b001);
wire is_SW = is_S_type && (funct3 == 3'b010);
wire [4:0] mem_shift = {mem_addr_low, 3'b000};

wire [31:0] write_data_comb = 
    is_SB ? ({24'b0, read_data_2[7:0]}  << mem_shift) :
    is_SH ? ({16'b0, read_data_2[15:0]} << mem_shift) : 
    is_SW ? read_data_2 : 
    32'd0;
wire [3:0] write_strb_comb = 
    is_SB ? (4'b0001 << mem_addr_low) : 
    is_SH ? (4'b0011 << mem_addr_low) : 
    is_SW ? 4'b1111 : 
    4'b0000;

// ====================== 分支与 PC 计算 ======================
wire is_memory_access = is_S_type || is_LOAD; // 判断是否访问数据内存（读或写）
wire branch_eq  = zero_flag_alu;
wire branch_ne  = ~zero_flag_alu;
wire branch_lt_signed = alu_result_normal[31] ^ overflow_flag;
wire branch_ge_signed = ~branch_lt_signed;
wire branch_lt_unsigned = carryout_flag;
wire branch_ge_unsigned = ~carryout_flag;

wire branch_taken = Branch && (
    (funct3 == 3'b000 && branch_eq) || 
    (funct3 == 3'b001 && branch_ne) ||
    (funct3 == 3'b100 && branch_lt_signed) || 
    (funct3 == 3'b101 && branch_ge_signed) ||
    (funct3 == 3'b110 && branch_lt_unsigned) || 
    (funct3 == 3'b111 && branch_ge_unsigned));

wire [31:0] pc_next = 
    (Jump && is_JAL)                ? (pc_reg + imm_J_sext) :    
    (Jump && is_JALR)               ? {alu_result[31:1], 1'b0} : 
    (Branch && branch_taken)        ? (pc_reg + imm_B_sext) :    
    (pc_reg + 32'd4);          

// ====================== 状态机第一段：状态跳转 ======================
always @(posedge clk) begin
    if (rst)
        current_state <= S_INIT;
    else
        current_state <= next_state;
end

// ====================== 状态机第二段：次态计算 ======================
always @(*) begin
    next_state = current_state;
    case (current_state)
        S_INIT: begin
            next_state = S_IF;                // 复位释放后第一个周期进入取指
        end
        S_IF: begin
            if (Inst_Req_Ready)               // Inst_Req_Valid 已拉高，只需等 Ready
                next_state = S_IW;
        end
        S_IW: begin
            if (Inst_Valid && Inst_Ready)
                next_state = S_ID;
        end
        S_ID: begin
            next_state = S_EX;                // 无条件进入执行（译码已隐含在组合逻辑中）
        end
        S_EX: begin
            if (is_S_type)                    // Store 指令
                next_state = S_ST;
            else if (is_LOAD)                 // Load 指令
                next_state = S_LD;
            else if (is_B_type)               // 分支指令，直接返回取指
                next_state = S_IF;
            else                              // R/I/U/J 型运算或跳转，进入写回
                next_state = S_WB;
        end
        S_LD: begin
            if (Mem_Req_Ready)
                next_state = S_RDW;
        end
        S_RDW: begin
            if (Read_data_Valid && Read_data_Ready)
                next_state = S_WB;
        end
        S_WB: begin                           // 写回完成，返回取指
            next_state = S_IF;
        end
        S_ST: begin
            if (Mem_Req_Ready)
                next_state = S_IF;
        end
        default: next_state = S_INIT;
    endcase
end

// ====================== 状态机第三段：输出逻辑与时序控制 ======================
// PC 更新
always @(posedge clk) begin
    if (rst)
        pc_reg <= 32'h0;
    else if ((current_state == S_EX && next_state == S_IF) ||   // 分支 / 部分跳转
             (current_state == S_ST && next_state == S_IF) ||   // Store 完成
             (current_state == S_WB && next_state == S_IF))     // 写回完成（含Load及算术指令）
        pc_reg <= pc_next;
end

// 指令锁存
always @(posedge clk) begin
    if (current_state == S_IW && Inst_Valid && Inst_Ready)
        instruction_reg <= Instruction;
end

// 执行阶段信息锁存（只锁存访存相关信号，写回寄存器移到独立 always 块）
always @(posedge clk) begin
    if (current_state == S_EX) begin
        // 访存相关信息锁存
        mem_addr_reg  <= {alu_result[31:2], 2'b00};
        mem_wdata_reg <= write_data_comb;
        mem_wstrb_reg <= write_strb_comb;
        is_load_reg   <= is_LOAD;
        is_store_reg  <= is_S_type;

        // 退休 PC 锁存
        retire_pc_reg <= pc_reg;
    end
end

// ====================== 写回寄存器统一管理（消除多驱动） ======================
always @(posedge clk) begin
    if (rst) begin
        wb_en_reg   <= 1'b0;
        wb_addr_reg <= 5'd0;
        wb_data_reg <= 32'd0;
    end else begin
        if (current_state == S_EX) begin
            // 非访存且需要写回的指令（R/I/U/J 型）
            if (!is_S_type && !is_LOAD && RegWrite) begin
                wb_en_reg   <= 1'b1;
                wb_addr_reg <= rd;
                wb_data_reg <= RF_wdata_comb;
            end
            // Load 指令：先锁存目标寄存器地址，写使能留到数据返回时再置位
            else if (is_LOAD) begin
                wb_addr_reg <= rd;
                wb_en_reg   <= 1'b0;   // 保证不会意外写回
            end
            // Store / 分支 / 其他指令
            else begin
                wb_en_reg <= 1'b0;
            end
        end else if (current_state == S_RDW && Read_data_Valid && Read_data_Ready) begin
            // Load 数据返回，锁存数据并使能写回
            wb_en_reg   <= 1'b1;
            wb_data_reg <= load_data;
        end else if (current_state == S_WB) begin
            // 写回完成后清除写使能
            wb_en_reg <= 1'b0;
        end
    end
end


// 寄存器写端口连接
assign RF_wen   = wb_en_reg && (current_state == S_WB);
assign RF_waddr = wb_addr_reg;
assign RF_wdata = wb_data_reg;

// 顶层握手信号
assign PC              = pc_reg;
assign Inst_Req_Valid  = (current_state == S_IF) && !(current_state == S_INIT);    // 取指时有效
assign Inst_Ready      = (current_state == S_IW) || (current_state == S_INIT);
assign MemRead         = (current_state == S_LD);    // 内存读请求
assign MemWrite        = (current_state == S_ST);    // 内存写请求
assign Address         = (current_state == S_LD || 
                          current_state == S_ST || 
                          current_state == S_RDW) ? mem_addr_reg : 32'd0;
assign Write_data      = (current_state == S_ST) ? mem_wdata_reg : 32'd0;
assign Write_strb      = (current_state == S_ST) ? mem_wstrb_reg : 4'b0;
assign Read_data_Ready = (current_state == S_RDW) ||          // 普通内存读等待
                         (current_state == S_INIT);                                 // 复位期间

// ====================== 性能计数器 ======================
always @(posedge clk) begin
    if (rst)
        cycle_cnt <= 32'h0;
    else
        cycle_cnt <= cycle_cnt + 1;
end

assign cpu_perf_cnt_0  = cycle_cnt;
assign cpu_perf_cnt_1  = 32'h0;
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

// ====================== 指令退休信号 ======================
wire retire_now = (current_state == S_EX && next_state == S_IF) ||
                  (current_state == S_WB && next_state == S_IF);
wire retire_wen = (current_state == S_EX && RegWrite && !is_memory_access) ||
                  (current_state == S_WB && wb_en_reg);
wire [4:0] retire_waddr = (current_state == S_WB) ? wb_addr_reg : rd;
wire [31:0] retire_wdata = (current_state == S_WB) ? wb_data_reg : RF_wdata_comb;
assign inst_retire = retire_now ? {retire_wen, retire_waddr, retire_wdata, retire_pc_reg} : 70'b0;

// ====================== 中断（暂不使用） ======================
// intr 端口直接忽略，实验5才会用到

endmodule