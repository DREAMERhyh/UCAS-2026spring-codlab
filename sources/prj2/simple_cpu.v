`timescale 10 ns / 1 ns

module simple_cpu(
    input             clk,
    input             rst,
    
    output [31:0]     PC,
    input  [31:0]     Instruction,
    
    output [31:0]     Address,
    output            MemWrite,
    output [31:0]     Write_data,
    output [ 3:0]     Write_strb,
    
    input  [31:0]     Read_data,
    output            MemRead
);

// ====================== 1. 指令字段解析 (RISC-V) ======================
wire [6:0]  opcode   = Instruction[6:0];    // 7位操作码
wire [4:0]  rd       = Instruction[11:7];   // 目标寄存器地址
wire [2:0]  funct3   = Instruction[14:12];  // 3位功能码
wire [6:0]  funct7   = Instruction[31:25];  // 7位功能码
wire [4:0]  rs1      = Instruction[19:15];  // 源寄存器1
wire [4:0]  rs2      = Instruction[24:20];  // 源寄存器2
wire [11:0] imm_I    = Instruction[31:20];  // I-type立即数
wire [11:0] imm_S    = {Instruction[31:25], Instruction[11:7]}; // S-type立即数
wire [12:0] imm_B    = {Instruction[31], Instruction[7], Instruction[30:25], Instruction[11:8], 1'b0};
wire [19:0] imm_U    = Instruction[31:12];  // U-type立即数
wire [20:0] imm_J    = {Instruction[31], Instruction[19:12], Instruction[20], Instruction[30:21], 1'b0};

// 立即数符号扩展
wire [31:0] imm_I_sext = {{20{imm_I[11]}}, imm_I};
wire [31:0] imm_S_sext = {{20{imm_S[11]}}, imm_S};
wire [31:0] imm_B_sext = {{19{imm_B[12]}}, imm_B};
wire [31:0] imm_U_sext = {imm_U, 12'd0};
wire [31:0] imm_J_sext = {{11{imm_J[20]}}, imm_J};

// ====================== 2. 寄存器文件信号定义 ======================
// THESE THREE SIGNALS ARE USED IN OUR TESTBENCH - DO NOT MODIFY
wire            RF_wen;
wire [4:0]      RF_waddr;
wire [31:0]     RF_wdata;

// 内部寄存器文件信号
wire [31:0]     read_data_1;
wire [31:0]     read_data_2;

// 实例化寄存器堆模块
reg_file rf_inst (
    .clk(clk),
    .waddr(RF_waddr),
    .raddr1(rs1),          // 使用已有的 rs1
    .raddr2(rs2),          // 使用已有的 rs2
    .wen(RF_wen),
    .wdata(RF_wdata),
    .rdata1(read_data_1),  // 驱动已有的 read_data_1
    .rdata2(read_data_2)   // 驱动已有的 read_data_2
);

// ====================== 3. 控制信号生成 (组合逻辑) ======================
wire is_R_type = (opcode == 7'b0110011);

wire is_OP_IMM = (opcode == 7'b0010011);
wire is_LOAD   = (opcode == 7'b0000011);
wire is_I_type = is_OP_IMM || is_LOAD; //jalr没有包含在I-type里，尽管是属于I-type格式的指令

wire is_S_type = (opcode == 7'b0100011);
wire is_B_type = (opcode == 7'b1100011);
wire is_U_type = (opcode == 7'b0110111) || (opcode == 7'b0010111);
wire is_J_type = (opcode == 7'b1101111) || (opcode == 7'b1100111); //将jalr也算作J-type

wire RegWrite;
wire ALUSrc;
wire MemtoReg;
wire Branch;
wire Jump;

// 主要控制信号
assign RegWrite = (is_R_type) ||  // R-type (except x0)
                  (is_I_type) ||  // I-type (except x0)  
                  (is_U_type) ||  // U-type (except x0)
                  (is_J_type); // JAL and JALR

assign ALUSrc =
    is_OP_IMM ||     // addi/slti/sltiu/xori/ori/andi, shift imm 由 shifter 用
    is_LOAD   ||     // load 地址 = rs1 + imm_I
    is_S_type ||     // store 地址 = rs1 + imm_S
    (opcode == 7'b1100111); // jalr 地址 = rs1 + imm_I

assign MemtoReg = (opcode == 7'b0000011); // lw指令
assign Branch = is_B_type;
assign Jump = is_J_type;

assign RF_wen   = RegWrite;  // 用内部逻辑驱动测试平台接口
assign RF_waddr = rd;        // 驱动写地址
// RF_wdata 已经在第7节有 assign 了

// ====================== 4. ALU和移位器控制信号 ======================
// ALU控制信号 (用于实例化的ALU模块)
wire [2:0] alu_op; // 3位ALU操作码

// 移位器控制信号 (用于实例化的移位器模块)
wire [1:0] shift_op; // 2位移位操作码
wire is_shift_inst;  // 是否为移位指令

// ALU操作码生成 (组合逻辑)
assign alu_op = is_R_type ? 
                  (funct7 == 7'b0100000 && funct3 == 3'b000) ? 3'b110 : // SUB
                  (funct3 == 3'b000) ? 3'b010 : // ADD
                  (funct3 == 3'b010) ? 3'b111 : // SLT
                  (funct3 == 3'b011) ? 3'b011 : // SLTU
                  (funct3 == 3'b100) ? 3'b100 : // XOR
                  (funct3 == 3'b110) ? 3'b001 : // OR
                  (funct3 == 3'b111) ? 3'b000 : // AND
                  3'b010 : // default: ADD
               is_I_type ? 
                  (opcode == 7'b0010011) ? // 立即数指令
                      (funct3 == 3'b000) ? 3'b010 : // ADDI
                      (funct3 == 3'b010) ? 3'b111 : // SLTI
                      (funct3 == 3'b011) ? 3'b011 : // SLTIU
                      (funct3 == 3'b100) ? 3'b100 : // XORI
                      (funct3 == 3'b110) ? 3'b001 : // ORI
                      (funct3 == 3'b111) ? 3'b000 : // ANDI
                      3'b010 :
                  3'b010 : // lw: ADD
               is_S_type ? 3'b010 : // sw: ADD (地址 = rs1 + imm)
               is_B_type ? 3'b110 : // beq/bne/blt/bge: SUB (rs1 - rs2)
               is_U_type ? 3'b010 : // LUI/AUIPC: ADD (AUIPC用, LUI忽略)
               is_J_type ? 3'b010 : // jal/jalr: ADD (jalr用, jal用PC+imm)
               3'b010; // default: ADD

wire is_OP     = (opcode == 7'b0110011); // R-type指令

wire is_SLL  = is_OP     && (funct3 == 3'b001) && (funct7 == 7'b0000000);
wire is_SRL  = is_OP     && (funct3 == 3'b101) && (funct7 == 7'b0000000);
wire is_SRA  = is_OP     && (funct3 == 3'b101) && (funct7 == 7'b0100000);

wire is_SLLI = is_OP_IMM && (funct3 == 3'b001) && (Instruction[31:25] == 7'b0000000);
wire is_SRLI = is_OP_IMM && (funct3 == 3'b101) && (Instruction[31:25] == 7'b0000000);
wire is_SRAI = is_OP_IMM && (funct3 == 3'b101) && (Instruction[31:25] == 7'b0100000); //其实就是funct7

// 移位指令判断和移位操作码
assign is_shift_inst = is_SLL || is_SRL || is_SRA || is_SLLI || is_SRLI || is_SRAI;

assign shift_op =
    (is_SLL || is_SLLI) ? 2'b00 :
    (is_SRL || is_SRLI) ? 2'b10 :
    (is_SRA || is_SRAI) ? 2'b11 :
                          2'b00;


// ====================== 5. ALU和移位器输入选择 ======================
// ALU第一个操作数
wire [31:0] alu_input_1 = (opcode == 7'b0010111) ? PC : // auipc
                          read_data_1;

// ALU第二个操作数 (非移位指令)
wire [31:0] alu_input_2_normal;

assign alu_input_2_normal = ALUSrc ? ( //ALUSrc控制是否使用立即数
    (is_I_type)                           ? imm_I_sext : // lw
    (is_S_type)                           ? imm_S_sext : // sw
    (is_J_type && (opcode == 7'b1100111)) ? imm_I_sext : // jalr
    read_data_2  // ALUSrc=1 但以上都不匹配时的默认值
) : read_data_2; // 这是 ALUSrc=0 时的分支

// 移位器输入
wire [31:0] shift_input = read_data_1;

wire [4:0]  shift_amount = is_R_type ? read_data_2[4:0] : // R-type: rs2低5位
                           imm_I[4:0]; // I-type: 立即数低5位

// ====================== 6. ALU和移位器实例化 ======================

// ALU模块实例化
wire [31:0] alu_result_normal;
wire        zero_flag_alu;
wire        overflow_flag;
wire        carryout_flag;

alu alu_inst (
    .A         (alu_input_1),
    .B         (alu_input_2_normal),
    .ALUop     (alu_op),
    .Overflow  (overflow_flag),
    .CarryOut  (carryout_flag),
    .Zero      (zero_flag_alu),
    .Result    (alu_result_normal)
);

// 移位器模块实例化
wire [31:0] shift_result;

shifter shift_inst (
    .A      (shift_input),
    .B      (shift_amount),
    .Shiftop(shift_op),
    .Result (shift_result)
);

// 最终ALU结果选择 (移位指令使用移位器结果，其他使用ALU结果)
wire [31:0] alu_result =
    is_shift_inst ? shift_result :
                    alu_result_normal;
wire        zero_flag = is_shift_inst ? (shift_result == 32'd0) : zero_flag_alu;
wire        overflow = overflow_flag;
wire        carryout = carryout_flag;

// ====================== 7. 写回数据选择 ======================
wire [1:0] mem_addr_low = alu_result[1:0];

wire [7:0] load_byte =
    (mem_addr_low == 2'b00) ? Read_data[7:0]   :
    (mem_addr_low == 2'b01) ? Read_data[15:8]  :
    (mem_addr_low == 2'b10) ? Read_data[23:16] :
                              Read_data[31:24];

wire [15:0] load_half =
    mem_addr_low[1] ? Read_data[31:16] : Read_data[15:0];

wire [31:0] load_data =
    (funct3 == 3'b000) ? {{24{load_byte[7]}}, load_byte} :   // LB
    (funct3 == 3'b001) ? {{16{load_half[15]}}, load_half} :  // LH
    (funct3 == 3'b010) ? Read_data :                         // LW
    (funct3 == 3'b100) ? {24'b0, load_byte} :                 // LBU
    (funct3 == 3'b101) ? {16'b0, load_half} :                 // LHU
                         Read_data;

assign RF_wdata = 
    RegWrite ? (  // 如果允许写寄存器
        MemtoReg ? load_data :           // 1. 如果是从内存加载 -> 用内存数据
        (opcode == 7'b0110111) ? imm_U_sext :  // 2. 如果是 LUI 指令 -> 用扩展的立即数
        (opcode == 7'b0010111) ? PC + imm_U_sext :  // 3. 如果是 AUIPC -> 用 PC+立即数
        ((opcode == 7'b1101111) || (opcode == 7'b1100111)) ? (PC + 32'd4) :  // 4. JAL/JALR: 都写回 PC+4
        alu_result  // 5. 其他情况（R型、I型等）-> 用 ALU 结果
    ) : 32'h00000000;  // 如果不允许写寄存器 -> 固定输出 0

// ====================== 8. 内存控制 ======================

// 内存访问判断
wire is_memory_access = is_S_type || (opcode == 7'b0000011);

// 内存控制信号
assign MemWrite  = is_S_type;                    // Store指令写内存
assign MemRead   = (opcode == 7'b0000011);       // Load指令读内存

// 内存地址
assign Address = is_memory_access ? {alu_result[31:2], 2'b00} : 32'd0;

// 字节使能（支持SB/SH/SW）
wire is_SB = is_S_type && (funct3 == 3'b000);  // SB: Store Byte
wire is_SH = is_S_type && (funct3 == 3'b001);  // SH: Store Halfword
wire is_SW = is_S_type && (funct3 == 3'b010);  // SW: Store Word

// 写入数据（Store指令的数据来源）
wire [4:0] mem_shift = {mem_addr_low, 3'b000};

assign Write_data =
    is_SB ? ({24'b0, read_data_2[7:0]}   << mem_shift) : //左移是因为小端序，左移到高位
    is_SH ? ({16'b0, read_data_2[15:0]}  << mem_shift) :
    is_SW ? read_data_2 :
            32'd0;

assign Write_strb =
    is_SB ? (4'b0001 << mem_addr_low) :
    is_SH ? (4'b0011 << mem_addr_low) :
    is_SW ? 4'b1111 :
            4'b0000;

// ====================== 9. PC更新逻辑 ======================
reg [31:0] PC_reg;
assign PC = PC_reg;
wire [31:0] PC_next;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        PC_reg <= 32'h00000000;
    end
    else begin
        PC_reg <= PC_next;
    end
end

wire        branch_taken;

// ====================== Branch 标志位判断 ======================
// Branch 指令时 alu_op = SUB，ALU 计算 read_data_1 - read_data_2

wire branch_eq  = zero_flag_alu;
wire branch_ne  = ~zero_flag_alu;

wire branch_lt_signed = alu_result_normal[31] ^ overflow_flag;
wire branch_ge_signed = ~branch_lt_signed;

// 按当前 alu.v：SUB 时 carryout_flag 表示借位
wire branch_lt_unsigned = carryout_flag;
wire branch_ge_unsigned = ~carryout_flag;

// 分支条件判断：不使用 < / > / >= / <=
assign branch_taken = Branch && (
    ((funct3 == 3'b000) && branch_eq)          || // BEQ
    ((funct3 == 3'b001) && branch_ne)          || // BNE
    ((funct3 == 3'b100) && branch_lt_signed)   || // BLT
    ((funct3 == 3'b101) && branch_ge_signed)   || // BGE
    ((funct3 == 3'b110) && branch_lt_unsigned) || // BLTU
    ((funct3 == 3'b111) && branch_ge_unsigned)    // BGEU
);

// PC_next计算
assign PC_next = 
    (Jump && (opcode == 7'b1101111)) ? PC + imm_J_sext :          // JAL
    (Jump && (opcode == 7'b1100111)) ? {alu_result[31:1], 1'b0} :  // JALR
    (Branch && branch_taken)         ? PC + imm_B_sext :          // Branch taken
    PC + 32'd4;                                                     // 默认

endmodule