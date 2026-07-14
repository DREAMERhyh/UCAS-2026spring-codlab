`timescale 10ns / 1ns
`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

// 指令Cache模块：4路组相联，容量1KB，块大小32Byte
// 替换策略：轮替替换（Round-Robin）
// 接口：CPU侧32位指令读写，内存侧32位突发传输
module icache_top (
	input	      clk,
	input	      rst,
	
	//CPU interface
	/** CPU instruction fetch request to Cache: valid signal */
	input         from_cpu_inst_req_valid,
	/** CPU instruction fetch request: address (4 byte alignment) */
	input  [31:0] from_cpu_inst_req_addr,
	/** Acknowledgement from Cache: ready to receive CPU instruction fetch request */
	output        to_cpu_inst_req_ready,
	
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit Instruction value */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive Instruction */
	input	      from_cpu_cache_rsp_ready,

	//Memory interface (32 byte aligned address)
	/** Cache sending memory read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address (32 byte alignment) */
	output [31:0] to_mem_rd_req_addr,
	/** Acknowledgement from memory: ready to receive memory read request */
	input         from_mem_rd_req_ready,
	/** Memory return read data: valid signal of one data beat */
	input         from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst */
	input         from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready
);

// 状态机定义
localparam S_WAIT     = 4'd0;  // 等待CPU请求
localparam S_TAG_RD   = 4'd1;  // 读Tag并比较命中
localparam S_CACHE_RD = 4'd2;  // 读Cache数据
localparam S_RESP     = 4'd3;  // 向CPU返回响应
localparam S_EVICT    = 4'd4;  // 选择替换路并失效
localparam S_MEM_RD   = 4'd5;  // 向内存发读请求
localparam S_RECV     = 4'd6;  // 接收内存突发数据
localparam S_REFILL   = 4'd7;  // 重填Cache行

// 状态寄存器
reg  [3:0] curr_state;
reg  [3:0] next_state;

// 请求地址锁存与地址拆分
reg  [31:0] req_addr;       // 锁存当前处理的请求地址
wire [23:0] req_tag;        // Tag域 [31:8]
wire [ 2:0] req_index;      // Index域 [7:5]
wire [ 4:0] req_offset;     // Offset域 [4:0]
wire [ 2:0] word_sel;       // 32位字选择信号 [4:2]

assign req_tag    = req_addr[31:8];
assign req_index  = req_addr[7:5];
assign req_offset = req_addr[4:0];
assign word_sel   = req_offset[4:2];

// 存储阵列：4路Valid/Tag/Data
reg  [7:0] valid [0:3];     // 每路8个有效位，valid[way][set]
wire [23:0] tag_rdata [0:3];// Tag阵列读数据
wire [255:0] data_rdata [0:3];// Data阵列读数据
reg  [3:0] tag_wen;         // Tag阵列写使能，每路独立
reg  [3:0] data_wen;        // Data阵列写使能，每路独立

// 命中判断信号
wire [3:0] hit_way_onehot;  // 命中路独热码
wire       hit;             // 命中标志
reg  [1:0] hit_way;         // 命中路编号（二进制）

// 替换控制
reg  [1:0] evict_way_cnt;   // 轮替替换指针
reg  [1:0] evict_way_reg;   // 当前流程锁定的替换路号

// 突发接收控制
reg  [2:0] recv_cnt;        // 接收数据拍数计数
reg  [255:0] refill_data;   // 重填数据缓存（完整Cache行）

// 响应数据选择
reg  [31:0] rsp_data;


// 存储阵列例化：4路Tag阵列与4路Data阵列

genvar i;
generate
for(i = 0; i < 4; i = i + 1) begin : gen_way_array
    tag_array u_tag_array (
        .clk   (clk),
        .waddr (req_index),
        .raddr (req_index),
        .wen   (tag_wen[i]),
        .wdata (req_tag),
        .rdata (tag_rdata[i])
    );

    data_array u_data_array (
        .clk   (clk),
        .waddr (req_index),
        .raddr (req_index),
        .wen   (data_wen[i]),
        .wdata (refill_data),
        .rdata (data_rdata[i])
    );
end
endgenerate


// 地址锁存：锁存CPU请求地址，保证处理过程中地址稳定

always @(posedge clk) begin
    if(rst) begin
        req_addr <= 32'd0;
    end
    else if(curr_state == S_WAIT && from_cpu_inst_req_valid) begin
        req_addr <= from_cpu_inst_req_addr;
    end
end


// 命中判断逻辑：比较4路Tag与有效位
//当前位置有效 + 和cpu请求地址相等 = 命中
assign hit_way_onehot[0] = valid[0][req_index] & (tag_rdata[0] == req_tag);
assign hit_way_onehot[1] = valid[1][req_index] & (tag_rdata[1] == req_tag);
assign hit_way_onehot[2] = valid[2][req_index] & (tag_rdata[2] == req_tag);
assign hit_way_onehot[3] = valid[3][req_index] & (tag_rdata[3] == req_tag);

assign hit = |hit_way_onehot; //相当于所有位一起作或


always @(*) begin
    case(hit_way_onehot)
        4'b0001: hit_way = 2'd0;
        4'b0010: hit_way = 2'd1;
        4'b0100: hit_way = 2'd2;
        4'b1000: hit_way = 2'd3;
        default: hit_way = 2'd0;
    endcase
end


// 轮替替换指针控制

always @(posedge clk) begin
    if(rst) begin
        evict_way_cnt <= 2'd0;
        evict_way_reg <= 2'd0;
    end
    else if(curr_state == S_EVICT) begin
        evict_way_reg <= evict_way_cnt;            // 锁存本次要替换的路
        evict_way_cnt <= evict_way_cnt + 2'd1;     // 指针指向下一路（给下次替换用）
    end
end


// Valid位控制：复位清0、替换清0、重填置1

always @(posedge clk) begin
    if(rst) begin
        valid[0] <= 8'd0;
        valid[1] <= 8'd0;
        valid[2] <= 8'd0;
        valid[3] <= 8'd0;
    end
    else if(curr_state == S_EVICT) begin
        valid[evict_way_reg][req_index] <= 1'b0;
    end
    else if(curr_state == S_REFILL) begin
        valid[evict_way_reg][req_index] <= 1'b1;
    end
end


// Tag/Data阵列写使能控制

always @(*) begin
    tag_wen  = 4'b0;
    data_wen = 4'b0;
    if(curr_state == S_REFILL) begin
        tag_wen[evict_way_reg]  = 1'b1;
        data_wen[evict_way_reg] = 1'b1;
    end
end


// 突发数据接收：拼接8拍32位数据为256位Cache行

always @(posedge clk) begin
    if(rst) begin
        recv_cnt    <= 3'd0;
        refill_data <= 256'd0;
    end
    else if(curr_state == S_RECV && from_mem_rd_rsp_valid) begin
        refill_data[{recv_cnt, 5'b0} +: 32] <= from_mem_rd_rsp_data;
        recv_cnt <= recv_cnt + 3'd1;
    end
    else if(curr_state == S_WAIT) begin
        recv_cnt <= 3'd0;
    end
end


// 响应数据选择：命中读Cache / 缺失重填后直接返回

always @(*) begin
    if(curr_state == S_REFILL) begin
        rsp_data = refill_data[{word_sel, 5'b0} +: 32];
    end
    else begin
        rsp_data = data_rdata[hit_way][{word_sel, 5'b0} +: 32];
    end
end


// 状态机时序逻辑

always @(posedge clk) begin
    if(rst) begin
        curr_state <= S_WAIT;
    end
    else begin
        curr_state <= next_state;
    end
end


// 状态机组合转移逻辑

always @(*) begin
    next_state = curr_state;
    case(curr_state)
        S_WAIT: begin
            if(from_cpu_inst_req_valid) begin
                next_state = S_TAG_RD;
            end
        end

        S_TAG_RD: begin
            if(hit) begin
                next_state = S_CACHE_RD;
            end
            else begin
                next_state = S_EVICT;
            end
        end

        S_CACHE_RD: begin
            next_state = S_RESP;
        end

        S_RESP: begin
            if(from_cpu_cache_rsp_ready) begin
                next_state = S_WAIT;
            end
        end

        S_EVICT: begin
            next_state = S_MEM_RD;
        end

        S_MEM_RD: begin
            if(from_mem_rd_req_ready) begin
                next_state = S_RECV;
            end
        end

        S_RECV: begin
            if(from_mem_rd_rsp_valid && from_mem_rd_rsp_last) begin
                next_state = S_REFILL;
            end
        end

        S_REFILL: begin
            next_state = S_RESP;
        end

        default: begin
            next_state = S_WAIT;
        end
    endcase
end


// 输出信号赋值

assign to_cpu_inst_req_ready = (curr_state == S_WAIT);
assign to_cpu_cache_rsp_valid = (curr_state == S_RESP);
assign to_cpu_cache_rsp_data  = rsp_data;

assign to_mem_rd_req_valid = (curr_state == S_MEM_RD);
assign to_mem_rd_req_addr  = {req_addr[31:5], 5'b0}; // 32字节对齐，低5位清0

assign to_mem_rd_rsp_ready = (curr_state == S_RECV);

endmodule