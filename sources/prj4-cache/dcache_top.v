`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

// 数据Cache模块：4路组相联，容量1KB，块大小32Byte
// 写策略：写回(Write-back) + 写分配(Write-allocate)
// 替换策略：轮替替换(Round-Robin)
// 不可缓存区域：0x00-0x1F，0x40000000及以上
// 接口：CPU侧32位读写，内存侧32位突发传输
module dcache_top (
	input	      clk,
	input	      rst,
  
	//CPU interface
	/** CPU memory/IO access request to Cache: valid signal */
	input         from_cpu_mem_req_valid,
	/** CPU memory/IO access request to Cache: 0 for read; 1 for write (when req_valid is high) */
	input         from_cpu_mem_req,
	/** CPU memory/IO access request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_mem_req_addr,
	/** CPU memory/IO access request to Cache: 32-bit write data */
	input  [31:0] from_cpu_mem_req_wdata,
	/** CPU memory/IO access request to Cache: 4-bit write strobe */
	input  [ 3:0] from_cpu_mem_req_wstrb,
	/** Acknowledgement from Cache: ready to receive CPU memory access request */
	output        to_cpu_mem_req_ready,
		
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit read data */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive read data */
	input         from_cpu_cache_rsp_ready,
		
	//Memory/IO read interface
	/** Cache sending memory/IO read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address
	  * 4 byte alignment for I/O read 
	  * 32 byte alignment for cache read miss */
	output [31:0] to_mem_rd_req_addr,
        /** Cache sending memory read request: burst length
	  * 0 for I/O read (read only one data beat)
	  * 7 for cache read miss (read eight data beats) */
	output [ 7:0] to_mem_rd_req_len,
        /** Acknowledgement from memory: ready to receive memory read request */
	input	      from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input	      from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input	      from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready,

	//Memory/IO write interface
	/** Cache sending memory/IO write request: valid signal */
	output        to_mem_wr_req_valid,
	/** Cache sending memory write request: address
	  * 4 byte alignment for I/O write 
	  * 4 byte alignment for cache write miss
          * 32 byte alignment for cache write-back */
	output [31:0] to_mem_wr_req_addr,
        /** Cache sending memory write request: burst length
          * 0 for I/O write (write only one data beat)
          * 0 for cache write miss (write only one data beat)
          * 7 for cache write-back (write eight data beats) */
	output [ 7:0] to_mem_wr_req_len,
        /** Acknowledgement from memory: ready to receive memory write request */
	input         from_mem_wr_req_ready,

	/** Cache sending memory/IO write data: valid signal for current data beat */
	output        to_mem_wr_data_valid,
	/** Cache sending memory/IO write data: current data beat */
	output [31:0] to_mem_wr_data,
	/** Cache sending memory/IO write data: write strobe
	  * 4'b1111 for cache write-back 
	  * other values for I/O write and cache write miss according to the original CPU request*/ 
	output [ 3:0] to_mem_wr_data_strb,
	/** Cache sending memory/IO write data: if current data beat is the last in this burst data transmission */
	output        to_mem_wr_data_last,
	/** Acknowledgement from memory/IO: ready to receive current data beat */
	input	      from_mem_wr_data_ready
);

// -------------------- 状态机定义 --------------------
localparam S_IDLE            = 4'd0;    // 空闲等待CPU请求
localparam S_TAG_RD          = 4'd1;    // 读Tag并比较命中
localparam S_HIT_PROC        = 4'd2;    // 命中处理（读/写）
localparam S_EVICT           = 4'd3;    // 选择替换路并失效
localparam S_EVICT_CHECK     = 4'd4;    // 新增：路号锁存后，检查脏位
localparam S_WRITEBACK_REQ   = 4'd5;    // 发写回请求
localparam S_WRITEBACK_DATA  = 4'd6;    // 发送写回数据
localparam S_REFILL_REQ      = 4'd7;    // 发内存读请求重填
localparam S_REFILL_RECV     = 4'd8;    // 接收重填数据
localparam S_REFILL_DONE     = 4'd9;    // 重填完成更新Cache
localparam S_BYPASS_RD_REQ   = 4'd10;   // 旁路读：发读请求
localparam S_BYPASS_RD_WAIT  = 4'd11;   // 旁路读：等待数据返回
localparam S_BYPASS_WR_REQ   = 4'd12;   // 旁路写：发写请求
localparam S_BYPASS_WR_DATA  = 4'd13;   // 旁路写：发送写数据
localparam S_RESP            = 4'd14;   // 向CPU返回响应

// 状态寄存器
reg  [3:0] curr_state;
reg  [3:0] next_state;

// -------------------- 请求锁存与地址拆分 --------------------
reg         req_is_write;     // 锁存请求类型：0读，1写
reg  [31:0] req_addr;         // 锁存请求地址
reg  [31:0] req_wdata;        // 锁存写数据
reg  [ 3:0] req_wstrb;        // 锁存写字节使能

wire [23:0] req_tag;          // Tag域 [31:8]
wire [ 2:0] req_index;        // Index域 [7:5]
wire [ 4:0] req_offset;       // Offset域 [4:0]
wire [ 2:0] word_sel;         // 块内字选择信号 [4:2]

assign req_tag    = req_addr[31:8];
assign req_index  = req_addr[7:5];
assign req_offset = req_addr[4:0];
assign word_sel   = req_offset[4:2];

// -------------------- 存储阵列：4路Valid/Dirty/Tag/Data --------------------
reg  [7:0] valid [0:3];      // 每路8个有效位，valid[way][set]
reg  [7:0] dirty [0:3];      // 每路8个脏位，dirty[way][set]
wire [23:0] tag_rdata [0:3]; // Tag阵列读数据
wire [255:0] data_rdata [0:3];// Data阵列读数据
reg  [3:0] tag_wen;           // Tag阵列写使能，每路独立
reg  [3:0] data_wen;          // Data阵列写使能，每路独立
reg  [255:0] data_wdata;      // Data阵列写数据

// -------------------- 命中判断逻辑 --------------------
wire [3:0] hit_way_onehot;    // 命中路独热码
wire       hit;               // 命中标志
reg  [1:0] hit_way;           // 命中路编号（二进制）

assign hit_way_onehot[0] = valid[0][req_index] & (tag_rdata[0] == req_tag);
assign hit_way_onehot[1] = valid[1][req_index] & (tag_rdata[1] == req_tag);
assign hit_way_onehot[2] = valid[2][req_index] & (tag_rdata[2] == req_tag);
assign hit_way_onehot[3] = valid[3][req_index] & (tag_rdata[3] == req_tag);
assign hit = |hit_way_onehot;

always @(*) begin
    case(hit_way_onehot)
        4'b0001: hit_way = 2'd0;
        4'b0010: hit_way = 2'd1;
        4'b0100: hit_way = 2'd2;
        4'b1000: hit_way = 2'd3;
        default: hit_way = 2'd0;
    endcase
end

// -------------------- 轮替替换指针控制 --------------------
reg  [1:0] evict_way_cnt;     // 轮替替换指针
reg  [1:0] evict_way_reg;     // 当前流程锁定的替换路号

always @(posedge clk) begin
    if(rst) begin
        evict_way_cnt <= 2'd0;
        evict_way_reg <= 2'd0;
    end
    else if(curr_state == S_EVICT) begin
        evict_way_reg <= evict_way_cnt;            // 锁存本次要替换的路
        evict_way_cnt <= evict_way_cnt + 2'd1;     // 指针指向下一路
    end
end

// -------------------- 数据合并函数 --------------------
// 按字节使能将32位写数据合并到256位Cache行的指定字
function [255:0] merge_dcache_word;
    input [255:0] old_line;
    input [2:0]   word_idx;
    input [31:0]  wdata;
    input [3:0]   wstrb;
    integer i;
    begin
        merge_dcache_word = old_line;
        for(i = 0; i < 4; i = i + 1) begin
            if(wstrb[i]) begin
                merge_dcache_word[{word_idx, 5'b0} + {i, 3'b0} +: 8] = wdata[{i, 3'b0} +: 8];
            end
        end
    end
endfunction

// -------------------- 存储阵列例化（与I-Cache完全一致） --------------------
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
        .wdata (data_wdata),
        .rdata (data_rdata[i])
    );
end
endgenerate

// -------------------- Valid与Dirty位控制 --------------------
always @(posedge clk) begin
    if(rst) begin
        valid[0] <= 8'd0;
        valid[1] <= 8'd0;
        valid[2] <= 8'd0;
        valid[3] <= 8'd0;
        dirty[0] <= 8'd0;
        dirty[1] <= 8'd0;
        dirty[2] <= 8'd0;
        dirty[3] <= 8'd0;
    end
    else begin
        // 替换时失效对应块，同时清脏位
        if(curr_state == S_EVICT_CHECK) begin
            valid[evict_way_reg][req_index] <= 1'b0;
            dirty[evict_way_reg][req_index] <= 1'b0;
        end
        // 重填更新：有效位置1，脏位按请求类型设置
        if(curr_state == S_REFILL_DONE) begin
            valid[evict_way_reg][req_index] <= 1'b1;
            dirty[evict_way_reg][req_index] <= req_is_write ? 1'b1 : 1'b0;
        end
        // 写命中：脏位置1
        if(curr_state == S_HIT_PROC && req_is_write && hit) begin
            dirty[hit_way][req_index] <= 1'b1;
        end
    end
end

// -------------------- Tag/Data阵列写控制 --------------------
always @(*) begin
    tag_wen    = 4'b0;
    data_wen   = 4'b0;
    data_wdata = 256'd0;
    
    // 重填：写入Tag和完整Data行
    if(curr_state == S_REFILL_DONE) begin
        tag_wen[evict_way_reg]  = 1'b1;
        data_wen[evict_way_reg] = 1'b1;
        // 写缺失需合并CPU写数据到新行
        if(req_is_write) begin
            data_wdata = merge_dcache_word(refill_data, word_sel, req_wdata, req_wstrb);
        end
        else begin
            data_wdata = refill_data;
        end
    end
    
    // 写命中：合并后写入对应路的Data行
    if(curr_state == S_HIT_PROC && req_is_write && hit) begin
        data_wen[hit_way] = 1'b1;
        data_wdata = merge_dcache_word(data_rdata[hit_way], word_sel, req_wdata, req_wstrb);
    end
end

// -------------------- 写回控制 --------------------
reg  [2:0] wr_cnt;            // 写回数据拍数计数
wire [31:0] wb_data;          // 当前写回的32位数据

assign wb_data = data_rdata[evict_way_reg][{wr_cnt, 5'b0} +: 32];

always @(posedge clk) begin
    if(rst) begin
        wr_cnt <= 3'd0;
    end
    else if(curr_state == S_WRITEBACK_DATA && from_mem_wr_data_ready) begin
        wr_cnt <= wr_cnt + 3'd1;
    end
    else if(curr_state == S_IDLE) begin
        wr_cnt <= 3'd0;
    end
end

// -------------------- 重填数据接收 --------------------
reg  [2:0] recv_cnt;          // 接收数据拍数计数
reg  [255:0] refill_data;     // 重填数据缓存（完整Cache行）

always @(posedge clk) begin
    if(rst) begin
        recv_cnt    <= 3'd0;
        refill_data <= 256'd0;
    end
    else if(curr_state == S_REFILL_RECV && from_mem_rd_rsp_valid) begin
        refill_data[{recv_cnt, 5'b0} +: 32] <= from_mem_rd_rsp_data;
        recv_cnt <= recv_cnt + 3'd1;
    end
    else if(curr_state == S_IDLE) begin
        recv_cnt <= 3'd0;
    end
end

// -------------------- 旁路读数据缓存 --------------------
reg  [31:0] bypass_rdata;

always @(posedge clk) begin
    if(rst) begin
        bypass_rdata <= 32'd0;
    end
    else if(curr_state == S_BYPASS_RD_WAIT && from_mem_rd_rsp_valid) begin
        bypass_rdata <= from_mem_rd_rsp_data;
    end
end

// -------------------- 响应数据寄存器 --------------------
reg  [31:0] rsp_data_reg;

always @(posedge clk) begin
    if(rst) begin
        rsp_data_reg <= 32'd0;
    end
    else begin
        // 读命中：锁存命中路的Cache数据
        if(curr_state == S_HIT_PROC && !req_is_write) begin
            rsp_data_reg <= data_rdata[hit_way][{word_sel, 5'b0} +: 32];
        end
        // 读缺失重填完成：锁存重填后的数据
        if(curr_state == S_REFILL_DONE && !req_is_write) begin
            rsp_data_reg <= refill_data[{word_sel, 5'b0} +: 32];
        end
        // 旁路读：锁存内存返回的数据
        if(curr_state == S_BYPASS_RD_WAIT && from_mem_rd_rsp_valid) begin
            rsp_data_reg <= from_mem_rd_rsp_data;
        end
    end
end

// -------------------- 请求锁存 --------------------
always @(posedge clk) begin
    if(rst) begin
        req_is_write <= 1'b0;
        req_addr     <= 32'd0;
        req_wdata    <= 32'd0;
        req_wstrb    <= 4'd0;
    end
    else if(curr_state == S_IDLE && from_cpu_mem_req_valid) begin
        req_is_write <= from_cpu_mem_req;
        req_addr     <= from_cpu_mem_req_addr;
        req_wdata    <= from_cpu_mem_req_wdata;
        req_wstrb    <= from_cpu_mem_req_wstrb;
    end
end

// -------------------- 状态机时序逻辑 --------------------
always @(posedge clk) begin
    if(rst) begin
        curr_state <= S_IDLE;
    end
    else begin
        curr_state <= next_state;
    end
end

// -------------------- 状态机组合转移逻辑 --------------------
always @(*) begin
    next_state = curr_state;
    case(curr_state)
        S_IDLE: begin
            if(from_cpu_mem_req_valid) begin
                // 按地址判断是否可缓存
                if( (from_cpu_mem_req_addr < 32'h00000020) || (from_cpu_mem_req_addr >= 32'h40000000) ) begin
                    // 不可缓存，走旁路
                    if(from_cpu_mem_req) begin
                        next_state = S_BYPASS_WR_REQ;
                    end
                    else begin
                        next_state = S_BYPASS_RD_REQ;
                    end
                end
                else begin
                    // 可缓存，进入Tag比较
                    next_state = S_TAG_RD;
                end
            end
        end
        
        S_TAG_RD: begin
            if(hit) begin
                next_state = S_HIT_PROC;
            end
            else begin
                next_state = S_EVICT;
            end
        end
        
        S_HIT_PROC: begin
            if(req_is_write) begin
                // 写命中：更新Cache和脏位后直接结束，不需要给CPU返回响应
                next_state = S_IDLE;
            end
            else begin
                // 读命中：正常进入响应状态等待CPU握手
                next_state = S_RESP;
            end
        end
        
        S_EVICT: begin
            // 本拍只做路号锁存，下一拍再检查脏位
            next_state = S_EVICT_CHECK;
        end
        
        S_EVICT_CHECK: begin
            // 此时 evict_way_reg 已经稳定为本次替换路号，再判断是否脏块
            if(dirty[evict_way_reg][req_index]) begin
                next_state = S_WRITEBACK_REQ;
            end
            else begin
                next_state = S_REFILL_REQ;
            end
        end

        S_WRITEBACK_REQ: begin
            if(from_mem_wr_req_ready) begin
                next_state = S_WRITEBACK_DATA;
            end
        end
        
        S_WRITEBACK_DATA: begin
            // 最后一拍数据握手完成，进入重填流程
            if(from_mem_wr_data_ready && wr_cnt == 3'd7) begin
                next_state = S_REFILL_REQ;
            end
        end
        
        S_REFILL_REQ: begin
            if(from_mem_rd_req_ready) begin
                next_state = S_REFILL_RECV;
            end
        end
        
        S_REFILL_RECV: begin
            if(from_mem_rd_rsp_valid && from_mem_rd_rsp_last) begin
                next_state = S_REFILL_DONE;
            end
        end
        
        S_REFILL_DONE: begin
            if(req_is_write) begin
                // 写缺失重填完成：新行已写入、脏位已置1，直接结束
                next_state = S_IDLE;
            end
            else begin
                // 读缺失重填完成：进入响应状态返回数据
                next_state = S_RESP;
            end
        end
        
        // 旁路读流程
        S_BYPASS_RD_REQ: begin
            if(from_mem_rd_req_ready) begin
                next_state = S_BYPASS_RD_WAIT;
            end
        end
        
        S_BYPASS_RD_WAIT: begin
            if(from_mem_rd_rsp_valid) begin
                next_state = S_RESP;
            end
        end
        
        // 旁路写流程
        S_BYPASS_WR_REQ: begin
            if(from_mem_wr_req_ready) begin
                next_state = S_BYPASS_WR_DATA;
            end
        end
        
        S_BYPASS_WR_DATA: begin
            if(from_mem_wr_data_ready) begin
                // 旁路写数据握手完成，直接结束，不进入响应
                next_state = S_IDLE;
            end
        end
        
        S_RESP: begin
            if(from_cpu_cache_rsp_ready) begin
                next_state = S_IDLE;
            end
        end
        
        default: begin
            next_state = S_IDLE;
        end
    endcase
end

// -------------------- 输出信号赋值 --------------------
// CPU侧请求就绪
assign to_cpu_mem_req_ready = (curr_state == S_IDLE);

// CPU侧响应
assign to_cpu_cache_rsp_valid = (curr_state == S_RESP);
assign to_cpu_cache_rsp_data  = rsp_data_reg;

// 内存侧读接口
assign to_mem_rd_req_valid = (curr_state == S_REFILL_REQ) || (curr_state == S_BYPASS_RD_REQ);
assign to_mem_rd_req_addr  = (curr_state == S_REFILL_REQ) ? 
                            {req_tag, req_index, 5'b0} : 
                            {req_addr[31:2], 2'b00};
assign to_mem_rd_req_len   = (curr_state == S_REFILL_REQ) ? 8'd7 : 8'd0;
assign to_mem_rd_rsp_ready = (curr_state == S_REFILL_RECV) || (curr_state == S_BYPASS_RD_WAIT);

// 内存侧写接口
assign to_mem_wr_req_valid = (curr_state == S_WRITEBACK_REQ) || (curr_state == S_BYPASS_WR_REQ);
assign to_mem_wr_req_addr  = (curr_state == S_WRITEBACK_REQ) ? 
                            {tag_rdata[evict_way_reg], req_index, 5'b0} : 
                            {req_addr[31:2], 2'b00};
assign to_mem_wr_req_len   = (curr_state == S_WRITEBACK_REQ) ? 8'd7 : 8'd0;

assign to_mem_wr_data_valid = (curr_state == S_WRITEBACK_DATA) || (curr_state == S_BYPASS_WR_DATA);
assign to_mem_wr_data       = (curr_state == S_WRITEBACK_DATA) ? wb_data : req_wdata;
assign to_mem_wr_data_strb  = (curr_state == S_WRITEBACK_DATA) ? 4'b1111 : req_wstrb;
assign to_mem_wr_data_last  = (curr_state == S_WRITEBACK_DATA) ? (wr_cnt == 3'd7) : 1'b1;

endmodule