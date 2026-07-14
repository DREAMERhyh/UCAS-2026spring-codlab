`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 
// Design Name: 
// Module Name: engine_core
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: DMA引擎核心逻辑 - 基于队列的突发传输控制器
//              修正：读写引擎解耦并行，消除死锁；增加RD_DONE防止重复读
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module engine_core #(
	parameter integer  DATA_WIDTH       = 32
)
(
	input    clk,
	input    rst,
	
	output [31:0]       src_base,
	output [31:0]       dest_base,
	output [31:0]       tail_ptr,
	output [31:0]       head_ptr,
	output [31:0]       dma_size,
	output [31:0]       ctrl_stat,
	input  [31:0]	    reg_wr_data,
	input  [ 5:0]       reg_wr_en,
  
	output              intr,
  
	output [31:0]       rd_req_addr,
	output [ 4:0]       rd_req_len,
	output              rd_req_valid,
	
	input               rd_req_ready,
	input  [31:0]       rd_rdata,
	input               rd_last,
	input               rd_valid,
	output              rd_ready,
	
	output [31:0]       wr_req_addr,
	output [ 4:0]       wr_req_len,
	output              wr_req_valid,
	input               wr_req_ready,
	output [31:0]       wr_data,
	output              wr_valid,
	input               wr_ready,
	output              wr_last,
	
	output              fifo_rden,
	output [31:0]       fifo_wdata,
	output              fifo_wen,
	
	input  [31:0]       fifo_rdata,
	input               fifo_is_empty,
	input               fifo_is_full
);

// ====================== 读引擎状态定义 ======================
localparam RD_IDLE  = 2'b00;
localparam RD_REQ   = 2'b01;
localparam RD_DATA  = 2'b10;
localparam RD_DONE  = 2'b11;   // 子缓冲读完，等待写完成

// ====================== 写引擎状态定义 ======================
localparam WR_IDLE  = 2'b00;
localparam WR_REQ   = 2'b01;
localparam WR_DATA  = 2'b10;

// ====================== 内部控制寄存器 ======================
reg [31:0] reg_src_base;
reg [31:0] reg_dest_base;
reg [31:0] reg_tail_ptr;
reg [31:0] reg_head_ptr;
reg [31:0] reg_dma_size;
reg [31:0] reg_ctrl_stat; // bit0: EN, bit31: INTR

// ====================== 读引擎内部信号 ======================
reg [1:0]  rd_state;
reg [31:0] rd_addr;
reg [31:0] rd_bytes_remain;

// ====================== 写引擎内部信号 ======================
reg [1:0]  wr_state;
reg [31:0] wr_addr;
reg [31:0] wr_bytes_remain;
reg [4:0]  wr_data_cnt; // 本次Burst已发送数据个数
reg [4:0]  wr_burst_len_r; // 锁存的本次Burst长度

// ====================== FIFO数据计数 ======================
reg [8:0] fifo_cnt; // FIFO中32位字的数量
wire subbuf_done;   // 单缓冲区传输完成标志（写侧产生）

// ====================== 控制寄存器读写逻辑 ======================
assign src_base  = reg_src_base;
assign dest_base = reg_dest_base;
assign tail_ptr  = reg_tail_ptr;
assign head_ptr  = reg_head_ptr;
assign dma_size  = reg_dma_size;
assign ctrl_stat = reg_ctrl_stat;
assign intr      = reg_ctrl_stat[31];

always @(posedge clk) begin
    if (rst) begin
        reg_src_base  <= 32'h0;
        reg_dest_base <= 32'h0;
        reg_tail_ptr  <= 32'h0;
        reg_head_ptr  <= 32'h0;
        reg_dma_size  <= 32'h0;
        reg_ctrl_stat <= 32'h0;
    end else begin
        // 0: src_base
        if (reg_wr_en[0]) begin
            reg_src_base <= reg_wr_data;
        end
        // 1: dest_base
        if (reg_wr_en[1]) begin
            reg_dest_base <= reg_wr_data;
        end
        // 2: tail_ptr / DMA自动更新
        if (reg_wr_en[2]) begin
            reg_tail_ptr <= reg_wr_data;
        end else if (subbuf_done) begin
            reg_tail_ptr <= reg_tail_ptr + reg_dma_size;
        end
        // 3: head_ptr
        if (reg_wr_en[3]) begin
            reg_head_ptr <= reg_wr_data;
        end
        // 4: dma_size
        if (reg_wr_en[4]) begin
            reg_dma_size <= reg_wr_data;
        end
        // 5: ctrl_stat
        if (reg_wr_en[5]) begin
            reg_ctrl_stat[0] <= reg_wr_data[0]; // EN位
            // INTR位：写1清零
            if (reg_wr_data[31]) begin
                reg_ctrl_stat[31] <= 1'b0;
            end
        end
        // 硬件置位INTR（优先级高于软件清零）
        if (subbuf_done) begin
            reg_ctrl_stat[31] <= 1'b1;
        end
    end
end

// ====================== FIFO数据量计数器 ======================
always @(posedge clk) begin
    if (rst) begin
        fifo_cnt <= 6'd0;
    end else begin
        case ({fifo_wen, fifo_rden})
            2'b10: fifo_cnt <= fifo_cnt + 1'b1;
            2'b01: fifo_cnt <= fifo_cnt - 1'b1;
            default: ;
        endcase
    end
end

// ====================== 读引擎：Burst长度计算 ======================
// 单次Burst最大32字节(8字)，最后一次向上4字节对齐
wire [31:0] rd_curr_rem = rd_bytes_remain;
wire [31:0] rd_burst_bytes = (rd_curr_rem >= 32) ? 32 : ((rd_curr_rem + 3) & ~32'd3);
wire [4:0]  rd_burst_len = (rd_burst_bytes >> 2) - 1'b1;


// ====================== 读引擎状态机 ======================
always @(posedge clk) begin
    if (rst) begin
        rd_state <= RD_IDLE;
        rd_addr <= 32'h0;
        rd_bytes_remain <= 32'h0;
    end else begin
        case (rd_state)
            RD_IDLE: begin
                // DMA使能且队列非空，启动新子缓冲区读取
                // 修正：不再依赖写引擎状态，读写完全解耦
                if (reg_ctrl_stat[0] && (reg_tail_ptr != reg_head_ptr)) begin
                    rd_state <= RD_REQ;
                    rd_addr <= reg_src_base + reg_tail_ptr;
                    rd_bytes_remain <= reg_dma_size;
                end
            end
            RD_REQ: begin
                // 读请求握手成功
                if (rd_req_valid && rd_req_ready) begin
                    rd_state <= RD_DATA;
                    rd_bytes_remain <= rd_bytes_remain - rd_burst_bytes;
                end
            end
            RD_DATA: begin
                // 收到最后一个数据（包括锁存的 last），本次Burst结束
                if (rd_valid && rd_ready && rd_last) begin
                    if (rd_bytes_remain == 0) begin
                        // 整个子缓冲区读取完成，进入等待状态
                        rd_state <= RD_DONE;
                    end else begin
                        // 继续下一个Burst请求
                        rd_state <= RD_REQ;
                        rd_addr <= rd_addr + rd_burst_bytes;
                    end
                end
            end
            RD_DONE: begin
                // 等待写引擎完成该子缓冲区，再回到IDLE处理下一块
                if (subbuf_done) begin
                    rd_state <= RD_IDLE;
                end
            end
        endcase
    end
end

// 读引擎端口输出
assign rd_req_valid = (rd_state == RD_REQ);
assign rd_req_addr  = rd_addr;   // 修正：使用实际地址，不强制32B对齐
assign rd_req_len   = rd_burst_len;
assign rd_ready     = (rd_state == RD_DATA) && !fifo_is_full;

// FIFO写控制
assign fifo_wen   = rd_valid && rd_ready;
assign fifo_wdata = rd_rdata;

// ====================== 写引擎：Burst长度计算 ======================
wire [31:0] wr_curr_rem = wr_bytes_remain;
wire [31:0] wr_burst_bytes = (wr_curr_rem >= 32) ? 32 : ((wr_curr_rem + 3) & ~32'd3);
wire [4:0]  wr_burst_len = (wr_burst_bytes >> 2) - 1'b1;
wire [4:0]  wr_burst_total = wr_burst_len + 1'b1; // 本次Burst总字数

// ====================== 写引擎状态机 ======================
always @(posedge clk) begin
    if (rst) begin
        wr_state <= WR_IDLE;
        wr_addr <= 32'h0;
        wr_bytes_remain <= 32'h0;
        wr_data_cnt <= 5'b0;
    end else begin
        case (wr_state)
            WR_IDLE: begin
                // DMA使能且队列非空，启动新子缓冲区写入
                // 修正：不再依赖读引擎状态，读写完全解耦
                if (reg_ctrl_stat[0] && (reg_tail_ptr != reg_head_ptr)) begin
                    wr_state <= WR_REQ;
                    wr_addr <= reg_dest_base + reg_tail_ptr;
                    wr_bytes_remain <= reg_dma_size;
                end
            end
            WR_REQ: begin
                if (wr_req_valid && wr_req_ready) begin
                    wr_state <= WR_DATA;
                    wr_data_cnt <= 5'b0;
                    wr_bytes_remain <= wr_bytes_remain - wr_burst_bytes;
                    wr_burst_len_r <= wr_burst_len;   // 锁存本次突发长度
                end
            end
            WR_DATA: begin
                if (wr_valid && wr_ready) begin
                    wr_data_cnt <= wr_data_cnt + 1'b1;
                    if (wr_last) begin
                        // 本次Burst发送完成
                        if (wr_bytes_remain == 0) begin
                            // 整个子缓冲区写入完成
                            wr_state <= WR_IDLE;
                        end else begin
                            // 继续下一个Burst请求
                            wr_state <= WR_REQ;
                            wr_addr <= wr_addr + wr_burst_bytes;
                        end
                    end
                end
            end
        endcase
    end
end

// 写引擎端口输出
assign wr_req_valid = (wr_state == WR_REQ) && (fifo_cnt >= wr_burst_total);
assign wr_req_addr  = wr_addr;   // 修正：使用实际地址
assign wr_req_len   = wr_burst_len;

assign wr_valid = (wr_state == WR_DATA) && !fifo_is_empty;
assign wr_data  = fifo_rdata;
assign wr_last  = (wr_state == WR_DATA) && (wr_data_cnt == wr_burst_len_r);

// FIFO读控制
assign fifo_rden = wr_valid && wr_ready;

// ====================== 子缓冲区完成标志 ======================
// 写引擎写完最后一个字且剩余字节为0时，表示整个子缓冲区搬移完成
assign subbuf_done = (wr_state == WR_DATA) && wr_valid && wr_ready && wr_last && (wr_bytes_remain == 0);

endmodule