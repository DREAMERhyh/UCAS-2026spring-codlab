# 全局变量：记录上一次中断时的 DMA 尾指针，用于计算增量
.data
.align 2
last_tail_ptr:
        .word 0

.section .exception
.globl intr_handler
.align  2
.type intr_handler, @function

intr_handler:
        # ========== 1. 开辟栈帧，保存所有用到的寄存器 ==========
        # 共使用 t0~t5 6个临时寄存器，开32字节栈（满足16字节栈对齐规范）
        addi    sp, sp, -32
        sw      t0, 0(sp)
        sw      t1, 4(sp)
        sw      t2, 8(sp)
        sw      t3, 12(sp)
        sw      t4, 16(sp)
        sw      t5, 20(sp)

        # ========== 2. 清除 DMA 中断标志（适配硬件：写1清零 + 保持DMA使能） ==========
        # 硬件 INTR 位是写1清零(W1C)，读改写0无效；必须写入 bit31=1 才能清中断
        # 同时 bit0 必须写1，否则会关闭 DMA 引擎，导致死循环
        li      t0, 0x60020000        # DMA 寄存器基地址（讲义标准地址）
        li      t1, 0x80000001        # bit31=1 清中断，bit0=1 保持DMA使能
        sw      t1, 0x14(t0)          # 写入 ctrl_stat 寄存器（偏移0x14）

        # ========== 3. 计算本次中断完成的子缓冲区数量（讲义标准方案） ==========
        lw      t1, 0x08(t0)          # t1 = 当前 tail_ptr（DMA硬件自动更新）
        lw      t2, 0x10(t0)          # t2 = 单个子缓冲区大小 dma_size

        la      t3, last_tail_ptr     # t3 = last_tail_ptr 变量地址
        lw      t4, 0(t3)             # t4 = 上一次中断的尾指针

        sub     t4, t1, t4            # t4 = 尾指针增量 = 当前值 - 历史值

        # RV32I 无除法指令，用循环减法计算 增量 / 单缓冲区大小 = 完成的缓冲区数
        li      t5, 0                  # t5 = 完成的缓冲区计数，初始为0
div_loop:
        bltu    t4, t2, div_done      # 剩余增量 < 单缓冲区大小，除法结束
        sub     t4, t4, t2            # 增量减去一个缓冲区大小
        addi    t5, t5, 1             # 计数+1
        j       div_loop
div_done:
        # t5 中保存了本次中断累计完成的子缓冲区数量

        # ========== 4. 更新历史尾指针 ==========
        sw      t1, 0(t3)             # last_tail_ptr = 当前 tail_ptr，为下一次中断做准备

        # ========== 5. 更新全局子缓冲区计数 dma_buf_stat ==========
        la      t3, dma_buf_stat      # t3 = dma_buf_stat 全局变量地址
        lw      t1, 0(t3)             # t1 = 当前剩余待处理计数
        sub     t1, t1, t5            # 减去本次完成的数量
        sw      t1, 0(t3)             # 写回全局变量

        # ========== 6. 恢复寄存器现场 ==========
        lw      t0, 0(sp)
        lw      t1, 4(sp)
        lw      t2, 8(sp)
        lw      t3, 12(sp)
        lw      t4, 16(sp)
        lw      t5, 20(sp)
        addi    sp, sp, 32

        # ========== 7. 中断返回 ==========
        # 硬件自动恢复 PC 为 mepc，同时重新打开全局中断使能
        mret