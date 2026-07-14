.globl global_result
.globl dma_buf_stat
.globl start
.globl _SP_START_
.type start, @function
.section .start
.align 2        # 强制4字节对齐，符合RISC-V指令规范

start:
        # 临时设置栈指针（保留原模板逻辑）
        li      sp, 0x4000
        # 跳过数据区，防止数据被当作指令执行
        j       continued

.org 0x0c       # 显式固定global_result地址为0x0c，对齐原模板布局
global_result:
        .word   0xffffffff

.org 0x10       # 显式固定dma_buf_stat地址为0x10，匹配中断服务程序硬编码访问
dma_buf_stat:
        .word   0x0

continued:
        # 初始化栈指针为链接脚本定义的栈顶地址
        la      sp, _SP_START_
        # 调用主函数
        call    main
        # 主函数返回后进入结束陷阱
        call    hit_good_trap

        # 异常兜底：防止陷阱函数返回后程序跑飞
1:
        j       1b