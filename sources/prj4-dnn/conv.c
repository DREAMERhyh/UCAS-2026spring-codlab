#include "printf.h"
#include "trap.h"
#include "mul.h"
#include "div.h"
#include "perf_cnt.h"   // 性能计数器接口头文件

#define FRAC_BIT 10
#define RD_ADDR 135106448
#define RD_SIZE_D0 1
#define RD_SIZE_D1 1
#define RD_SIZE_D2 28
#define RD_SIZE_D3 28
#define WEIGHT_ADDR 134217728
#define WEIGHT_SIZE_D0 20
#define WEIGHT_SIZE_D1 1
#define WEIGHT_SIZE_D2 5
#define WEIGHT_SIZE_D3 5
#define WR_ADDR 135108240
#define WR_SIZE_D0 1
#define WR_SIZE_D1 20
#define WR_SIZE_D2 12
#define WR_SIZE_D3 12
#define KERN_ATTR_CONV_PAD 0
#define KERN_ATTR_CONV_STRIDE 1
#define KERN_ATTR_POOL_PAD 0
#define KERN_ATTR_POOL_KERN_SIZE 2
#define KERN_ATTR_POOL_STRIDE 2
//MMIO register address of DNN accelerator
#define GPIO_START_ADDR    0x60030000
#define GPIO_DONE_ADDR     0x60030008

struct size_vec4
{
	unsigned d0;
	unsigned d1;
	unsigned d2;
	unsigned d3;
};
struct mem_addr
{
	unsigned rd_addr;
	unsigned weight_addr;
	unsigned wr_addr;
};

int mul(short a, short b)
{
#ifndef USE_MUL
	int ans = mul_ll(a, b);
#else
	int ans = a * b;
#endif
	return ans;
}

struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR};
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3};
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3};
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3};
struct size_vec4 conv_size;

extern char _binary_data_result_bin_start[];
extern char _binary_data_result_bin_size[];

void convolution()
{
	short *in = (short *)addr.rd_addr;
	short *weight = (short *)addr.weight_addr;
	short *out = (short *)addr.wr_addr;
	unsigned output_offset = 0;
	unsigned input_offset = 0;
	unsigned input_fm_w = rd_size.d3;
	unsigned input_fm_h = rd_size.d2;
	unsigned pad = KERN_ATTR_CONV_PAD;
	unsigned pad_len = pad << 1;
	unsigned conv_out_w = rd_size.d3 - weight_size.d3 + pad_len;
	unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len;
	unsigned stride = KERN_ATTR_CONV_STRIDE;
	conv_out_w = div(conv_out_w, stride);
	conv_out_h = div(conv_out_h, stride);
	conv_out_w++;
	conv_out_h++;
	conv_size.d0 = wr_size.d0;
	conv_size.d1 = wr_size.d1;
	conv_size.d2 = conv_out_h;
	conv_size.d3 = conv_out_w;

	// 消除模板预留未使用变量的编译警告，保留原有声明
	(void)output_offset;
	(void)input_offset;

	// ========== 卷积核心算法实现 ==========
    // 数据格式：输入/权重/输出均为Q5.10定点数(1符号+5整数+10小数)
    // 防溢出策略：32位int存储Q10.20格式的中间乘加结果，全部累加完成后单次归一化
    // 精度控制：最终右移截断低位，无多次舍入误差

	// 遍历每个输出通道
	for (unsigned oc = 0; oc < conv_size.d1; oc++)
	{
		// 计算当前输出通道的权重基地址偏移
		// 每个卷积核布局：1个bias + 5x5个权重，共26个short
		unsigned weight_base = oc * (1 + weight_size.d2 * weight_size.d3);
		short bias = weight[weight_base + 0];

		// 遍历输出特征图的每一行
		for (unsigned oh = 0; oh < conv_size.d2; oh++)
		{
			// 遍历输出特征图的每一列
			for (unsigned ow = 0; ow < conv_size.d3; ow++)
			{
				// 32位累加器，存储Q10.20格式的乘加结果
				int acc = 0;

				// 遍历卷积核的每一行
				for (unsigned kh = 0; kh < weight_size.d2; kh++)
				{
					// 计算输入特征图对应的行坐标，考虑padding偏移
					int ih = (int)(oh * stride) + kh - (int)pad;

					// 遍历卷积核的每一列
					for (unsigned kw = 0; kw < weight_size.d3; kw++)
					{
						// 计算输入特征图对应的列坐标
						int iw = (int)(ow * stride) + kw - (int)pad;
						short pixel_val = 0;

						// 坐标在输入范围内则取像素值，否则为padding的0
						if (ih >= 0 && ih < (int)input_fm_h &&
							iw >= 0 && iw < (int)input_fm_w)
						{
							pixel_val = in[ih * input_fm_w + iw];
						}

						// 获取当前位置的权重值
						short w_val = weight[weight_base + 1 + kh * weight_size.d3 + kw];

						// 调用mul函数完成乘法，结果为Q10.20格式，累加到累加器
						acc += mul(pixel_val, w_val);
					}
				}

				// 叠加偏置：bias为Q5.10格式，左移FRAC_BIT转换为Q10.20后累加
				acc += (int)bias << FRAC_BIT;

				// 归一化：右移FRAC_BIT位，截断低位，转换回Q5.10格式
				acc >>= FRAC_BIT;

				// 饱和截断，防止结果超出16位有符号数范围
				if (acc > 32767)
					acc = 32767;
				else if (acc < -32768)
					acc = -32768;

				// 写入输出特征图，采用通道优先(NCHW)布局
				unsigned out_idx = oc * conv_size.d2 * conv_size.d3
								+ oh * conv_size.d3
								+ ow;
				out[out_idx] = (short)acc;
			}
		}
	}
}

void pooling()
{
	short *out = (short *)addr.wr_addr;
	unsigned output_offset = 0;
	unsigned input_offset = 0;
	unsigned input_fm_w = conv_size.d3;
	unsigned input_fm_h = conv_size.d2;
	unsigned pad = KERN_ATTR_POOL_PAD;
	unsigned pad_len = pad << 1;
	unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE;
	unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE;
	unsigned pool_out_w = pad_w_test + pad_len;
	unsigned pool_out_h = pad_h_test + pad_len;
	unsigned stride = KERN_ATTR_POOL_STRIDE;
	unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride);
	unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride);
	pool_out_w = div(pool_out_w, stride);
	pool_out_h = div(pool_out_h, stride);
	pool_out_w++;
	pool_out_h++;
	if ((!pad) && (pad_w_test_remain || pad_h_test_remain))
	{
		pool_out_w++;
		pool_out_h++;
	}

	// 消除模板预留未使用变量的编译警告，保留原有声明
	(void)output_offset;
	(void)input_offset;

	// ========== 2×2最大值池化核心实现 ==========
    // 池化方式：原地操作，输入为卷积输出特征图，输出直接覆盖原内存
    // 边界处理：仅对窗口内有效像素取最大值，超出边界部分直接忽略
    // 无算术运算，不存在溢出与精度损失问题

	// 遍历每个通道
	for (unsigned oc = 0; oc < conv_size.d1; oc++)
	{
		// 当前通道输入特征图的基地址偏移
		unsigned in_base = oc * input_fm_h * input_fm_w;
		// 当前通道输出特征图的基地址偏移
		unsigned out_base = oc * pool_out_h * pool_out_w;

		// 遍历池化输出的每一行
		for (unsigned ph = 0; ph < pool_out_h; ph++)
		{
			// 遍历池化输出的每一列
			for (unsigned pw = 0; pw < pool_out_w; pw++)
			{
				// 初始化为short最小值，确保所有有效像素都能参与比较
				short max_val = -32768;

				// 遍历池化窗口的每一行
				for (unsigned kh = 0; kh < KERN_ATTR_POOL_KERN_SIZE; kh++)
				{
					// 计算输入特征图对应的行坐标
					int ih = (int)(ph * stride) + kh - (int)pad;

					// 遍历池化窗口的每一列
					for (unsigned kw = 0; kw < KERN_ATTR_POOL_KERN_SIZE; kw++)
					{
						// 计算输入特征图对应的列坐标
						int iw = (int)(pw * stride) + kw - (int)pad;

						// 仅在坐标有效时读取像素并更新最大值
						if (ih >= 0 && ih < (int)input_fm_h &&
							iw >= 0 && iw < (int)input_fm_w)
						{
							short val = out[in_base + ih * input_fm_w + iw];
							if (val > max_val)
								max_val = val;
						}
						// 超出边界的像素直接忽略，不参与比较
					}
				}

				// 写入池化结果
				unsigned out_idx = out_base + ph * pool_out_w + pw;
				out[out_idx] = max_val;
			}
		}
	}
}

#ifdef USE_HW_ACCEL
void launch_hw_accel()
{
	// 使用volatile指针，防止编译器优化掉轮询操作
	volatile int *gpio_start = (volatile int *)(GPIO_START_ADDR);
	volatile int *gpio_done  = (volatile int *)(GPIO_DONE_ADDR);

	// 向START寄存器第0位写1，启动硬件加速器
	*gpio_start = 0x1;

	// 轮询DONE寄存器，直到第0位变为1，表示运算完成
	while ((*gpio_done & 0x1) == 0)
	{
		// 空转等待硬件完成
	}
}
#endif

int comparing()
{
	char *out = (char *)addr.wr_addr;
	char *result = (char *)_binary_data_result_bin_start;
#ifdef USE_HW_ACCEL
	int count = (int)_binary_data_result_bin_size + 
		    (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
	int count = (int)_binary_data_result_bin_size;
#endif
	for (int i = 0, j = 0; i < count; i++)
	{
#ifdef USE_HW_ACCEL
		int alignment = i & 0x0000001f;
		if (alignment >= (WR_SIZE_D3 << 1))
			continue;
#endif
		if (*(out + i) != *(result + j))
		{
			printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
			return 1;
		}
		j++;
	}
	printf("Passed!\n");
	return 0;
}

int main()
{
#ifdef USE_HW_ACCEL
	printf("Launching task...\n");
	launch_hw_accel();
#else
	Result conv_res, pool_res;

	printf("starting convolution\n");
	bench_prepare(&conv_res);
	convolution();
	bench_done(&conv_res);
	printf("convolution cycles: %lu\n", conv_res.msec);

	printf("starting pooling\n");
	bench_prepare(&pool_res);
	pooling();
	bench_done(&pool_res);
	printf("pooling cycles: %lu\n", pool_res.msec);
#endif
	int result = comparing();
	printf("benchmark finished\n");
	if (result == 0) {
		hit_good_trap();
	} else {
		nemu_assert(0);
	}
	return 0;
}