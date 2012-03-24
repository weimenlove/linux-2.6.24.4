/*
 ================================================
 Name        : my2410_pwm.c
 Author      : Huang Gang
 Date        : 25/11/09
 Copyright   : GPL
 Description : my2410 pwm driver
 ================================================
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/regs-gpio.h>
#include <asm/plat-s3c/regs-timer.h>
#define PWM_MAJOR 0                  //主设备号
#define PWM_NAME "my2410_pwm"        //设备名称
//#define PWM_IRQ IRQ_TIMER0 	     //S3C2410_IRQ(10)

static int device_major = PWM_MAJOR; //系统动态生成的主设备号

#if 0
static irqreturn_t isr_pwm(int irq,void *dev_id,struct pt_regs *regs)
{
	disable_irq(10);  //禁止中断

	enable_irq(10);  //使能中断
	return 0;
}
#endif

//打开设备
static int pwm_open(struct inode *inode, struct file *file)
{
	    //对GPB0复用口进行复用功能设置，设置为TOUT0 PWM输出
	s3c2410_gpio_cfgpin(S3C2410_GPB0, S3C2410_GPB0_TOUT0);
	return 0;
}

//关闭设备
static int pwm_close(struct inode *inode, struct file *file)
{
	return 0;
}

//对设备进行控制
static int pwm_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	if(cmd <= 0)//如果输入的参数小于或等于0的话，就让蜂鸣器停止工作
	{
	//这里又恢复GPB0口为IO口输出功能，由原理图可知直接给低电平可让蜂鸣器停止工作
		s3c2410_gpio_cfgpin(S3C2410_GPB0, S3C2410_GPB0_OUTP);
		s3c2410_gpio_setpin(S3C2410_GPB0, 0);
	}
	else//如果输入的参数大于0，就让蜂鸣器开始工作，不同的参数，蜂鸣器的频率也不一样
	{
		//定义一些局部变量
		unsigned long tcon;
		unsigned long tcnt;
		unsigned long tcmp;
		unsigned long tcfg1;
		unsigned long tcfg0;
		struct clk *clk_p;
		unsigned long pclk;

		tcnt = 0xffff;  /* default value for tcnt */

		tcon = __raw_readl(S3C2410_TCON);
		tcfg0 = __raw_readl(S3C2410_TCFG0);     //读取定时器配置寄存器0的值
		tcfg1 = __raw_readl(S3C2410_TCFG1);     //读取定时器配置寄存器1的值
		//以下对各寄存器的操作结合上面讲的开始一个PWM定时器的步骤和2440手册PWM寄存器操作部分来看就比较容易理解
		tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK; //S3C2410_TCFG_PRESCALER0_MASK=(255<<0)，将低八位清零
		tcfg0 |= (10 - 1);                      //设置tcfg0(0~255)的值为10
		__raw_writel(tcfg0, S3C2410_TCFG0);     //将值tcfg0写入定时器配置寄存器0中

		tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;	//S3C2410_TCFG1_MUX0_MASK=(15<<0)
		tcfg1 |= S3C2410_TCFG1_MUX0_DIV2;      //设置tcfg1的值为0x0000即：1/2
		__raw_writel(tcfg1, S3C2410_TCFG1);     //将值tcfg1写入定时器配置寄存器1中

		clk_p = clk_get(NULL, "pclk");
		pclk = clk_get_rate(clk_p);   //从系统平台时钟队列中获取pclk的时钟频率，在include/linux/clk.h中定义
		//printk("pclk=%ld\n",pclk); //add by fantity 2011-5-19
		//计算定时器0的输出时钟频率(pclk/{prescaler0 + 1}/divider value)
		tcnt = (pclk/10/2/50);			//20ms
		tcmp = tcnt*(cmd+4)/200;
		__raw_writel(tcnt, S3C2410_TCNTB(0));   //设置定时器0计数缓存寄存器的值
		__raw_writel(tcmp, S3C2410_TCMPB(0)); //设置定时器0比较缓存寄存器的值

		tcon = __raw_readl(S3C2410_TCON);       //读取定时器控制寄存器的值
		tcon &= ~0x1f;
		tcon |= 0xb;  //关闭死区、自动重载、关反相器、更新TCNTB0&TCMPB0、启动定时器0
		__raw_writel(tcon, S3C2410_TCON);  //设置定时器控制寄存器的0-4位，即对定时器0进行控制
		tcon &= ~2;
		__raw_writel(tcon, S3C2410_TCON); //清除定时器0的手动更新位
	}
	return 0;
}

//设备操作结构体
static struct file_operations pwm_fops =
{
	    .owner   = THIS_MODULE,
	    .open    = pwm_open,
	    .release = pwm_close,
	    .ioctl   = pwm_ioctl,
};

//定义一个设备类
static struct class *pwm_class;

static int __init pwm_init(void)
{
#if 0			//for irq
	int ret;
	ret=request_irq(PWM_IRQ,isr_pwm,IRQF_SAMPLE_RANDOM,DEVICE_NAME,NULL); //申请中断
	if(ret) {//申请失败
		printk("PWM_IRQ: could not register interrupt\n");
		return ret;
	}
#endif
	//注册为字符设备，主设备号为0让系统自动分配，设备名为my2410_pwm，注册成功返回动态生成的主设备号
	device_major = register_chrdev(PWM_MAJOR, PWM_NAME, &pwm_fops);

	if(device_major < 0){
		printk(PWM_NAME " register falid!\n");
		return device_major;
	}

	//注册一个设备类，使mdev可以在/dev/目录下自动建立设备节点
	pwm_class = class_create(THIS_MODULE, PWM_NAME);

	if(IS_ERR(pwm_class)){
		printk(PWM_NAME " register class falid!\n");
		return -1;
	}

	//创建一个设备节点，设备名为PWM_NAME，即：my2410_pwm
	class_device_create(pwm_class, NULL,MKDEV(device_major,0),NULL,PWM_NAME);
	printk(PWM_NAME " initialnation\n");
	return 0;
}

static void __exit pwm_exit(void)
{
	    //注销设备
	    unregister_chrdev(device_major, PWM_NAME);

	    //删除设备节点
	    class_device_destroy(pwm_class, MKDEV(device_major, 0));

	    //注销设备类
	    class_destroy(pwm_class);

	   // free_irq(PWM_IRQ,NULL); //释放中断
	    printk(PWM_NAME " release\n");
}

module_init(pwm_init);
module_exit(pwm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Huang Gang");
MODULE_DESCRIPTION("my2410 pwm driver");
