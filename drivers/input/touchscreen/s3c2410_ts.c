/* By dean adapted from original driver of linux-2.6.13.4 for mini2440 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/arch/irqs.h>


#include <asm/arch/regs-adc.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/ts.h>

/* For ts.dev.id.version */
#define S3C2410TSVERSION 0x0101
#define TSC_SLEEP (S3C2410_ADCTSC_PULL_UP_DISABLE | S3C2410_ADCTSC_XY_PST(0))
#define WAIT4INT(x) (((x)<<8) | S3C2410_ADCTSC_YM_SEN | S3C2410_ADCTSC_YP_SEN | \
		S3C2410_ADCTSC_XP_SEN | S3C2410_ADCTSC_XY_PST(3))

#define AUTOPST (S3C2410_ADCTSC_YM_SEN | S3C2410_ADCTSC_YP_SEN | S3C2410_ADCTSC_XP_SEN |\
		S3C2410_ADCTSC_AUTO_PST | S3C2410_ADCTSC_XY_PST(0))
#define DEBUG_LVL KERN_DEBUG

MODULE_AUTHOR("Arnaud Patard <arnaud.patard@rtp-net.org>");
MODULE_DESCRIPTION("s3c2410 touchscreen driver");
MODULE_LICENSE("GPL");

/*
   * Definitions & global arrays.
   */


static char *s3c2410ts_name = "s3c2410 TouchScreen";

/*
   * Per-touchscreen data.
   */

struct s3c2410ts {
	struct input_dev *dev;
	long xp;
	long yp;
	int count;
	int shift;
	char phys[32];
};

static struct s3c2410ts ts;
static void __iomem *base_addr;
static inline void s3c2410_ts_connect(void)
{
	s3c2410_gpio_cfgpin(S3C2410_GPG12, S3C2410_GPG12_XMON);
	s3c2410_gpio_cfgpin(S3C2410_GPG13, S3C2410_GPG13_nXPON);
	s3c2410_gpio_cfgpin(S3C2410_GPG14, S3C2410_GPG14_YMON);
	s3c2410_gpio_cfgpin(S3C2410_GPG15, S3C2410_GPG15_nYPON);
}

static void touch_timer_fire(unsigned long data)
{
	unsigned long data0;
	unsigned long data1;
	int updown;
	//printk("touch_timer_fire 1:%d\n",updown);

	data0 = readl(base_addr+S3C2410_ADCDAT0);
	data1 = readl(base_addr+S3C2410_ADCDAT1);
	//printk("touch time fire: 2:data0:%d:data1:%d.\n",data0,data1);
	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) && (!(data1 & S3C2410_ADCDAT0_UPDOWN));
	//printk("touch_timer_fire 3:%d\n",updown);
	if (updown)
	{
		if (ts.count != 0)
		{
			long tmp;
			tmp = ts.xp;
			ts.xp = ts.yp;
			ts.yp = tmp;
			//printk("touch time fire:4:xp:%d:yp:%d.\n",ts.xp,ts.yp);
			ts.xp >>= ts.shift;
			ts.yp >>= ts.shift;
			//printk("func 2");
#ifdef CONFIG_TOUCHSCREEN_S3C2410_DEBUG
			{
				struct timeval tv;
				do_gettimeofday(&tv);
				printk(DEBUG_LVL "T: %06d, X: %03ld, Y: %03ld\n", (int)tv.tv_usec, ts.xp, ts.yp);
			}
#endif

			input_report_abs(ts.dev, ABS_X, ts.xp);
			input_report_abs(ts.dev, ABS_Y, ts.yp);
			//printk("touch time fire:reportsbs:xp:%d:yp:%d.\n",ts.xp,ts.yp);
			input_report_key(ts.dev, BTN_TOUCH, 1);
			input_report_abs(ts.dev, ABS_PRESSURE, 1);
			input_sync(ts.dev);
		}

		ts.xp = 0;
		ts.yp = 0;
		ts.count = 0;
		//printk("func 1");

		writel(S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST, base_addr+S3C2410_ADCTSC);
		writel(readl(base_addr+S3C2410_ADCCON) | S3C2410_ADCCON_ENABLE_START, base_addr+S3C2410_ADCCON);
	}
	else
	{
		ts.count = 0;
		//printk("func 3");
		input_report_key(ts.dev, BTN_TOUCH, 0);
		input_report_abs(ts.dev, ABS_PRESSURE, 0);
		input_sync(ts.dev);

		writel(WAIT4INT(0), base_addr+S3C2410_ADCTSC);
	}
}

static struct timer_list touch_timer =
TIMER_INITIALIZER(touch_timer_fire, 0, 0);

static irqreturn_t stylus_updown(int irq, void *dev_id)
{
	unsigned long data0;
	unsigned long data1;
	int updown;
	data0 = readl(base_addr+S3C2410_ADCDAT0);
	data1 = readl(base_addr+S3C2410_ADCDAT1);
	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) && (!(data1 & S3C2410_ADCDAT0_UPDOWN));

	/* TODO we should never get an interrupt with updown set while
	   * the timer is running, but maybe we ought to verify that the
	   * timer isn't running anyways. */
	if (updown)
		touch_timer_fire(0);

	return IRQ_HANDLED;
}


static irqreturn_t stylus_action(int irq, void *dev_id)
{
	//printk("action\n");
	unsigned long data0;
	unsigned long data1;

	data0 = readl(base_addr+S3C2410_ADCDAT0);
	data1 = readl(base_addr+S3C2410_ADCDAT1);

	ts.xp += data0 & S3C2410_ADCDAT0_XPDATA_MASK;
	ts.yp += data1 & S3C2410_ADCDAT1_YPDATA_MASK;
	ts.count++;

	if (ts.count < (1<<ts.shift)) {
		writel(S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST, base_addr+S3C2410_ADCTSC);
		writel(readl(base_addr+S3C2410_ADCCON) | S3C2410_ADCCON_ENABLE_START, base_addr+S3C2410_ADCCON);
	} else {
		mod_timer(&touch_timer, jiffies+1);
		writel(WAIT4INT(1), base_addr+S3C2410_ADCTSC);
	}

	return IRQ_HANDLED;
}
static struct clk *adc_clock;

static int __init s3c2410ts_probe(struct platform_device *pdev)
{
	int rc;
	struct s3c2410_ts_mach_info *info;
	struct input_dev *input_dev;

	info = ( struct s3c2410_ts_mach_info *)pdev->dev.platform_data;

	if (!info)
	{
		printk(KERN_ERR "Hm... too bad : no platform data for ts\n");
		return -EINVAL;
	}

#ifdef CONFIG_TOUCHSCREEN_S3C2410_DEBUG
	printk(DEBUG_LVL "Entering s3c2410ts_init\n");
#endif

	adc_clock = clk_get(NULL, "adc");
	if (!adc_clock) {
		printk(KERN_ERR "failed to get adc clock source\n");
		return -ENOENT;
	}
	clk_enable(adc_clock);

#ifdef CONFIG_TOUCHSCREEN_S3C2410_DEBUG
	printk(DEBUG_LVL "got and enabled clock\n");
#endif

	base_addr=ioremap(S3C2410_PA_ADC,0x20);
	if (base_addr == NULL) {
		printk(KERN_ERR "Failed to remap register block\n");
		return -ENOMEM;
	}


	/* Configure GPIOs */
	s3c2410_ts_connect();

	if ((info->presc&0xff) > 0)
		writel(S3C2410_ADCCON_PRSCEN | S3C2410_ADCCON_PRSCVL(info->presc&0xFF),\
				base_addr+S3C2410_ADCCON);
	else
		writel(0,base_addr+S3C2410_ADCCON);


	/* Initialise registers */
	if ((info->delay&0xffff) > 0)
		writel(info->delay & 0xffff, base_addr+S3C2410_ADCDLY);

	writel(WAIT4INT(0), base_addr+S3C2410_ADCTSC);

	/* Initialise input stuff */
	memset(&ts, 0, sizeof(struct s3c2410ts));
	input_dev = input_allocate_device();
	if (!input_dev) {
		printk(KERN_ERR "Unable to allocate the input device !!\n");
		return -ENOMEM;
	}
	sprintf(ts.phys, "ts0");
	ts.dev=input_dev;

	ts.dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts.dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(ts.dev, ABS_X, 0, 0x3FF, 0, 0);
	input_set_abs_params(ts.dev, ABS_Y, 0, 0x3FF, 0, 0);
	input_set_abs_params(ts.dev, ABS_PRESSURE, 0, 1, 0, 0);

	ts.dev->name = s3c2410ts_name;
	ts.dev->id.bustype = BUS_RS232;
	ts.dev->id.vendor = 0xDEAD;
	ts.dev->id.product = 0xBEEF;
	ts.dev->id.version = S3C2410TSVERSION;

	ts.shift = info->oversampling_shift;

	/* Get irqs */
	if (request_irq(IRQ_ADC, stylus_action,IRQF_SAMPLE_RANDOM, "s3c2410_action", ts.dev))
	{
		printk(KERN_ERR "s3c2410_ts.c: Could not allocate ts IRQ_ADC !\n");
		iounmap(base_addr);
		return -EIO;
	}
	if (request_irq(IRQ_TC, stylus_updown,IRQF_SAMPLE_RANDOM, "s3c2410_action", ts.dev))
	{
		printk(KERN_ERR "s3c2410_ts.c: Could not allocate ts IRQ_TC !\n");
		iounmap(base_addr);
		return -EIO;
	}

	printk(KERN_INFO "%s successfully loaded\n", s3c2410ts_name);

	/* All went ok, so register to the input system */
	#if 0 //del by weimen 2011-3-31
	rc=input_register_device(ts.dev);
	if (rc)
	{
		free_irq(IRQ_TC, ts.dev);
		free_irq(IRQ_ADC, ts.dev);
		clk_disable(adc_clock);
		iounmap(base_addr);
		return -EIO;
	}
	#endif //del end
	input_register_device(ts.dev);
	return 0;
}

static int s3c2410ts_remove(struct platform_device *pdev)
{
	disable_irq(IRQ_ADC);
	disable_irq(IRQ_TC);
	free_irq(IRQ_TC,ts.dev);
	free_irq(IRQ_ADC,ts.dev);

	if (adc_clock) {
		clk_disable(adc_clock);
		clk_put(adc_clock);
		adc_clock = NULL;
	}

	input_unregister_device(ts.dev);
	iounmap(base_addr);

	return 0;
}

#ifdef CONFIG_PM
static int s3c2410ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	writel(TSC_SLEEP, base_addr+S3C2410_ADCTSC);
	writel(readl(base_addr+S3C2410_ADCCON) | S3C2410_ADCCON_STDBM,base_addr+S3C2410_ADCCON);
	disable_irq(IRQ_ADC);
	disable_irq(IRQ_TC);
	clk_disable(adc_clock);
	return 0;
}

static int s3c2410ts_resume(struct platform_device *pdev)
{
	struct s3c2410_ts_mach_info *info =
		( struct s3c2410_ts_mach_info *)pdev->dev.platform_data;

	clk_enable(adc_clock);
	msleep(1);

	enable_irq(IRQ_ADC);
	enable_irq(IRQ_TC);

	if ((info->presc&0xff) > 0)
		writel(S3C2410_ADCCON_PRSCEN | S3C2410_ADCCON_PRSCVL(info->presc&0xFF),base_addr+S3C2410_ADCCON);
	else
		writel(0,base_addr+S3C2410_ADCCON);

	/* Initialise registers */
	if ((info->delay&0xffff) > 0)
		writel(info->delay & 0xffff, base_addr+S3C2410_ADCDLY);

	writel(WAIT4INT(0), base_addr+S3C2410_ADCTSC);

	return 0;
}

#else
#define s3c2410ts_suspend NULL
#define s3c2410ts_resume NULL
#endif


static struct platform_driver s3c2410ts_driver = {
	.driver ={
		.name = "s3c2410-ts",
		.owner = THIS_MODULE,
	},
	.probe = s3c2410ts_probe,
	.remove = s3c2410ts_remove,
	.suspend = s3c2410ts_suspend,
	.resume = s3c2410ts_resume,

};



int __init s3c2410ts_init(void)
{

	return platform_driver_register(&s3c2410ts_driver);
}

void __exit s3c2410ts_exit(void)
{

	platform_driver_unregister(&s3c2410ts_driver);
}

module_init(s3c2410ts_init);
module_exit(s3c2410ts_exit);
