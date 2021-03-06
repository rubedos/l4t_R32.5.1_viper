/*
 * Driver for PCA9685 16-channel 12-bit PWM LED controller
 *
 * Copyright (C) 2013 Steffen Trumtrar <s.trumtrar@pengutronix.de>
 * Copyright (C) 2015 Clemens Gruber <clemens.gruber@pqgruber.com>
 *
 * based on the pwm-twl-led.c driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/interrupt.h>            // Required for the IRQ code
#include <linux/time.h>
#include <linux/kthread.h>

/*
 * Because the PCA9685 has only one prescaler per chip, changing the period of
 * one channel affects the period of all 16 PWM outputs!
 * However, the ratio between each configured duty cycle and the chip-wide
 * period remains constant, because the OFF time is set in proportion to the
 * counter range.
 */

#define PCA9685_MODE1		0x00
#define PCA9685_MODE2		0x01
#define PCA9685_SUBADDR1	0x02
#define PCA9685_SUBADDR2	0x03
#define PCA9685_SUBADDR3	0x04
#define PCA9685_ALLCALLADDR	0x05
#define PCA9685_LEDX_ON_L	0x06
#define PCA9685_LEDX_ON_H	0x07
#define PCA9685_LEDX_OFF_L	0x08
#define PCA9685_LEDX_OFF_H	0x09

#define PCA9685_ALL_LED_ON_L	0xFA
#define PCA9685_ALL_LED_ON_H	0xFB
#define PCA9685_ALL_LED_OFF_L	0xFC
#define PCA9685_ALL_LED_OFF_H	0xFD
#define PCA9685_PRESCALE	0xFE

#define PCA9685_PRESCALE_MIN	0x03	/* => max. frequency of 1526 Hz */
#define PCA9685_PRESCALE_MAX	0xFF	/* => min. frequency of 24 Hz */

#define PCA9685_COUNTER_RANGE	4096
#define PCA9685_DEFAULT_PERIOD	5000000	/* Default period_ns = 1/200 Hz */
#define PCA9685_OSC_CLOCK_MHZ	25	/* Internal oscillator with 25 MHz */

#define PCA9685_NUMREGS		0xFF
#define PCA9685_MAXCHAN		0

#define LED_FULL		(1 << 4)
#define MODE1_SLEEP		(1 << 4)
#define MODE2_INVRT		(1 << 4)
#define MODE2_OUTDRV		(1 << 2)

#define LED_N_ON_H(N)	(PCA9685_LEDX_ON_H + (4 * (N)))
#define LED_N_ON_L(N)	(PCA9685_LEDX_ON_L + (4 * (N)))
#define LED_N_OFF_H(N)	(PCA9685_LEDX_OFF_H + (4 * (N)))
#define LED_N_OFF_L(N)	(PCA9685_LEDX_OFF_L + (4 * (N)))

struct pca9685 {
	struct pwm_chip chip;
	struct regmap *regmap;
	int active_cnt;
	int duty_ns;
	int period_ns;
};

#if defined(CONFIG_PCA9685_ECON_CALIBRATION)

#define MAX_PWM_MODE		1				// Maximum PWM modes supported

static irq_handler_t calibration_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

int calibration_init(int);
void calibration_exit(void);

static int thread_calibrate_pwm(void *);
static int thread_finish_pwm_calib(void *);

static struct task_struct *st_calib_start;
static struct task_struct *st_calib_stop;

struct timespec ts;
struct regmap *g_regmap;

static unsigned int pwmCalibGpio;		// Gpio for PWM calibration from DTB file
static unsigned int irqNumber;

int prescaler_val;

static bool irq_status = true;

struct psc_mode {
	int pwm_low_limit;
	int pwm_high_limit;
	int psc_start;
};

static struct psc_mode pre_limit[MAX_PWM_MODE] = {
	{340,342,0xD0},	// 29.23Hz to 29.41Hz
};

static int psc_lookup[MAX_PWM_MODE] = {0};

static int pwm_mode;

#endif


static inline struct pca9685 *to_pca(struct pwm_chip *chip)
{
	return container_of(chip, struct pca9685, chip);
}

static int pca9685_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			      int duty_ns, int period_ns)
{
	struct pca9685 *pca = to_pca(chip);
	unsigned long long duty;
	unsigned int reg;
	int prescale;

	if (period_ns != pca->period_ns) {
		prescale = DIV_ROUND_CLOSEST(PCA9685_OSC_CLOCK_MHZ * period_ns,
					     PCA9685_COUNTER_RANGE * 1000) - 1;

		if (prescale >= PCA9685_PRESCALE_MIN &&
			prescale <= PCA9685_PRESCALE_MAX) {
			/* Put chip into sleep mode */
			regmap_update_bits(pca->regmap, PCA9685_MODE1,
					   MODE1_SLEEP, MODE1_SLEEP);

			/* Change the chip-wide output frequency */
			regmap_write(pca->regmap, PCA9685_PRESCALE, prescale);

			/* Wake the chip up */
			regmap_update_bits(pca->regmap, PCA9685_MODE1,
					   MODE1_SLEEP, 0x0);

			/* Wait 500us for the oscillator to be back up */
			udelay(500);

			pca->period_ns = period_ns;
		} else {
			dev_err(chip->dev,
				"prescaler not set: period out of bounds!\n");
			return -EINVAL;
		}
	}

	pca->duty_ns = duty_ns;

	if (duty_ns < 1) {
		if (pwm->hwpwm >= PCA9685_MAXCHAN)
			reg = PCA9685_ALL_LED_OFF_H;
		else
			reg = LED_N_OFF_H(pwm->hwpwm);

		regmap_write(pca->regmap, reg, LED_FULL);

		return 0;
	}

	if (duty_ns == period_ns) {
		/* Clear both OFF registers */
		if (pwm->hwpwm >= PCA9685_MAXCHAN)
			reg = PCA9685_ALL_LED_OFF_L;
		else
			reg = LED_N_OFF_L(pwm->hwpwm);

		regmap_write(pca->regmap, reg, 0x0);

		if (pwm->hwpwm >= PCA9685_MAXCHAN)
			reg = PCA9685_ALL_LED_OFF_H;
		else
			reg = LED_N_OFF_H(pwm->hwpwm);

		regmap_write(pca->regmap, reg, 0x0);

		/* Set the full ON bit */
		if (pwm->hwpwm >= PCA9685_MAXCHAN)
			reg = PCA9685_ALL_LED_ON_H;
		else
			reg = LED_N_ON_H(pwm->hwpwm);

		regmap_write(pca->regmap, reg, LED_FULL);

		return 0;
	}

	duty = PCA9685_COUNTER_RANGE * (unsigned long long)duty_ns;
	duty = DIV_ROUND_UP_ULL(duty, period_ns);

	if (pwm->hwpwm >= PCA9685_MAXCHAN)
		reg = PCA9685_ALL_LED_OFF_L;
	else
		reg = LED_N_OFF_L(pwm->hwpwm);

	regmap_write(pca->regmap, reg, (int)duty & 0xff);

	if (pwm->hwpwm >= PCA9685_MAXCHAN)
		reg = PCA9685_ALL_LED_OFF_H;
	else
		reg = LED_N_OFF_H(pwm->hwpwm);

	regmap_write(pca->regmap, reg, ((int)duty >> 8) & 0xf);

	/* Clear the full ON bit, otherwise the set OFF time has no effect */
	if (pwm->hwpwm >= PCA9685_MAXCHAN)
		reg = PCA9685_ALL_LED_ON_H;
	else
		reg = LED_N_ON_H(pwm->hwpwm);

	regmap_write(pca->regmap, reg, 0);
#if defined(CONFIG_PCA9685_ECON_CALIBRATION)
	g_regmap = pca->regmap;
	calibration_init(MAX_PWM_MODE - 1);
#endif
	return 0;
}

#if defined(CONFIG_PCA9685_ECON_CALIBRATION)
/** @brief PWM Calibration initialization function
 *  This function sets up the calibration GPIO, calibration threads and the IRQ
 *  @return returns 0 if successful
 */
int calibration_init(int num)
{
	int result = 0;

    //Create the PWM Calibration thread
    st_calib_start = kthread_create(thread_calibrate_pwm, NULL, "pwm_auto_calib_thread");
    if (st_calib_start)
        printk("Thread Created successfully\n");
    else
        printk(KERN_ERR "Thread creation failed\n");

    //Create the thread to finish PWM calibration
    st_calib_stop = kthread_create(thread_finish_pwm_calib, NULL, "pwm_stop_auto_calib_stop_thread");
    if (st_calib_stop)
        printk("PWM stop Thread Created successfully\n");
    else
        printk(KERN_ERR "PWM stop Thread creation failed\n");

	// GPIO validation
	if (!gpio_is_valid(pwmCalibGpio)){
	  printk(KERN_ERR "Invalid GPIO \n");
	  return -ENODEV;
	}
	gpio_request(pwmCalibGpio, "sysfs");       // Set up the pwmCalibGpio
	gpio_direction_input(pwmCalibGpio);        // Set the GPIO to be an input
	gpio_export(pwmCalibGpio, false);          // Causes gpio to appear in /sys/class/gpio
						// the bool argument prevents the direction from being changed

	irqNumber = gpio_to_irq(pwmCalibGpio);

	if(psc_lookup[num] == 0 )
		prescaler_val = pre_limit[num].psc_start;
	else
		prescaler_val = psc_lookup[num];

	pwm_mode = num;
	wake_up_process(st_calib_start);
        udelay(500);

	// This next call requests an interrupt line
	result = request_irq(irqNumber,
						(irq_handler_t) calibration_irq_handler,
						IRQF_TRIGGER_RISING,
						"pwm_auto_calib_handler",
						NULL);

	return result;
}
EXPORT_SYMBOL(calibration_init);

/** @brief Calibration GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the Calibration GPIO.
 */

static irq_handler_t calibration_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
	static unsigned int ignore_samples = 0;
	static int t0 = 0, t1 = 0;

	/* Ignoring Five Samples to stable the PWM Frequency */
	if(ignore_samples++ < 5) {
		getnstimeofday(&ts);
		t0 = (int) ts.tv_nsec/100000;
	} else {
		getnstimeofday(&ts);
		t1 =(int) ts.tv_nsec/100000;

	/* Change the prescalar value Based on PWM Frequency limit */
		if( !((t1-t0) >= pre_limit[0].pwm_low_limit &&
					(t1-t0) <= pre_limit[0].pwm_high_limit) ) {
			ignore_samples = 0;
			if( (t1-t0) <= pre_limit[0].pwm_high_limit) {
				wake_up_process(st_calib_start);
				udelay(500);
			} else {
				prescaler_val = prescaler_val - 2;
				wake_up_process(st_calib_start);
				udelay(500);
			}
	/* Calibrated PWM and Stop calibration process */
		} else {
			printk(KERN_INFO "PWM Calibrated.. \n");
			psc_lookup[pwm_mode] = prescaler_val - 1;
			wake_up_process(st_calib_stop);
			udelay(500);
		}
	}
	return (irq_handler_t) IRQ_HANDLED;
}
/** @brief Calibration thread
 *  This thread will write the appropriate Prescalar value
 *  to get the desired frequency output from the PWM Chip
 */
static int thread_calibrate_pwm(void *data)
{
	int ret = 0;
	irq_status = true;
	while (!kthread_should_stop()) {
		/* Put chip into sleep mode */
		ret = regmap_update_bits(g_regmap, PCA9685_MODE1,
				   MODE1_SLEEP, MODE1_SLEEP);

		/* Change the chip-wide output frequency */
		ret = regmap_write(g_regmap, PCA9685_PRESCALE, prescaler_val);

		/* Wake the chip up */
		ret = regmap_update_bits(g_regmap, PCA9685_MODE1,
				   MODE1_SLEEP, 0x0);

		/* Wait 500us for the oscillator to be back up */
		udelay(500);
		prescaler_val++;

		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}
	return 0;
}

static int thread_finish_pwm_calib(void *data){
	while (!kthread_should_stop()) {
		udelay(500);
		if (st_calib_start)	{
			kthread_stop(st_calib_start);
			put_task_struct(st_calib_start);
			st_calib_start = NULL;
		}
		if (irq_status == true) {
			free_irq(irqNumber, NULL);
			irq_status = false;
			do_exit(0);
		}
	}
	return 0;
}

 /** @brief PWM Calibration cleanup function
 *  This function releases the calibration GPIO, calibration threads and the IRQ
 */
void calibration_exit(void){
	if (st_calib_start)	{
		kthread_stop(st_calib_start);
		put_task_struct(st_calib_start);
		st_calib_start = NULL;
	}
	if (irq_status == true) {
		free_irq(irqNumber, NULL);
		irq_status = false;
	}
	gpio_unexport(pwmCalibGpio);
	gpio_free(pwmCalibGpio);
}
EXPORT_SYMBOL(calibration_exit);
#endif	// End of CONFIG_PCA9685_ECON_CALIBRATION

static int pca9685_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pca9685 *pca = to_pca(chip);
	unsigned int reg;

	/*
	 * The PWM subsystem does not support a pre-delay.
	 * So, set the ON-timeout to 0
	 */
	if (pwm->hwpwm >= PCA9685_MAXCHAN)
		reg = PCA9685_ALL_LED_ON_L;
	else
		reg = LED_N_ON_L(pwm->hwpwm);

	regmap_write(pca->regmap, reg, 0);

	if (pwm->hwpwm >= PCA9685_MAXCHAN)
		reg = PCA9685_ALL_LED_ON_H;
	else
		reg = LED_N_ON_H(pwm->hwpwm);

	regmap_write(pca->regmap, reg, 0);

	/*
	 * Clear the full-off bit.
	 * It has precedence over the others and must be off.
	 */
	if (pwm->hwpwm >= PCA9685_MAXCHAN)
		reg = PCA9685_ALL_LED_OFF_H;
	else
		reg = LED_N_OFF_H(pwm->hwpwm);

	regmap_update_bits(pca->regmap, reg, LED_FULL, 0x0);

	return 0;
}

static void pca9685_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pca9685 *pca = to_pca(chip);
	unsigned int reg;

	if (pwm->hwpwm >= PCA9685_MAXCHAN)
		reg = PCA9685_ALL_LED_OFF_H;
	else
		reg = LED_N_OFF_H(pwm->hwpwm);

	regmap_write(pca->regmap, reg, LED_FULL);

	/* Clear the LED_OFF counter. */
	if (pwm->hwpwm >= PCA9685_MAXCHAN)
		reg = PCA9685_ALL_LED_OFF_L;
	else
		reg = LED_N_OFF_L(pwm->hwpwm);

	regmap_write(pca->regmap, reg, 0x0);
}

static int pca9685_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pca9685 *pca = to_pca(chip);

	if (pca->active_cnt++ == 0)
		return regmap_update_bits(pca->regmap, PCA9685_MODE1,
					  MODE1_SLEEP, 0x0);

	return 0;
}

static void pca9685_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pca9685 *pca = to_pca(chip);

	if (--pca->active_cnt == 0)
		regmap_update_bits(pca->regmap, PCA9685_MODE1, MODE1_SLEEP,
				   MODE1_SLEEP);
}

static const struct pwm_ops pca9685_pwm_ops = {
	.enable = pca9685_pwm_enable,
	.disable = pca9685_pwm_disable,
	.config = pca9685_pwm_config,
	.request = pca9685_pwm_request,
	.free = pca9685_pwm_free,
	.owner = THIS_MODULE,
};

static const struct regmap_config pca9685_regmap_i2c_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = PCA9685_NUMREGS,
	.cache_type = REGCACHE_NONE,
};

static int pca9685_pwm_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct pca9685 *pca;
	int ret, duty_ns = -1, period_ns = -1;
 	int mode2;
#if defined(CONFIG_PCA9685_ECON_CALIBRATION)
	struct device_node *node = client->dev.of_node;
	pwmCalibGpio = of_get_named_gpio(node, "pwm-calib-gpio", 0);
#endif
	pca = devm_kzalloc(&client->dev, sizeof(*pca), GFP_KERNEL);
	if (!pca)
		return -ENOMEM;

	pca->regmap = devm_regmap_init_i2c(client, &pca9685_regmap_i2c_config);
	if (IS_ERR(pca->regmap)) {
		ret = PTR_ERR(pca->regmap);
		dev_err(&client->dev, "Failed to initialize register map: %d\n",
			ret);
		return ret;
	}
	pca->duty_ns = 0;
	pca->period_ns = PCA9685_DEFAULT_PERIOD;

	i2c_set_clientdata(client, pca);

	regmap_read(pca->regmap, PCA9685_MODE2, &mode2);

	if (device_property_read_bool(&client->dev, "invert"))
		mode2 |= MODE2_INVRT;
	else
		mode2 &= ~MODE2_INVRT;

	if (device_property_read_bool(&client->dev, "open-drain"))
		mode2 &= ~MODE2_OUTDRV;
	else
		mode2 |= MODE2_OUTDRV;

	regmap_write(pca->regmap, PCA9685_MODE2, mode2);

	/* clear all "full off" bits */
	regmap_write(pca->regmap, PCA9685_ALL_LED_OFF_L, 0);
	regmap_write(pca->regmap, PCA9685_ALL_LED_OFF_H, 0);

	pca->chip.ops = &pca9685_pwm_ops;
	/* add an extra channel for ALL_LED */
	pca->chip.npwm = PCA9685_MAXCHAN + 1;

	pca->chip.dev = &client->dev;
	pca->chip.base = -1;
	pca->chip.can_sleep = true;

	device_property_read_u32(&client->dev, "duty_ns", &duty_ns);
	device_property_read_u32(&client->dev, "period_ns", &period_ns);

	ret =  pwmchip_add(&pca->chip);
	if (!ret) {
		if ((duty_ns != -1) && (period_ns != -1)) {
			pca9685_pwm_config(&pca->chip, &pca->chip.pwms[pca->chip.npwm], duty_ns, period_ns);
		}
	}
	return ret;
}

static int pca9685_pwm_remove(struct i2c_client *client)
{
	struct pca9685 *pca = i2c_get_clientdata(client);

#if defined(CONFIG_PCA9685_ECON_CALIBRATION)
	calibration_exit();
#endif
	
	regmap_update_bits(pca->regmap, PCA9685_MODE1, MODE1_SLEEP,
			   MODE1_SLEEP);

	return pwmchip_remove(&pca->chip);
}

static const struct i2c_device_id pca9685_id[] = {
	{ "pca9685", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, pca9685_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id pca9685_acpi_ids[] = {
	{ "INT3492", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(acpi, pca9685_acpi_ids);
#endif

/*  #ifdef CONFIG_OF */
static const struct of_device_id pca9685_dt_ids[] = {
	{ .compatible = "nxp,pca9685-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pca9685_dt_ids);
/*  #endif */

static struct i2c_driver pca9685_i2c_driver = {
	.driver = {
		.name = "pca9685-pwm",
		.acpi_match_table = ACPI_PTR(pca9685_acpi_ids),
		.of_match_table = of_match_ptr(pca9685_dt_ids),
	},
	.probe = pca9685_pwm_probe,
	.remove = pca9685_pwm_remove,
	.id_table = pca9685_id,
};

module_i2c_driver(pca9685_i2c_driver);

MODULE_AUTHOR("Steffen Trumtrar <s.trumtrar@pengutronix.de>");
MODULE_DESCRIPTION("PWM driver for PCA9685");
MODULE_LICENSE("GPL");
