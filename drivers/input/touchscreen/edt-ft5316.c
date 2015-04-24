/*
 * Copyright (C) 2014, bbv Software Services AG, Zurich, Switzerland, <adam.szalkowski@bbv.ch>
 *
 * based on edt-ft5316 driver
 * Copyright (C) 2012 Simon Budig, <simon.budig@kernelconcepts.de>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 * This is a driver for the EDT "Polytouch" family of touch controllers
 * based on the FocalTech FT5316 line of chips.
  *
 * Development of this driver has been sponsored by bbv Software Services:
 *    http://www.bbv.ch/
*/

#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/input/edt-ft5316.h>
#include <linux/pinctrl/consumer.h>

#define MAX_SUPPORT_POINTS		5

#define REGISTER_OPMODE			0x00
#define REGISTER_POINTS			0x02
#define REGISTER_EVENT_OFFSET		0x03
#define REGISTER_EVENT_LENGTH		0x06
#define REGISTER_FIRMWARE_VERSION	0xa6
#define REGISTER_THRESHOLD		0x80
#define REGISTER_PEAK_THRESHOLD		0x81

#define DEVICE_MODE_NORMAL		0x00
#define DEVICE_MODE_SYSINFO		0x10
#define DEVICE_MODE_RAW			0x40

#define TOUCH_EVENT_DOWN		0x00
#define TOUCH_EVENT_UP			0x01
#define TOUCH_EVENT_ON			0x02
#define TOUCH_EVENT_RESERVED		0x03

#define EDT_NAME_LEN			20

struct edt_ft5316_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	u32 max_x;
	u32 max_y;

	int reset_pin;
	int irq_pin;

	u8 active_ids;

#if defined(CONFIG_DEBUG_FS)
	struct dentry *debug_dir;
#endif

	struct mutex mutex;
	int threshold;
	int peak_threshold;

	char name[EDT_NAME_LEN];
};

static int edt_ft5316_read_block(struct i2c_client *client,
				   u8 addr, u8 length, u8 *buffer)
{
	int ret;
	struct i2c_msg wrmsg[2];

	wrmsg[0].addr  = client->addr;
	wrmsg[0].flags = 0;
	wrmsg[0].len = 1;
	wrmsg[0].buf = &addr;

	wrmsg[1].addr  = client->addr;
	wrmsg[1].flags = I2C_M_RD;
	wrmsg[1].len = length;
	wrmsg[1].buf = buffer;

	ret = i2c_transfer(client->adapter, wrmsg, 2);
	if (ret < 0)
		return ret;
	if (ret != 2)
		return -EIO;

	return 0;
}

static int __maybe_unused edt_ft5316_register_write(struct i2c_client *client,
				     u8 addr, u8 value)
{
	return i2c_smbus_write_byte_data(client, addr, value);
}

static int edt_ft5316_register_read(struct i2c_client *client,
				    u8 addr)
{
	return i2c_smbus_read_byte_data(client, addr);
}

static irqreturn_t edt_ft5316_ts_isr(int irq, void *dev_id)
{
	struct edt_ft5316_ts_data *tsdata = dev_id;
	struct device *dev = &tsdata->client->dev;
	u8 rdbuf[32];
	int i, error;
	u8 points;
	u8 current_active_ids = 0;
	u8 released_ids = 0;

	points = edt_ft5316_register_read(tsdata->client, REGISTER_POINTS) & 0x0f;
	dev_dbg(dev, "Number of touch points: %d\n",
		points);

	if(points < 0) {
		dev_err_ratelimited(dev, "Unable to fetch register, error: %d\n",
				    error);
		goto out;
	}
	if(points > MAX_SUPPORT_POINTS) {
		points = MAX_SUPPORT_POINTS;
	}

	if(points > 0) {
		error = edt_ft5316_read_block(tsdata->client,
					      REGISTER_EVENT_OFFSET, points*REGISTER_EVENT_LENGTH, rdbuf);
		if (error) {
			dev_err_ratelimited(dev, "Unable to fetch data, error: %d\n",
					    error);
			goto out;
		}
	}

	for (i = 0; i < points; i++) {
		u8 *buf = &rdbuf[i * 6];
		u32 x, y;
		u8 id, type;

		x = ((buf[0] << 8) | buf[1]) & 0x0fff;
		y = ((buf[2] << 8) | buf[3]) & 0x0fff;
		id = (buf[2] >> 4) & 0x0f;
		type = (buf[0] >> 6) & 0xf;

		if(id >= MAX_SUPPORT_POINTS) {
			dev_warn(dev, "id out of range: %d\n", id);
			id = MAX_SUPPORT_POINTS - 1;
		}

		/* ignore Reserved events */
		if (type == TOUCH_EVENT_RESERVED)
			continue;

		dev_dbg(dev, "x: %d, y: %d, id: %d\n",
			x, y, id);

		if (type == TOUCH_EVENT_UP)
			continue;

		current_active_ids |= (1 << id);

		input_mt_slot(tsdata->input, id);
		input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, 1);

		input_report_abs(tsdata->input, ABS_MT_POSITION_X, x);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y, y);
	}
	
	/* clear released points */
	released_ids = tsdata->active_ids & ~current_active_ids;
	for (i = 0; i < MAX_SUPPORT_POINTS; i++) {
		if(released_ids & (1 << i)) {
			dev_dbg(dev, "released id: %d\n", i);
			input_mt_slot(tsdata->input, i);
			input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, 0);
		}
	}
	tsdata->active_ids = current_active_ids;

	input_mt_report_pointer_emulation(tsdata->input, true);
	input_sync(tsdata->input);

out:
	return IRQ_HANDLED;
}

static int edt_ft5316_ts_reset(struct i2c_client *client,
			struct edt_ft5316_ts_data *tsdata)
{
	int error;

	if (gpio_is_valid(tsdata->reset_pin)) {
		/* this pulls reset down, enabling the low active reset */
		error = devm_gpio_request_one(&client->dev, tsdata->reset_pin, GPIOF_OUT_INIT_LOW,
					 "edt-ft5316 reset");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as reset pin, error %d\n",
				tsdata->reset_pin, error);
			return error;
		}

		msleep(5);
		gpio_set_value(tsdata->reset_pin, 1);
		msleep(300);
	}

	return 0;
}

static int edt_ft5316_ts_identify(struct i2c_client *client,
					    struct edt_ft5316_ts_data *tsdata,
					    char *fw_version)
{
	int reg;

	strcpy(tsdata->name, "ft5316");
	reg = edt_ft5316_register_read(client, REGISTER_FIRMWARE_VERSION);
	if(reg < 0 || reg > 255) {
		return -ENODEV;
	}
	snprintf(fw_version, 3, "%x", reg);
	fw_version[2] = 0;

	return 0;
}

#ifdef CONFIG_OF
static int edt_ft5316_i2c_ts_probe_dt(struct device *dev,
				struct edt_ft5316_ts_data *tsdata)
{
	int ret, error;
	struct device_node *np = dev->of_node;
	struct pinctrl *pinctrl;
	enum of_gpio_flags gpio_flags;
	u32 resolution[2];

	if (!np)
		return -ENODEV;

	/*
	 * irq_pin is not needed for DT setup.
	 * irq is associated via 'interrupts' property in DT
	 */
	tsdata->irq_pin = -EINVAL;

	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "unable to select pin group\n");

	ret = of_get_named_gpio_flags(np, "reset-gpios", 0, &gpio_flags);
	tsdata->reset_pin = ret;

	error = of_property_read_u32_array(np, "resolution", resolution, 2);
	if(error) {
		dev_warn(dev, "missing resolution in device tree node\n");
		return error;
	}
	tsdata->max_x = resolution[0];
	tsdata->max_y = resolution[1];

	return 0;
}
#else
static inline int edt_ft5316_i2c_ts_probe_dt(struct device *dev,
					struct edt_ft5316_ts_data *tsdata)
{
	return -ENODEV;
}
#endif

static struct input_dev* edt_ft5316_init_input(struct edt_ft5316_ts_data *tsdata)
{
	struct input_dev *input;
	int error;

	input = devm_input_allocate_device(&tsdata->client->dev);
	input->name = tsdata->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &tsdata->client->dev;

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_X, 0, tsdata->max_x - 1, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, tsdata->max_y - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X,
			     0, tsdata->max_x - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
			     0, tsdata->max_y - 1, 0, 0);

	error = input_mt_init_slots(input, MAX_SUPPORT_POINTS, 0);
	if (error) {
		dev_err(&tsdata->client->dev, "Unable to init MT slots.\n");
		return NULL;
	}

	input_set_drvdata(input, tsdata);
	error = input_register_device(input);
	if (error) {
		dev_err(&tsdata->client->dev, "Unable to register input device.\n");
		return NULL;
	}

	return input;
}

static int edt_ft5316_init_irq(struct edt_ft5316_ts_data *tsdata)
{
	int error;

	error = devm_request_threaded_irq(&tsdata->client->dev, tsdata->client->irq, NULL, edt_ft5316_ts_isr,
				     IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				     tsdata->client->name, tsdata);
	return error;
}


static int edt_ft5316_init(struct edt_ft5316_ts_data *tsdata)
{
	int error;

	tsdata->active_ids = 0;
	mutex_init(&tsdata->mutex);
	tsdata->input = edt_ft5316_init_input(tsdata);
	if (!tsdata->input) {
		dev_err(&tsdata->client->dev, "failed to allocate input device.\n");
		return -ENOMEM;
	}

	error = edt_ft5316_init_irq(tsdata);
	if (error) {
		dev_err(&tsdata->client->dev, "Unable to request touchscreen IRQ.\n");
		return error;
	}

	device_init_wakeup(&tsdata->client->dev, 1);
	return 0;
}

static int edt_ft5316_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	const struct edt_ft5316_platform_data *pdata =
				dev_get_platdata(&client->dev);
	struct edt_ft5316_ts_data *tsdata;
	char fw_version[3];
	int error;

	dev_dbg(&client->dev, "probing for EDT FT5316 I2C\n");

	tsdata = devm_kzalloc(&client->dev, sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err(&client->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}

	tsdata->client = client;
	i2c_set_clientdata(client, tsdata);

	if (!pdata) {
		error = edt_ft5316_i2c_ts_probe_dt(&client->dev, tsdata);
		if (error) {
			dev_err(&client->dev,
				"DT probe failed and no platform data present\n");
			return error;
		}
	} else {
		tsdata->reset_pin = pdata->reset_pin;
		tsdata->irq_pin = pdata->irq_pin;
		tsdata->max_x = pdata->max_x;
		tsdata->max_y = pdata->max_y;
	}

	error = edt_ft5316_ts_reset(client, tsdata);
	if (error)
		return error;

	if (gpio_is_valid(tsdata->irq_pin)) {
		error = devm_gpio_request_one(&client->dev, tsdata->irq_pin,
					 GPIOF_IN, "edt-ft5316 irq");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d, error %d\n",
				tsdata->irq_pin, error);
			return error;
		}
	}

	error = edt_ft5316_ts_identify(client, tsdata, fw_version);
	if (error) {
		dev_err(&client->dev, "touchscreen probe failed\n");
		return error;
	}

	dev_dbg(&client->dev,
		"Model \"%s\", Rev. \"%s\", %dx%d sensors\n",
		tsdata->name, fw_version, tsdata->max_x, tsdata->max_y);


	error = edt_ft5316_init(tsdata);
	if (error) {
		dev_err(&client->dev, "touchscreen initialization failed\n");
		return error;
	}

	dev_dbg(&client->dev,
		"EDT FT5316 initialized: IRQ %d, Reset pin %d.\n",
		client->irq, tsdata->reset_pin);

	return 0;
}

static int edt_ft5316_ts_remove(struct i2c_client *client)
{
	return 0;
}

static int __maybe_unused edt_ft5316_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int __maybe_unused edt_ft5316_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(edt_ft5316_ts_pm_ops,
			 edt_ft5316_ts_suspend, edt_ft5316_ts_resume);

static const struct i2c_device_id edt_ft5316_ts_id[] = {
	{ "edt-ft5316", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, edt_ft5316_ts_id);

#ifdef CONFIG_OF
static struct of_device_id edt_ft5316_of_match[] = {
	{ .compatible = "edt,ft5316" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, edt_ft5316_of_match);
#endif

static struct i2c_driver edt_ft5316_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "edt_ft5316",
		.of_match_table = of_match_ptr(edt_ft5316_of_match),
		.pm = &edt_ft5316_ts_pm_ops,
	},
	.id_table = edt_ft5316_ts_id,
	.probe    = edt_ft5316_ts_probe,
	.remove   = edt_ft5316_ts_remove,
};

module_i2c_driver(edt_ft5316_ts_driver);

MODULE_AUTHOR("Adam Szalkowski <adam.szalkowski@bbv.ch>");
MODULE_DESCRIPTION("EDT FT5316 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
