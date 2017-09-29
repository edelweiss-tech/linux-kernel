#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/pm.h>

enum I2C_REGS {
	R_ID1 = 0,
	R_ID2,
	R_ID3,
	R_ID4,
	R_SOFTOFF_RQ,
	R_PWROFF_RQ,
	R_PWRBTN_STATE,
	R_VERSION1,
	R_VERSION2,
	R_BOOTREASON,
	R_BOOTREASON_ARG,
	R_SCRATCH1,
	R_SCRATCH2,
	R_SCRATCH3,
	R_SCRATCH4,
	R_COUNT
};

#define BMC_ID1_VAL 0x49
#define BMC_ID2_VAL 0x54
#define BMC_ID3_VAL 0x58
#define BMC_ID4_VAL0 0x32
#define BMC_ID4_VAL1 0x2

#define POLL_JIFFIES 100

struct bmc_poll_data {
	struct i2c_client *c;
};

static struct i2c_driver mitx2_bmc_i2c_driver;
static struct input_dev *button_dev;
static struct bmc_poll_data poll_data;
static struct task_struct *polling_task;
static u8 bmc_proto_version[3];
static u8 bmc_bootreason[2];
static u8 bmc_scratch[4];
static const char input_name[] = "BMC input dev";
static u8 prev_ret;

void
bmc_pwroff_rq(void) {
	int ret = 0;

	dev_info(&poll_data.c->dev, "Write reg R_PWROFF_RQ\n");
	ret = i2c_smbus_write_byte_data(poll_data.c, R_PWROFF_RQ, 0x01);
	dev_info(&poll_data.c->dev, "ret: %i\n", ret);
}

int
pwroff_rq_poll_fn(void *data) {
	int ret;

	while (1) {
		if (kthread_should_stop())
			break;
		dev_dbg(&poll_data.c->dev, "Polling\n");
		ret = i2c_smbus_read_byte_data(poll_data.c, R_SOFTOFF_RQ);
		dev_dbg(&poll_data.c->dev, "Polling returned: %i\n", ret);
		if (prev_ret != ret) {
			dev_info(&poll_data.c->dev, "key change [%i]\n", ret);
			if (ret < 0) {
				dev_err(&poll_data.c->dev,
					"Could not read register %x\n",
					R_SOFTOFF_RQ);
				return -EIO;
			} else if (ret != 0) {
				dev_info(&poll_data.c->dev,
					 "PWROFF \"irq\" detected [%i]\n", ret);
				input_event(button_dev, EV_KEY, KEY_POWER, 1);
			} else {
				input_event(button_dev, EV_KEY, KEY_POWER, 0);
			}
			input_sync(button_dev);
		}
		prev_ret = ret;
		msleep_interruptible(100);
	}
	do_exit(1);
	return 0;
}

static int
mitx2_bmc_validate(struct i2c_client *client) {
	int ret = 0;
	int i = 0;
	static const u8 regs[] = {R_ID1, R_ID2, R_ID3};
	static const u8 vals[] = {BMC_ID1_VAL, BMC_ID2_VAL, BMC_ID3_VAL};

	bmc_proto_version[0] = 0;
	bmc_proto_version[1] = 0;
	bmc_proto_version[2] = 0;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = i2c_smbus_read_byte_data(client, regs[i]);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %x\n",
				regs[i]);
			return -EIO;
		}
		if (ret != vals[i]) {
			dev_err(&client->dev,
				"Bad value [0x%02x] in register 0x%02x, should be [0x%02x]\n",
				 ret, regs[i], vals[i]);

			return -ENODEV;
		}
	}
	ret = i2c_smbus_read_byte_data(client, R_ID4);
	if (ret < 0) {
		dev_err(&client->dev, "Could not read register %x\n", R_ID4);
		return -EIO;
	}
	if (ret == BMC_ID4_VAL0) {
		bmc_proto_version[0] = 0;
	} else if (ret == BMC_ID4_VAL1) {
		bmc_proto_version[0] = 2;
		ret = i2c_smbus_read_byte_data(client, R_VERSION1);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %x\n",
				R_VERSION1);
			return -EIO;
		}
		bmc_proto_version[1] = ret;
		ret = i2c_smbus_read_byte_data(client, R_VERSION2);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %x\n",
				R_VERSION2);
			return -EIO;
		}
		bmc_proto_version[2] = ret;
		ret = i2c_smbus_read_byte_data(client, R_BOOTREASON);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %x\n",
				R_BOOTREASON);
			return -EIO;
		}
		bmc_bootreason[0] = ret;
		dev_info(&client->dev, "BMC bootreason[0]->%i\n", ret);
		ret = i2c_smbus_read_byte_data(client, R_BOOTREASON_ARG);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %x\n",
				R_BOOTREASON_ARG);
			return -EIO;
		}
		bmc_bootreason[1] = ret;
		dev_info(&client->dev, "BMC bootreason[1]->%i\n", ret);
		for (i = R_SCRATCH1; i <= R_SCRATCH4; i++) {
			ret = i2c_smbus_read_byte_data(client, i);
			if (ret < 0) {
				dev_err(&client->dev,
					"Could not read register %x\n", i);
				return -EIO;
			}
			bmc_scratch[i - R_SCRATCH1] = ret;
		}
	} else {
		dev_err(&client->dev, "Bad value [0x%02x] in register 0x%02x\n",
			ret, R_ID4);
		return -ENODEV;
	}
	dev_info(&client->dev, "BMC seems to be valid\n");
	return 0;
}

static int
mitx2_bmc_i2c_probe(struct i2c_client *client,
		    const struct i2c_device_id *id)
{
	int err = 0;
	int i = 0;

	dev_info(&client->dev, "mitx2 bmc probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	for (i = 0; i < 10; i++) {
		err = mitx2_bmc_validate(client);
		if (!err)
			break;
		msleep_interruptible(20);
	}
	if (err)
		return err;

	button_dev = input_allocate_device();
	if (!button_dev) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	button_dev->id.bustype = BUS_I2C;
	button_dev->dev.parent = &client->dev;
	button_dev->name = input_name;
	button_dev->phys = "bmc-input0";
	button_dev->evbit[0] = BIT_MASK(EV_KEY);
	button_dev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

	err = input_register_device(button_dev);
	if (err) {
		dev_err(&client->dev, "Failed to register device\n");
		input_free_device(button_dev);
		return err;
	}

	dev_info(&client->dev, "Starting polling thread\n");
	polling_task = kthread_run(pwroff_rq_poll_fn, NULL, "BMC poll task");

	poll_data.c = client;

	/* register as poweroff handler */
	pm_power_off = bmc_pwroff_rq;

	return 0;
}

static int
mitx2_bmc_i2c_remove(struct i2c_client *client)
{
	kthread_stop(polling_task);
	input_unregister_device(button_dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mitx2_bmc_of_match[] = {
	{ .compatible = "tp,mitx2-bmc" },
	{}
};
MODULE_DEVICE_TABLE(of, mitx2_bmc_of_match);
#endif

static const struct i2c_device_id mitx2_bmc_i2c_id[] = {
	{ "mitx2_bmc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mitx2_bmc_i2c_id);

static ssize_t
version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i.%i.%i\n", bmc_proto_version[0],
			bmc_proto_version[1], bmc_proto_version[2]);
}

static struct kobj_attribute version_attribute =
	__ATTR(version, 0664, version_show, NULL);

static ssize_t
bootreason_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", (bmc_bootreason[0] |
			(bmc_bootreason[1] << 8)));
}

static struct kobj_attribute bootreason_attribute =
	__ATTR(bootreason, 0664, bootreason_show, NULL);

static ssize_t
scratch_show(struct kobject *kobj,
	     struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", (bmc_scratch[0] | (bmc_scratch[1] << 8) |
			(bmc_scratch[2] << 16) | (bmc_scratch[3] << 24)));
}

static struct kobj_attribute scratch_attribute =
	__ATTR(scratch, 0664, scratch_show, NULL);

static struct attribute *bmc_attrs[] = {
	&version_attribute.attr,
	&bootreason_attribute.attr,
	&scratch_attribute.attr,
	NULL,
};

ATTRIBUTE_GROUPS(bmc);

static struct i2c_driver mitx2_bmc_i2c_driver = {
	.driver		= {
		.name	= "mitx2-bmc",
		.of_match_table = of_match_ptr(mitx2_bmc_of_match),
		.groups = bmc_groups,
	},
	.probe		= mitx2_bmc_i2c_probe,
	.remove	  = mitx2_bmc_i2c_remove,
	.id_table	= mitx2_bmc_i2c_id,
};
module_i2c_driver(mitx2_bmc_i2c_driver);

MODULE_AUTHOR("Konstantin Kirik");
MODULE_DESCRIPTION("mITX2 BMC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("serial:bmc");
