/*
 * rtc-ab-eoz9-s3 - Driver for Abracon AB-RTCMC-32.768Khz-EOZ9-S3
 *                  I2C RTC / Alarm chip (reduced smbus version)
 *
 * Copyright (C) 2015,2016 Baikal Electronics JSC
 * 
 *  Author:
 *    Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>
 *
 * Detailed datasheet of the chip is available here:
 *
 * http://www.abracon.com/realtimeclock/AB-RTCMC-32.768kHz-EOZ9-S3-Application-Manual.pdf
 *
 * This work is based on AB-RTCMC-32.768Khz-B5ZE-S3 driver (drivers/rtc/rtc-ab-b5ze-s3.c).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/of.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/thermal.h>

#define DRV_NAME "rtc-ab-eoz9-s3"
#define DRV_VERSION "1.0.2"

/* Control section */
#define ABEOZ9S3_REG_CTRL1	   0x00	   /* Control 1 register */
#define ABEOZ9S3_REG_CTRL1_WE	   BIT(0)  /* 1Hz watch enable */
#define ABEOZ9S3_REG_CTRL1_TE	   BIT(1)  /* Countdown timer enable */
#define ABEOZ9S3_REG_CTRL1_TAR	   BIT(2)  /*  */
#define ABEOZ9S3_REG_CTRL1_EERE	   BIT(3)  /*  */
#define ABEOZ9S3_REG_CTRL1_SRO	   BIT(4)  /*  */
#define ABEOZ9S3_REG_CTRL1_TD0	   BIT(5)  /*  */
#define ABEOZ9S3_REG_CTRL1_TD1	   BIT(6)  /*  */
#define ABEOZ9S3_REG_CTRL1_INT	   BIT(7)

#define ABEOZ9S3_REG_CTRL2	   0x01	   /* Control 2 register */
#define ABEOZ9S3_REG_CTRL2_AIE	   BIT(0)  /*  */
#define ABEOZ9S3_REG_CTRL2_TIE	   BIT(1)  /*  */
#define ABEOZ9S3_REG_CTRL2_V1IE	   BIT(2)  /*  */
#define ABEOZ9S3_REG_CTRL2_V2IE	   BIT(3)  /*  */
#define ABEOZ9S3_REG_CTRL2_SRIE	   BIT(4)  /*  */

#define ABEOZ9S3_REG_CTRL3	   0x02	   /* Control 3 register */
#define ABEOZ9S3_REG_CTRL3_AF	   BIT(0)  /*  */
#define ABEOZ9S3_REG_CTRL3_TF	   BIT(1)  /*  */
#define ABEOZ9S3_REG_CTRL3_V1IF	   BIT(2)  /*  */
#define ABEOZ9S3_REG_CTRL3_V2IF	   BIT(3)  /*  */
#define ABEOZ9S3_REG_CTRL3_SRF	   BIT(4)  /*  */

#define ABEOZ9S3_REG_CTRL4	   0x03	   /* Control 4 register */
#define ABEOZ9S3_REG_CTRL4_V1F	   BIT(2)  /*  */
#define ABEOZ9S3_REG_CTRL4_V2F	   BIT(3)  /*  */
#define ABEOZ9S3_REG_CTRL4_SR	   BIT(4)  /*  */
#define ABEOZ9S3_REG_CTRL4_PON	   BIT(5)  /*  */
#define ABEOZ9S3_REG_CTRL4_EEBUSY  BIT(7)  /*  */

#define ABEOZ9S3_REG_CTRL5	   0x04	   /* Control 5 register */
#define ABEOZ9S3_REG_CTRL5_SYSR	   BIT(4)  /*  */

#define ABEOZ9S3_CTRL_SEC_OFS  0  /* Section offset */
#define ABEOZ9S3_CTRL_SEC_LEN  5  /* Section length */

/* RTC section */
#define ABEOZ9S3_REG_RTC_SC	   0x00	   /* RTC Seconds register */
#define ABEOZ9S3_REG_RTC_MN	   0x01	   /* RTC Minutes register */
#define ABEOZ9S3_REG_RTC_HR	   0x02	   /* RTC Hours register */
#define ABEOZ9S3_REG_RTC_HR_PM BIT(6)  /* RTC Hours PM bit */
#define ABEOZ9S3_REG_RTC_DT	   0x03	   /* RTC Date register */
#define ABEOZ9S3_REG_RTC_DW	   0x04	   /* RTC Day of the week register */
#define ABEOZ9S3_REG_RTC_MO	   0x05	   /* RTC Month register */
#define ABEOZ9S3_REG_RTC_YR	   0x06	   /* RTC Year register */

#define ABEOZ9S3_RTC_SEC_OFS       0x08  /* Section offset */
#define ABEOZ9S3_RTC_SEC_LEN  	   7 	/* Section length */
#define ABEOZ9S3_RTC_SEC_REG(R)    (ABEOZ9S3_RTC_SEC_OFS + R)

/* Alarm section (enable bits are all active low) */
#define ABEOZ9S3_REG_ALRM_SC	   0x00	   /* Alarm - seconds register */
#define ABEOZ9S3_REG_ALRM_SC_AE	   BIT(7)  /* Second enable */
#define ABEOZ9S3_REG_ALRM_MN	   0x01	   /* Alarm - minute register */
#define ABEOZ9S3_REG_ALRM_MN_AE	   BIT(7)  /* Minute enable */
#define ABEOZ9S3_REG_ALRM_HR	   0x02	   /* Alarm - hours register */
#define ABEOZ9S3_REG_ALRM_HR_AE	   BIT(7)  /* Hour enable */
#define ABEOZ9S3_REG_ALRM_DT	   0x03	   /* Alarm - date register */
#define ABEOZ9S3_REG_ALRM_DT_AE	   BIT(7)  /* Date (day of the month) enable */
#define ABEOZ9S3_REG_ALRM_DW	   0x04	   /* Alarm - day of the week reg. */
#define ABEOZ9S3_REG_ALRM_DW_AE	   BIT(7)  /* Day of the week enable */
#define ABEOZ9S3_REG_ALRM_MO	   0x05	   /* Alarm - month reg. */
#define ABEOZ9S3_REG_ALRM_MO_AE	   BIT(7)  /* Month enable */
#define ABEOZ9S3_REG_ALRM_YR	   0x06	   /* Alarm - year reg. */
#define ABEOZ9S3_REG_ALRM_YR_AE	   BIT(7)  /* Year enable */

#define ABEOZ9S3_ALRM_SEC_OFS  0x10  /* Section offset */
#define ABEOZ9S3_ALRM_SEC_LEN  7 	/* Section length */

/* Timer counter section */
#define ABEOZ9S3_REG_TIMER_LOW	   0x18	   /* Timer counter low byte register */
#define ABEOZ9S3_REG_TIMER_HIGH	   0x19	   /* Timer counter high byte register */

/* Temperature sensor section */
#define ABEOZ9S3_REG_TEMP	   		0x20	   /* Temperature sensor register */
#define ABEOZ9S3_TEMP_MAX_POS		194
#define ABEOZ9S3_TEMP_MAX_NEG		60

/* EEPROM user section */
#define ABEOZ9S3_REG_EEPROM_USR1   0x28	   /* User EEPROM data byte 1 register */
#define ABEOZ9S3_REG_EEPROM_USR2   0x29	   /* User EEPROM data byte 2 register */

/* EEPROM control section */
#define ABEOZ9S3_REG_EEPROM_CTL	   0x30	   /* EEPROM control register */
#define ABEOZ9S3_REG_EEPROM_THP	   BIT(0)  /*  */
#define ABEOZ9S3_REG_EEPROM_THE	   BIT(1)  /*  */
#define ABEOZ9S3_REG_EEPROM_FD0	   BIT(2)  /*  */
#define ABEOZ9S3_REG_EEPROM_FD1	   BIT(3)  /*  */
#define ABEOZ9S3_REG_EEPROM_R1K	   BIT(4)  /*  */
#define ABEOZ9S3_REG_EEPROM_R5K	   BIT(4)  /*  */
#define ABEOZ9S3_REG_EEPROM_R20K   BIT(4)  /*  */
#define ABEOZ9S3_REG_EEPROM_R80K   BIT(4)  /*  */

#define ABEOZ9S3_MEM_MAP_LEN	   0x30

struct abeoz9s3_rtc_data {
	struct i2c_client *client;
	struct rtc_device *rtc;
	struct mutex lock;
	struct device		*hwmon_dev;
	struct thermal_zone_device	*tz;
};

/*
 * Try and match register bits w/ fixed null values to see whether we
 * are dealing with an ABEOZ9S3. Note: this function is called early
 * during init and hence does need mutex protection.
 */
static int abeoz9s3_i2c_validate_client(struct i2c_client *client)
{
	static const unsigned char regmask[] = {
			/* Control page */
			ABEOZ9S3_CTRL_SEC_OFS + ABEOZ9S3_REG_CTRL1, 0x00,
			ABEOZ9S3_CTRL_SEC_OFS + ABEOZ9S3_REG_CTRL2, 0xe0,
			ABEOZ9S3_CTRL_SEC_OFS + ABEOZ9S3_REG_CTRL3, 0xe0,
			ABEOZ9S3_CTRL_SEC_OFS + ABEOZ9S3_REG_CTRL4, 0x43,
			ABEOZ9S3_CTRL_SEC_OFS + ABEOZ9S3_REG_CTRL5, 0xef,
			/* Time page */
			ABEOZ9S3_RTC_SEC_OFS + ABEOZ9S3_REG_RTC_SC, 0x80,
			ABEOZ9S3_RTC_SEC_OFS + ABEOZ9S3_REG_RTC_MN, 0x80,
			ABEOZ9S3_RTC_SEC_OFS + ABEOZ9S3_REG_RTC_HR, 0x80,
			ABEOZ9S3_RTC_SEC_OFS + ABEOZ9S3_REG_RTC_DT, 0xc0,
			ABEOZ9S3_RTC_SEC_OFS + ABEOZ9S3_REG_RTC_DW, 0xf8,
			ABEOZ9S3_RTC_SEC_OFS + ABEOZ9S3_REG_RTC_MO, 0xe0,
			ABEOZ9S3_RTC_SEC_OFS + ABEOZ9S3_REG_RTC_YR, 0x80,
			/* Alarm page */
			ABEOZ9S3_ALRM_SEC_OFS + ABEOZ9S3_REG_ALRM_HR, 0x40,
			ABEOZ9S3_ALRM_SEC_OFS + ABEOZ9S3_REG_ALRM_DT, 0x40,
			ABEOZ9S3_ALRM_SEC_OFS + ABEOZ9S3_REG_ALRM_DW, 0x78,
			ABEOZ9S3_ALRM_SEC_OFS + ABEOZ9S3_REG_ALRM_MO, 0x60,
		};

	int i, ret;

	for (i = 0; i < ARRAY_SIZE(regmask); i += 2) {

		ret = i2c_smbus_read_byte_data(client, regmask[i]);

		if (ret < 0) {
			dev_err(&client->dev, "%s: could not read register %x\n",
				__func__, regmask[i]);

			return -EIO;
		}

		if (ret & regmask[i+1]) {
			dev_err(&client->dev,
				"%s: register=%02x, mask=0x%02x, value=0x%02x\n",
				__func__, regmask[i], regmask[i+1], ret);

			return -ENODEV;
		}
	}

	return 0;
}

/*
 * Note: we only read, so regmap inner lock protection is sufficient, i.e.
 * we do not need driver's main lock protection.
 */
static int abeoz9s3_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
//	struct abeoz9s3_rtc_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char regs[ABEOZ9S3_RTC_SEC_LEN];
	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {

		regs[i] = i2c_smbus_read_byte_data(client, ABEOZ9S3_RTC_SEC_REG(i));

		if (regs[i] < 0) {
			dev_err(dev, "%s: could not read register %x\n",
				__func__, ABEOZ9S3_RTC_SEC_OFS + i);

			return -EIO;
		}
	}

	tm->tm_sec = bcd2bin(regs[ABEOZ9S3_REG_RTC_SC] & 0x7F);
	tm->tm_min = bcd2bin(regs[ABEOZ9S3_REG_RTC_MN] & 0x7F);

	if (regs[ABEOZ9S3_REG_RTC_HR] & ABEOZ9S3_REG_RTC_HR_PM) { /* 12hr mode */
		tm->tm_hour = bcd2bin(regs[ABEOZ9S3_REG_RTC_HR] & 0x1f);
		if (regs[ABEOZ9S3_REG_RTC_HR] & ABEOZ9S3_REG_RTC_HR_PM) /* PM */
			tm->tm_hour += 12;
	} else {
		/* 24hr mode */
		tm->tm_hour = bcd2bin(regs[ABEOZ9S3_REG_RTC_HR]);
	}

	tm->tm_mday = bcd2bin(regs[ABEOZ9S3_REG_RTC_DT]);
	tm->tm_wday = bcd2bin(regs[ABEOZ9S3_REG_RTC_DW]);
	tm->tm_mon  = bcd2bin(regs[ABEOZ9S3_REG_RTC_MO]) - 1; /* starts at 1 */
	tm->tm_year = bcd2bin(regs[ABEOZ9S3_REG_RTC_YR]) + 100; /* starts at 20xx */

	return rtc_valid_tm(tm);
}

static int abeoz9s3_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct abeoz9s3_rtc_data *data = dev_get_drvdata(dev);
	unsigned char regs[ABEOZ9S3_RTC_SEC_LEN];
	int i, ret = 0;

	/*
	 * Year register is 8-bit wide and bcd-coded, i.e records values
	 * between 0 and 99. tm_year is an offset from 1900 and we are
	 * interested in the 2000-2099 range, so any value less than 100
	 * is invalid.
	 */
	if (tm->tm_year < 100)
		return -EINVAL;

	regs[ABEOZ9S3_REG_RTC_SC] = bin2bcd(tm->tm_sec); /* MSB=0 clears OSC */
	regs[ABEOZ9S3_REG_RTC_MN] = bin2bcd(tm->tm_min);
	regs[ABEOZ9S3_REG_RTC_HR] = bin2bcd(tm->tm_hour); /* 24-hour format */
	regs[ABEOZ9S3_REG_RTC_DT] = bin2bcd(tm->tm_mday);
	regs[ABEOZ9S3_REG_RTC_DW] = bin2bcd(tm->tm_wday);
	regs[ABEOZ9S3_REG_RTC_MO] = bin2bcd(tm->tm_mon + 1);
	regs[ABEOZ9S3_REG_RTC_YR] = bin2bcd(tm->tm_year - 100);

	mutex_lock(&data->lock);

	for (i = 0; i < ARRAY_SIZE(regs); i++) {

		ret = i2c_smbus_write_byte_data(client, ABEOZ9S3_RTC_SEC_REG(i), regs[i]);

		if (ret < 0) {
			dev_err(&client->dev, "%s: could not write register %x\n",
				__func__, ABEOZ9S3_RTC_SEC_OFS + i);
			goto err;
		}
	}
	/* Check and clean PON flag */
	ret = i2c_smbus_read_byte_data(client, ABEOZ9S3_REG_CTRL4);
	if (ret < 0) {
		dev_err(dev, "%s: unable to get CTRL4 register (%d)\n",
			__func__, ret);
		goto err;
	}
	/* Check and clear */
	if (ret & ABEOZ9S3_REG_CTRL4_PON) {
		ret = i2c_smbus_write_byte_data(client, ABEOZ9S3_REG_CTRL4,
									ret & ~(ABEOZ9S3_REG_CTRL4_PON));
		if (ret < 0)
			dev_err(dev, "%s: unable to set CTRL4 register (%d)\n",
				__func__, ret);
	}
err:
	mutex_unlock(&data->lock);

	return ret;
}

static int abeoz9s3_rtc_temp(void *sensor)
{
	struct abeoz9s3_rtc_data *data = sensor;
	struct i2c_client *client = data->client;
	int ret;

	ret = i2c_smbus_read_byte_data(client, ABEOZ9S3_REG_TEMP);

	if (ret < 0) {
		dev_err(data->hwmon_dev, "%s: could not read register %x\n",
			__func__, ABEOZ9S3_REG_TEMP);
	}

	return (ret - ABEOZ9S3_TEMP_MAX_NEG);
}

static inline int abeoz9s3_rtc_temp_valid(int temp)
{
	return (temp >= ABEOZ9S3_TEMP_MAX_NEG) ||
		   (temp <= ABEOZ9S3_TEMP_MAX_POS);
}

/* sysfs attributes for hwmon */

static int abeoz9s3_read_temp(void *sensor, int *temp)
{
	*temp = abeoz9s3_rtc_temp(sensor) * 1000;

	return 0;
}

static ssize_t abeoz9s3_show_temp(struct device *dev, struct device_attribute *da,
			 char *buf)
{
	struct abeoz9s3_rtc_data *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int temp;

	switch (attr->index) {
	case 0:
		temp = abeoz9s3_rtc_temp(data);
		break;
	case 1:
		temp = -ABEOZ9S3_TEMP_MAX_NEG;
		break;
	case 2:
		temp = ABEOZ9S3_TEMP_MAX_POS;
		break;
	default:
		temp = -EINVAL;
	}

	return sprintf(buf, "%d\n", temp * 1000);
}

static int abeoz9s3_rtc_proc(struct device *dev, struct seq_file *seq)
{
	int temp = abeoz9s3_rtc_temp(dev);

	if (abeoz9s3_rtc_temp_valid(temp))
		seq_printf(seq, "Temperature	: %2d°C\n", temp);

	return 0;
}

/*
 * Check current RTC status and enable/disable what needs to be. Return 0 if
 * everything went ok and a negative value upon error. Note: this function
 * is called early during init and hence does need mutex protection.
 */
static int abeoz9s3_rtc_setup(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	/* Control1 register setup */
	ret = i2c_smbus_read_byte_data(client, ABEOZ9S3_REG_CTRL1);
	if (ret < 0) {
		dev_err(dev, "%s: unable to get CTRL1 register (%d)\n",
			__func__, ret);
		return ret;
	}
	/* Enable Self Recovery, Clock for Watch, EEPROM refresh */
	ret |= (ABEOZ9S3_REG_CTRL1_WE | ABEOZ9S3_REG_CTRL1_EERE |
		ABEOZ9S3_REG_CTRL1_SRO);
	/* Disable Countdown Timer, Set INT function on CLKOUT pin */
	ret &= ~(ABEOZ9S3_REG_CTRL1_TE | ABEOZ9S3_REG_CTRL1_TAR |
		ABEOZ9S3_REG_CTRL1_INT);
	/* Set new value */
	ret = i2c_smbus_write_byte_data(client, ABEOZ9S3_REG_CTRL1, ret);
	if (ret < 0) {
		dev_err(dev, "%s: unable to set CTRL1 register (%d)\n",
			__func__, ret);
		return ret;
	}

	/* Control INT register setup. No interrupts */
	ret = i2c_smbus_write_byte_data(client, ABEOZ9S3_REG_CTRL2, 0);
	if (ret < 0) {
		dev_err(dev, "%s: unable to set CTRL2 register (%d)\n",
			__func__, ret);
		return ret;
	}

	/* Control INT Flag register setup. Clear flags */
	ret = i2c_smbus_write_byte_data(client, ABEOZ9S3_REG_CTRL3, 0);
	if (ret < 0) {
		dev_err(dev, "%s: unable to set CTRL3 register (%d)\n",
			__func__, ret);
		return ret;
	}

	/* Control Status register setup. */
	ret = i2c_smbus_read_byte_data(client, ABEOZ9S3_REG_CTRL4);
	if (ret < 0) {
		dev_err(dev, "%s: unable to get CTRL4 register (%d)\n",
			__func__, ret);
		return ret;
	}

	if (ret & ABEOZ9S3_REG_CTRL4_PON)
		dev_warn(dev, "RTC Date/Time may be corrupted!\n");

	/* Clear all status bits */
	ret = i2c_smbus_write_byte_data(client, ABEOZ9S3_REG_CTRL4, 0);
	if (ret < 0) {
		dev_err(dev, "%s: unable to set CTRL4 register (%d)\n",
			__func__, ret);
		return ret;
	}

	ret = i2c_smbus_read_byte_data(client, ABEOZ9S3_REG_EEPROM_CTL);
	if (ret < 0) {
		dev_err(dev, "%s: unable to get EEPROM Control register (%d)\n",
			__func__, ret);
		return ret;
	}
	/* Enable Termometer */
	ret |= (ABEOZ9S3_REG_EEPROM_THE);
	/* Disable charge resistors, Temperature Scanning Interval: 1 sec */
	ret &= ~((ABEOZ9S3_REG_EEPROM_R1K | ABEOZ9S3_REG_EEPROM_R5K |
		ABEOZ9S3_REG_EEPROM_R20K | ABEOZ9S3_REG_EEPROM_R80K |
		ABEOZ9S3_REG_EEPROM_THP));

	ret = i2c_smbus_write_byte_data(client, ABEOZ9S3_REG_EEPROM_CTL, ret);
	if (ret < 0) {
		dev_err(dev, "%s: unable to set EEPROM Control register (%d)\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, abeoz9s3_show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_min, S_IRUGO, abeoz9s3_show_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_max, S_IRUGO, abeoz9s3_show_temp, NULL, 2);

static struct attribute *abeoz9s3_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_min.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(abeoz9s3);

static const struct thermal_zone_of_device_ops abeoz9s3_of_thermal_ops = {
	.get_temp = abeoz9s3_read_temp,
};

static const struct rtc_class_ops rtc_ops = {
	.read_time 	= abeoz9s3_rtc_read_time,
	.set_time 	= abeoz9s3_rtc_set_time,
	.proc 		= abeoz9s3_rtc_proc,
};

static int abeoz9s3_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct abeoz9s3_rtc_data *data = NULL;
	struct device *dev = &client->dev;
	int ret;

	dev_dbg(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C |
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENODEV;

	ret = abeoz9s3_i2c_validate_client(client);
	if (ret)
		return ret;

	dev_info(&client->dev, "chip found, driver version " DRV_VERSION "\n");

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_set_drvdata(dev, data);
	mutex_init(&data->lock);
	data->client = client;

	ret = abeoz9s3_rtc_setup(dev);
	if (ret)
		goto err;

	data->rtc = devm_rtc_device_register(dev, DRV_NAME, &rtc_ops,
					     THIS_MODULE);
	ret = PTR_ERR_OR_ZERO(data->rtc);
	if (ret) {
		dev_err(dev, "%s: unable to register RTC device (%d)\n",
			__func__, ret);
		goto err;
	}

	data->hwmon_dev = hwmon_device_register_with_groups(dev, client->name,
							    data, abeoz9s3_groups);
	if (IS_ERR(data->hwmon_dev)) {
		data->hwmon_dev = NULL;
		goto err;
	}

	data->tz = thermal_zone_of_sensor_register(data->hwmon_dev, 0,
						   data->hwmon_dev,
						   &abeoz9s3_of_thermal_ops);
	if (IS_ERR(data->tz))
		data->tz = NULL;

	dev_info(dev, "%s: sensor '%s'\n",
		 dev_name(data->hwmon_dev), client->name);


err:
	return ret;
}

static int abeoz9s3_remove(struct i2c_client *client)
{
	struct abeoz9s3_rtc_data *data = i2c_get_clientdata(client);

	if (data->hwmon_dev) {
		thermal_zone_of_sensor_unregister(data->hwmon_dev, data->tz);
		hwmon_device_unregister(data->hwmon_dev);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id abeoz9s3_dt_match[] = {
	{ .compatible = "abracon,abeoz9s3" },
	{ },
};
#endif

static const struct i2c_device_id abeoz9s3_id[] = {
	{ "abeoz9s3", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, abeoz9s3_id);

static struct i2c_driver abeoz9s3_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(abeoz9s3_dt_match),
	},
	.probe	  = abeoz9s3_probe,
	.remove	  = abeoz9s3_remove,
	.id_table = abeoz9s3_id,
};
module_i2c_driver(abeoz9s3_driver);

MODULE_AUTHOR("Dmitry Dunaev");
MODULE_DESCRIPTION("Abracon AB-RTCMC-32.768kHz-EOZ9-S3 RTC/Alarm driver");
MODULE_LICENSE("GPL2");
MODULE_VERSION(DRV_VERSION);
