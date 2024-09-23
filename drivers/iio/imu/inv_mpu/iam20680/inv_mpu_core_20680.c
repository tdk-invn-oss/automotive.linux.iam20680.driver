// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017-2020 InvenSense, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "inv_mpu: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
#include <linux/minmax.h>
#endif

#include "../inv_mpu_iio.h"


static const struct inv_hw_s hw_info[INV_NUM_PARTS] = {
	[IAM20680] = {128, "iam20680"},
};

#ifndef SUPPORT_ONLY_BASIC_FEATURES
static char debug_reg_addr = 0x6;
#endif

static const char sensor_l_info[][30] = {
	"SENSOR_L_ACCEL",
	"SENSOR_L_GYRO",
	"SENSOR_L_MAG",
	"SENSOR_L_ALS",
	"SENSOR_L_SIXQ",
	"SENSOR_L_THREEQ",
	"SENSOR_L_NINEQ",
	"SENSOR_L_PEDQ",
	"SENSOR_L_GEOMAG",
	"SENSOR_L_PRESSURE",
	"SENSOR_L_GYRO_CAL",
	"SENSOR_L_MAG_CAL",
	"SENSOR_L_EIS_GYRO",
	"SENSOR_L_ACCEL_WAKE",
	"SENSOR_L_GYRO_WAKE",
	"SENSOR_L_MAG_WAKE",
	"SENSOR_L_ALS_WAKE",
	"SENSOR_L_SIXQ_WAKE",
	"SENSOR_L_NINEQ_WAKE",
	"SENSOR_L_PEDQ_WAKE",
	"SENSOR_L_GEOMAG_WAKE",
	"SENSOR_L_PRESSURE_WAKE",
	"SENSOR_L_GYRO_CAL_WAKE",
	"SENSOR_L_MAG_CAL_WAKE",
	"SENSOR_L_NUM_MAX",
};

static int inv_set_accel_bias_reg(struct inv_mpu_state *st,
			int accel_bias, int axis)
{
	int accel_reg_bias;
	u8 addr;
	u8 d[2];
	int result = 0;

	switch (axis) {
	case 0:
		/* X */
		addr = REG_XA_OFFS_H;
		accel_reg_bias = st->org_accel_offset_reg[0];
		break;
	case 1:
		/* Y */
		addr = REG_YA_OFFS_H;
		accel_reg_bias = st->org_accel_offset_reg[1];
		break;
	case 2:
		/* Z* */
		addr = REG_ZA_OFFS_H;
		accel_reg_bias = st->org_accel_offset_reg[2];
		break;
	default:
		result = -EINVAL;
		goto accel_bias_set_err;
	}

	/* accel_bias is 2g scaled by 1<<16.
	 * Convert to 16g, and mask bit0
	 */
	accel_reg_bias -= ((accel_bias / 8 / 65536) & ~1);

	d[0] = (accel_reg_bias >> 8) & 0xff;
	d[1] = (accel_reg_bias) & 0xff;
	result = inv_plat_single_write(st, addr, d[0]);
	if (result)
		goto accel_bias_set_err;
	result = inv_plat_single_write(st, addr + 1, d[1]);
	if (result)
		goto accel_bias_set_err;

accel_bias_set_err:
	return result;
}

static int inv_set_gyro_bias_reg(struct inv_mpu_state *st,
			const int gyro_bias, int axis)
{
	int gyro_reg_bias;
	u8 addr;
	u8 d[2];
	int result = 0;

	switch (axis) {
	case 0:
		/* X */
		addr = REG_XG_OFFS_USR_H;
		break;
	case 1:
		/* Y */
		addr = REG_YG_OFFS_USR_H;
		break;
	case 2:
		/* Z */
		addr = REG_ZG_OFFS_USR_H;
		break;
	default:
		result = -EINVAL;
		goto gyro_bias_set_err;
	}

	/* gyro_bias is 2000dps scaled by 1<<16.
	 * Convert to 1000dps
	 */
	gyro_reg_bias = (-gyro_bias * 2 / 65536);

	d[0] = (gyro_reg_bias >> 8) & 0xff;
	d[1] = (gyro_reg_bias) & 0xff;
	result = inv_plat_single_write(st, addr, d[0]);
	if (result)
		goto gyro_bias_set_err;
	result = inv_plat_single_write(st, addr + 1, d[1]);
	if (result)
		goto gyro_bias_set_err;

gyro_bias_set_err:
	return result;
}

static int _bias_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result, data;

	result = inv_switch_power_in_lp(st, true);
	if (result)
		return result;

	result = kstrtoint(buf, 10, &data);
	if (result)
		goto bias_store_fail;
	switch (this_attr->address) {
	case ATTR_ACCEL_X_OFFSET:
		result = inv_set_accel_bias_reg(st, data, 0);
		if (result)
			goto bias_store_fail;
		st->input_accel_bias[0] = data;
		break;
	case ATTR_ACCEL_Y_OFFSET:
		result = inv_set_accel_bias_reg(st, data, 1);
		if (result)
			goto bias_store_fail;
		st->input_accel_bias[1] = data;
		break;
	case ATTR_ACCEL_Z_OFFSET:
		result = inv_set_accel_bias_reg(st, data, 2);
		if (result)
			goto bias_store_fail;
		st->input_accel_bias[2] = data;
		break;
	case ATTR_GYRO_X_OFFSET:
		result = inv_set_gyro_bias_reg(st, data, 0);
		if (result)
			goto bias_store_fail;
		st->input_gyro_bias[0] = data;
		break;
	case ATTR_GYRO_Y_OFFSET:
		result = inv_set_gyro_bias_reg(st, data, 1);
		if (result)
			goto bias_store_fail;
		st->input_gyro_bias[1] = data;
		break;
	case ATTR_GYRO_Z_OFFSET:
		result = inv_set_gyro_bias_reg(st, data, 2);
		if (result)
			goto bias_store_fail;
		st->input_gyro_bias[2] = data;
		break;
	default:
		break;
	}

bias_store_fail:
	if (result)
		return result;
	result = inv_switch_power_in_lp(st, false);
	if (result)
		return result;

	return count;
}

static ssize_t inv_bias_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int result;

	mutex_lock(&st->lock);
	result = _bias_store(dev, attr, buf, count);
	mutex_unlock(&st->lock);

	return result;
}

#ifndef SUPPORT_ONLY_BASIC_FEATURES
static ssize_t inv_debug_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result, data;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;
	switch (this_attr->address) {
	case ATTR_DMP_LP_EN_OFF:
		st->chip_config.lp_en_mode_off = !!data;
		inv_switch_power_in_lp(st, !!data);
		break;
	case ATTR_DMP_CLK_SEL:
		st->chip_config.clk_sel = !!data;
		inv_switch_power_in_lp(st, !!data);
		break;
	case ATTR_DEBUG_REG_ADDR:
		debug_reg_addr = data;
		break;
	case ATTR_DEBUG_REG_WRITE:
		inv_plat_single_write(st, debug_reg_addr, data);
		break;
	}
	return count;
}
#endif

static int _misc_attr_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result, data;

	result = inv_switch_power_in_lp(st, true);
	if (result)
		return result;
	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;
	switch (this_attr->address) {
	case ATTR_GYRO_SCALE:
		if (data > 3)
			return -EINVAL;
		st->chip_config.fsr = data;
		result = inv_set_gyro_sf(st);
		return result;
	case ATTR_ACCEL_SCALE:
		if (data > 3)
			return -EINVAL;
		st->chip_config.accel_fs = data;
		result = inv_set_accel_sf(st);
		return result;
	default:
		return -EINVAL;
	}
	st->trigger_state = MISC_TRIGGER;
	result = set_inv_enable(indio_dev);

	return result;
}

/*
 * inv_misc_attr_store() -  calling this function
 */
static ssize_t inv_misc_attr_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int result;

	mutex_lock(&st->lock);
	result = _misc_attr_store(dev, attr, buf, count);
	mutex_unlock(&st->lock);
	if (result)
		return result;

	return count;
}

static ssize_t inv_sensor_rate_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	return snprintf(buf, MAX_WR_SZ, "%d\n",
					st->sensor_l[this_attr->address].rate);
}

static ssize_t inv_sensor_rate_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int data, rate, ind;
	int result;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return -EINVAL;
	if (data <= 0) {
		pr_err("sensor_rate_store: invalid data=%d\n", data);
		return -EINVAL;
	}
	ind = this_attr->address;
	rate = inv_rate_convert(st, ind, data);

	pr_debug("sensor [%s] requested  rate %d input [%d]\n",
						sensor_l_info[ind], rate, data);

	if (rate == st->sensor_l[ind].rate)
		return count;
	mutex_lock(&st->lock);
	st->sensor_l[ind].rate = rate;
	st->trigger_state = DATA_TRIGGER;
	inv_check_sensor_on(st);
	result = set_inv_enable(indio_dev);
	pr_debug("%s rate %d div %d\n", sensor_l_info[ind],
				st->sensor_l[ind].rate, st->sensor_l[ind].div);
	mutex_unlock(&st->lock);

	return count;
}

static ssize_t inv_sensor_on_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	return snprintf(buf, MAX_WR_SZ, "%d\n",
			st->sensor_l[this_attr->address].on);
}

static ssize_t inv_sensor_on_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int data, on, ind;
	int result;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return -EINVAL;
	if (data < 0) {
		pr_err("sensor_on_store: invalid data=%d\n", data);
		return -EINVAL;
	}
	ind = this_attr->address;
	on = !!data;

	pr_debug("sensor [%s] requested  %s, input [%d]\n",
			sensor_l_info[ind], (on == 1) ? "On" : "Off", data);

	if (on == st->sensor_l[ind].on) {
		pr_debug("sensor [%s] is already %s, input [%d]\n",
			sensor_l_info[ind], (on == 1) ? "On" : "Off", data);
		return count;
	}

	mutex_lock(&st->lock);
	st->sensor_l[ind].on = on;
	st->trigger_state = RATE_TRIGGER;
	inv_check_sensor_on(st);
	result = set_inv_enable(indio_dev);
	mutex_unlock(&st->lock);
	if (result)
		return result;

	pr_debug("Sensor [%s] is %s by sysfs\n",
				sensor_l_info[ind], (on == 1) ? "On" : "Off");
	return count;
}

static int inv_check_l_step(struct inv_mpu_state *st)
{
	if (st->step_counter_l_on || st->step_counter_wake_l_on)
		st->ped.on = true;
	else
		st->ped.on = false;

	return 0;
}

static int _basic_attr_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int data;
	int result;
	u32 power_on_data;

	result = kstrtoint(buf, 10, &data);
	if (result || (data < 0))
		return -EINVAL;

	switch (this_attr->address) {
	case ATTR_DMP_PED_ON:
		if ((!!data) == st->ped.on)
			return count;
		st->ped.on = !!data;
		break;
	case ATTR_DMP_TILT_ENABLE:
		if ((!!data) == st->chip_config.tilt_enable)
			return count;
		st->chip_config.tilt_enable = !!data;
		pr_info("Tile %s\n",
			st->chip_config.tilt_enable ==
			1 ? "Enabled" : "Disabled");
		break;
	case ATTR_DMP_PICK_UP_ENABLE:
		if ((!!data) == st->chip_config.pick_up_enable) {
			pr_info("Pick_up enable already %s\n",
				st->chip_config.pick_up_enable ==
				1 ? "Enabled" : "Disabled");
			return count;
		}
		st->chip_config.pick_up_enable = !!data;
		pr_info("Pick up %s\n",
			st->chip_config.pick_up_enable ==
			1 ? "Enable" : "Disable");
		break;
	case ATTR_IN_POWER_ON:
		{
			u8 p0[2];
			u8 p1[2];

			power_on_data = (u32)data;
			p0[0] = (power_on_data & 0xff);
			p0[1] = ((power_on_data >> 8) & 0xff);
			p1[0] = ((power_on_data >> 16) & 0xff);
			p1[1] = ((power_on_data >> 24) & 0xff);

			if (st->bus_type == BUS_SPI) {
				struct spi_transfer power_on;
				struct spi_message msg;

				memset(&power_on, 0, sizeof(struct spi_transfer));

				power_on.bits_per_word = 8;
				power_on.len = 2;

				power_on.tx_buf = p0;
				power_on.rx_buf = p1;
				spi_message_init(&msg);
				spi_message_add_tail(&power_on, &msg);
				spi_sync(to_spi_device(st->dev), &msg);

			} else if (st->bus_type == BUS_I2C) {
				struct i2c_msg msgs[2];

				p0[0] &= 0x7f;

				msgs[0].addr = st->i2c_addr;
				msgs[0].flags = 0;	/* write */
				msgs[0].buf = &p0[0];
				msgs[0].len = 1;

				msgs[1].addr = st->i2c_addr;
				msgs[1].flags = I2C_M_RD;
				msgs[1].buf = &p1[1];
				msgs[1].len = 1;

				result = i2c_transfer(st->sl_handle, msgs, 2);
				if (result < 2)
					return -EIO;
			}
			st->power_on_data = ((p0[0] << 24) | (p0[1] << 16) |
							(p1[0] << 8) | p1[1]);
			return count;
		}
	case ATTR_DMP_EIS_ENABLE:
		if ((!!data) == st->chip_config.eis_enable)
			return count;
		st->chip_config.eis_enable = !!data;
		pr_info("Eis %s\n",
			st->chip_config.eis_enable == 1 ? "Enable" : "Disable");
		break;
	case ATTR_DMP_STEP_DETECTOR_ON:
		st->step_detector_l_on = !!data;
		break;
	case ATTR_DMP_STEP_DETECTOR_WAKE_ON:
		st->step_detector_wake_l_on = !!data;
		break;
	case ATTR_DMP_STEP_COUNTER_ON:
		st->step_counter_l_on = !!data;
		break;
	case ATTR_DMP_STEP_COUNTER_WAKE_ON:
		st->step_counter_wake_l_on = !!data;
		break;
	case ATTR_DMP_BATCHMODE_TIMEOUT:
		if (data == st->batch.timeout)
			return count;
		st->batch.timeout = data;
		break;
	default:
		return -EINVAL;
	};
	inv_check_l_step(st);
	inv_check_sensor_on(st);

	st->trigger_state = EVENT_TRIGGER;
	result = set_inv_enable(indio_dev);
	if (result)
		return result;

	return count;
}

/*
 * inv_basic_attr_store()
 */
static ssize_t inv_basic_attr_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int result;

	mutex_lock(&st->lock);
	result = _basic_attr_store(dev, attr, buf, count);
	mutex_unlock(&st->lock);

	return result;
}

/*
 * inv_attr_show()
 */
static ssize_t inv_attr_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	s8 *m;

	switch (this_attr->address) {
	case ATTR_GYRO_SCALE:
		{
			const s16 gyro_scale[] = { 250, 500, 1000, 2000 };

			return snprintf(buf, MAX_WR_SZ, "%d\n",
				gyro_scale[st->chip_config.fsr]);
		}
	case ATTR_ACCEL_SCALE:
		{
			const s16 accel_scale[] = { 2, 4, 8, 16 };

			return snprintf(buf, MAX_WR_SZ, "%d\n",
				accel_scale[st->chip_config.accel_fs]);
		}
	case ATTR_GYRO_ENABLE:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->chip_config.gyro_enable);
	case ATTR_ACCEL_ENABLE:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->chip_config.accel_enable);
	case ATTR_IN_POWER_ON:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->power_on_data);
	case ATTR_DMP_BATCHMODE_TIMEOUT:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->batch.timeout);
	case ATTR_DMP_PED_ON:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->ped.on);
	case ATTR_DMP_TILT_ENABLE:
		return snprintf(buf, MAX_WR_SZ, "%d\n",
			st->chip_config.tilt_enable);
	case ATTR_DMP_PICK_UP_ENABLE:
		return snprintf(buf, MAX_WR_SZ, "%d\n",
			st->chip_config.pick_up_enable);
	case ATTR_DMP_EIS_ENABLE:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->chip_config.eis_enable);
	case ATTR_DMP_LP_EN_OFF:
		return snprintf(buf, MAX_WR_SZ, "%d\n",
			st->chip_config.lp_en_mode_off);
	case ATTR_DMP_STEP_COUNTER_ON:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->step_counter_l_on);
	case ATTR_DMP_STEP_COUNTER_WAKE_ON:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->step_counter_wake_l_on);
	case ATTR_DMP_STEP_DETECTOR_ON:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->step_detector_l_on);
	case ATTR_DMP_STEP_DETECTOR_WAKE_ON:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->step_detector_wake_l_on);
	case ATTR_GYRO_MATRIX:
		m = st->plat_data.orientation;
		return snprintf(buf, MAX_WR_SZ, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
			m[8]);
	case ATTR_ACCEL_MATRIX:
		m = st->plat_data.orientation;
		return snprintf(buf, MAX_WR_SZ, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
			m[8]);
	case ATTR_GYRO_SF:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->gyro_sf);
	case ATTR_ANGLVEL_X_ST_CALIBBIAS:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->gyro_st_bias[0]);
	case ATTR_ANGLVEL_Y_ST_CALIBBIAS:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->gyro_st_bias[1]);
	case ATTR_ANGLVEL_Z_ST_CALIBBIAS:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->gyro_st_bias[2]);
	case ATTR_GYRO_LP_MODE:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->gyro_lp_mode);
	case ATTR_ACCEL_LP_MODE:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->accel_lp_mode);
	case ATTR_ACCEL_X_ST_CALIBBIAS:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->accel_st_bias[0]);
	case ATTR_ACCEL_Y_ST_CALIBBIAS:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->accel_st_bias[1]);
	case ATTR_ACCEL_Z_ST_CALIBBIAS:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->accel_st_bias[2]);
	case ATTR_GYRO_X_OFFSET:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->input_gyro_bias[0]);
	case ATTR_GYRO_Y_OFFSET:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->input_gyro_bias[1]);
	case ATTR_GYRO_Z_OFFSET:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->input_gyro_bias[2]);
	case ATTR_ACCEL_X_OFFSET:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->input_accel_bias[0]);
	case ATTR_ACCEL_Y_OFFSET:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->input_accel_bias[1]);
	case ATTR_ACCEL_Z_OFFSET:
		return snprintf(buf, MAX_WR_SZ, "%d\n", st->input_accel_bias[2]);
	default:
		return -EPERM;
	}
}

static ssize_t misc_self_test_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int res;

	mutex_lock(&st->lock);
	res = inv_hw_self_test(st);
	set_inv_enable(indio_dev);
	mutex_unlock(&st->lock);

	return snprintf(buf, MAX_WR_SZ, "%d\n", res);
}


/*
 *  out_temperature_show() - Read temperature data directly from registers.
 */
static ssize_t out_temperature_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	u8 data[2];
	s32 temp;
	int res;

	mutex_lock(&st->lock);
	res = inv_plat_read(st, REG_RAW_TEMP, 2, data);
	mutex_unlock(&st->lock);
	if (res)
		return res;

	temp = (s16)be16_to_cpup((__be16 *)(data)) * 10000;
	temp = temp / TEMP_SENSITIVITY + TEMP_OFFSET;

	return snprintf(buf, MAX_WR_SZ, "%d %lld\n", temp, get_time_ns());
}

/*
 *  debug_reg_dump_show() - Register dump for testing.
 */
static ssize_t debug_reg_dump_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ii;
	char data;
	int bytes_printed = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);

	for (ii = 0; ii < 0x7F; ii++) {
		/* don't read fifo r/w register */
		if ((ii == REG_MEM_R_W) || (ii == REG_FIFO_R_W))
			data = 0;
		else
			inv_plat_read(st, ii, 1, &data);
		bytes_printed += snprintf(buf + bytes_printed,
				MAX_WR_SZ - bytes_printed, "%#2x: %#2x\n", ii, data);
	}
	set_inv_enable(indio_dev);
	mutex_unlock(&st->lock);

	return bytes_printed;
}

static ssize_t misc_flush_batch_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int result, data;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;

	mutex_lock(&st->lock);
	result = inv_flush_batch_data(indio_dev, data);
	mutex_unlock(&st->lock);

	return count;
}

static unsigned int inv_convert_wom_to_val(unsigned int threshold)
{
	/* 4mg per LSB converted in g in micro (1000000) */
	const unsigned int convert = 4U * 1000U;
	unsigned int value;

	value = threshold * convert;

	/* compute the differential by multiplying by the frequency */
	return value;
}

static unsigned int inv_convert_val_to_wom(unsigned int val)
{
	/* 4mg per LSB converted in g in micro (1000000) */
	const unsigned int convert = 4U * 1000U;
	unsigned int value;

	/* return 0 only if val is 0 */
	if (val == 0)
		return 0;

	value = val / convert;

	/* limit value to 8 bits and prevent 0 */
	return min(255U, max(1U, value));
}

static int inv_set_lp_odr(struct inv_mpu_state *st, unsigned int freq)
{
	static const unsigned int freqs[] = {500, 250, 125, 62, 31, 16, 8, 4};
	static const unsigned int reg_values[] = {
		INV_LPOSC_500HZ, INV_LPOSC_250HZ, INV_LPOSC_125HZ, INV_LPOSC_62HZ,
		INV_LPOSC_31HZ, INV_LPOSC_16HZ, INV_LPOSC_8HZ, INV_LPOSC_4HZ,
	};
	unsigned int val, i;

	/* by default use lowest value */
	val = INV_LPOSC_4HZ;
	/* found the nearest inferior frequency */
	for (i = 0; i < ARRAY_SIZE(freqs); ++i) {
		if (freq >= freqs[i]) {
			val = reg_values[i];
			break;
		}
	}

	return inv_plat_single_write(st, REG_LP_MODE_CFG, val);
}

static int inv_set_wom_lp(struct inv_mpu_state *st, bool on)
{
	int result;

	if (on) {
		result = inv_plat_single_write(st, REG_20680_ACCEL_CONFIG2, 0x07);
		if (result)
			return result;

		/* set low power ODR based on accel wake-up frequency */
		result = inv_set_lp_odr(st, st->sensor_l[SENSOR_L_ACCEL_WAKE].rate);
		if (result)
			return result;

		/* set cycle mode */
		result = inv_plat_single_write(st, REG_PWR_MGMT_1, BIT_LP_EN | BIT_CLK_PLL);
	} else {
		/* disable cycle mode */
		result = inv_plat_single_write(st, REG_PWR_MGMT_1, BIT_CLK_PLL);
		if (result)
			return result;

		result = inv_set_accel_config2(st, st->chip_config.lp_en_set);
	}

	return result;
}

int inv_mpu_set_wom_lp(struct inv_mpu_state *st, bool en)
{
	u8 val;
	int result;

	if (en) {
		if (st->chip_config.is_asleep || st->chip_config.lp_en_set) {
			result = inv_plat_single_write(st, REG_PWR_MGMT_1, BIT_CLK_PLL);
			if (result)
				return result;
			usleep_range(REG_UP_TIME_USEC, REG_UP_TIME_USEC + 100);
		}

		if (!st->chip_config.accel_enable) {
			result = inv_plat_read(st, REG_PWR_MGMT_2, 1, &val);
			if (result)
				return result;
			val &= ~BIT_PWR_ACCEL_STBY;
			result = inv_plat_single_write(st, REG_PWR_MGMT_2, val);
			if (result)
				return result;
			msleep(INV_IAM20680_ACCEL_START_TIME);
		}

		result = inv_plat_single_write(st, REG_ACCEL_WOM_THR, st->wom_thld);
		if (result)
			return result;

		result = inv_plat_single_write(st, REG_ACCEL_INTEL_CTRL,
				BIT_ACCEL_INTEL_EN | BIT_ACCEL_INTEL_MODE);
		if (result)
			return result;

		result = inv_plat_single_write(st, REG_INT_ENABLE, BIT_WOM_X_INT_EN | BIT_WOM_Y_INT_EN | BIT_WOM_X_INT_EN);
		if (result)
			return result;

		result = inv_set_wom_lp(st, true);
		if (result)
			return result;
	} else {
		result = inv_set_wom_lp(st, false);
		if (result)
			return result;

		result = inv_plat_single_write(st, REG_INT_ENABLE, 0x00);
		if (result)
			return result;

		result = inv_plat_single_write(st, REG_ACCEL_INTEL_CTRL, 0x00);
		if (result)
			return result;

		if (!st->chip_config.accel_enable) {
			result = inv_plat_read(st, REG_PWR_MGMT_2, 1, &val);
			if (result)
				return result;
			val |= BIT_PWR_ACCEL_STBY;
			result = inv_plat_single_write(st, REG_PWR_MGMT_2, val);
			if (result)
				return result;
		}

		if (st->chip_config.is_asleep || st->chip_config.lp_en_set) {
			val = BIT_CLK_PLL;
			val |= st->chip_config.is_asleep ? BIT_SLEEP : 0;
			val |= st->chip_config.lp_en_set ? BIT_LP_EN : 0;
			result = inv_plat_single_write(st, REG_PWR_MGMT_1, val);
			if (result)
				return result;
		}
	}

	return 0;
}

static int inv_mpu_read_event_config(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);

	if (chan->type != IIO_ACCEL || type != IIO_EV_TYPE_THRESH ||
			dir != IIO_EV_DIR_RISING)
		return -EINVAL;

	return st->chip_config.wom_on ? 1 : 0;
}

static int inv_mpu_write_event_config(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, int state)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int enable;

	if (chan->type != IIO_ACCEL || type != IIO_EV_TYPE_THRESH ||
			dir != IIO_EV_DIR_RISING)
		return -EINVAL;

	enable = !!state;
	st->chip_config.wom_on = enable;

	return 0;
}

static int inv_mpu_read_event_value(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info, int *val,
		int *val2)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);
	unsigned int value;

	/* support only WoM (accel roc rising) event value */
	if (chan->type != IIO_ACCEL || type != IIO_EV_TYPE_THRESH ||
			dir != IIO_EV_DIR_RISING || info != IIO_EV_INFO_VALUE)
		return -EINVAL;

	/* convert to value and return in micro */
	value = inv_convert_wom_to_val(st->wom_thld);
	*val = value / 1000000U;
	*val2 = value % 1000000U;

	return IIO_VAL_INT_PLUS_MICRO;
}

static int inv_mpu_write_event_value(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info,
		int val, int val2)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);
	unsigned int value;

	/* support only WoM (accel roc rising) event value */
	if (chan->type != IIO_ACCEL || type != IIO_EV_TYPE_THRESH ||
	    dir != IIO_EV_DIR_RISING || info != IIO_EV_INFO_VALUE)
		return -EINVAL;

	if (val < 0 || val2 < 0)
		return -EINVAL;

	/* convert value to wom and save it */
	value = (unsigned int)val * 1000000U + (unsigned int)val2;
	st->wom_thld = inv_convert_val_to_wom(value);

	return 0;
}

static const struct iio_event_spec inv_mpu_wom_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec inv_mpu_channels[] = {
	{
		.type = IIO_ACCEL,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 64,
			.storagebits = 64,
		},
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_X_OR_Y_OR_Z,
		.event_spec = inv_mpu_wom_events,
		.num_event_specs = ARRAY_SIZE(inv_mpu_wom_events),
		.scan_index = -1,
	},
};

/* special run time sysfs entry, read only */
static DEVICE_ATTR_RO(debug_reg_dump);
static DEVICE_ATTR_RO(out_temperature);
static DEVICE_ATTR_RO(misc_self_test);

static IIO_DEVICE_ATTR(info_anglvel_matrix, 0444, inv_attr_show, NULL,
			ATTR_GYRO_MATRIX);
static IIO_DEVICE_ATTR(info_accel_matrix, 0444, inv_attr_show, NULL,
			ATTR_ACCEL_MATRIX);

static IIO_DEVICE_ATTR(info_gyro_sf, 0444, inv_attr_show, NULL,
			ATTR_GYRO_SF);
/* write only sysfs */
static DEVICE_ATTR_WO(misc_flush_batch);

/* sensor on/off sysfs control */
static IIO_DEVICE_ATTR(in_accel_enable, 0644,
			inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_ACCEL);
static IIO_DEVICE_ATTR(in_anglvel_enable, 0644,
			inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_GYRO);
#ifndef SUPPORT_ONLY_BASIC_FEATURES
static IIO_DEVICE_ATTR(in_eis_enable, 0644,
			inv_sensor_on_show, inv_sensor_on_store,
			SENSOR_L_EIS_GYRO);
#endif
static IIO_DEVICE_ATTR(in_accel_wake_enable, 0644,
			inv_sensor_on_show, inv_sensor_on_store,
			SENSOR_L_ACCEL_WAKE);
static IIO_DEVICE_ATTR(in_anglvel_wake_enable, 0644,
			inv_sensor_on_show, inv_sensor_on_store,
			SENSOR_L_GYRO_WAKE);

/* sensor rate sysfs control */
static IIO_DEVICE_ATTR(in_accel_rate, 0644,
			inv_sensor_rate_show, inv_sensor_rate_store,
			SENSOR_L_ACCEL);
static IIO_DEVICE_ATTR(in_anglvel_rate, 0644, inv_sensor_rate_show,
			inv_sensor_rate_store, SENSOR_L_GYRO);
#ifndef SUPPORT_ONLY_BASIC_FEATURES
static IIO_DEVICE_ATTR(in_eis_rate, 0644,
			inv_sensor_rate_show, inv_sensor_rate_store,
			SENSOR_L_EIS_GYRO);
#endif
static IIO_DEVICE_ATTR(in_accel_wake_rate, 0644,
			inv_sensor_rate_show, inv_sensor_rate_store,
			SENSOR_L_ACCEL_WAKE);
static IIO_DEVICE_ATTR(in_anglvel_wake_rate, 0644,
			inv_sensor_rate_show, inv_sensor_rate_store,
			SENSOR_L_GYRO_WAKE);

static IIO_DEVICE_ATTR(misc_batchmode_timeout, 0644,
			inv_attr_show, inv_basic_attr_store,
			ATTR_DMP_BATCHMODE_TIMEOUT);

/* engine scale */
static IIO_DEVICE_ATTR(in_accel_scale, 0644, inv_attr_show,
			inv_misc_attr_store, ATTR_ACCEL_SCALE);
static IIO_DEVICE_ATTR(in_anglvel_scale, 0644, inv_attr_show,
			inv_misc_attr_store, ATTR_GYRO_SCALE);


#ifndef SUPPORT_ONLY_BASIC_FEATURES
static IIO_DEVICE_ATTR(debug_lp_en_off, 0644, inv_attr_show,
			inv_debug_store, ATTR_DMP_LP_EN_OFF);
static IIO_DEVICE_ATTR(debug_clock_sel, 0644, inv_attr_show,
			inv_debug_store, ATTR_DMP_CLK_SEL);
static IIO_DEVICE_ATTR(debug_reg_write, 0644, inv_attr_show,
			inv_debug_store, ATTR_DEBUG_REG_WRITE);
static IIO_DEVICE_ATTR(debug_reg_write_addr, 0644, inv_attr_show,
			inv_debug_store, ATTR_DEBUG_REG_ADDR);
#endif

static IIO_DEVICE_ATTR(in_accel_x_st_calibbias, 0644,
			inv_attr_show, NULL, ATTR_ACCEL_X_ST_CALIBBIAS);
static IIO_DEVICE_ATTR(in_accel_y_st_calibbias, 0644,
			inv_attr_show, NULL, ATTR_ACCEL_Y_ST_CALIBBIAS);
static IIO_DEVICE_ATTR(in_accel_z_st_calibbias, 0644,
			inv_attr_show, NULL, ATTR_ACCEL_Z_ST_CALIBBIAS);

static IIO_DEVICE_ATTR(in_anglvel_x_st_calibbias, 0644,
			inv_attr_show, NULL, ATTR_ANGLVEL_X_ST_CALIBBIAS);
static IIO_DEVICE_ATTR(in_anglvel_y_st_calibbias, 0644,
			inv_attr_show, NULL, ATTR_ANGLVEL_Y_ST_CALIBBIAS);
static IIO_DEVICE_ATTR(in_anglvel_z_st_calibbias, 0644,
			inv_attr_show, NULL, ATTR_ANGLVEL_Z_ST_CALIBBIAS);

static IIO_DEVICE_ATTR(in_accel_x_offset, 0644,
			inv_attr_show, inv_bias_store, ATTR_ACCEL_X_OFFSET);
static IIO_DEVICE_ATTR(in_accel_y_offset, 0644,
			inv_attr_show, inv_bias_store, ATTR_ACCEL_Y_OFFSET);
static IIO_DEVICE_ATTR(in_accel_z_offset, 0644,
			inv_attr_show, inv_bias_store, ATTR_ACCEL_Z_OFFSET);

static IIO_DEVICE_ATTR(in_anglvel_x_offset, 0644,
			inv_attr_show, inv_bias_store, ATTR_GYRO_X_OFFSET);
static IIO_DEVICE_ATTR(in_anglvel_y_offset, 0644,
			inv_attr_show, inv_bias_store, ATTR_GYRO_Y_OFFSET);
static IIO_DEVICE_ATTR(in_anglvel_z_offset, 0644,
			inv_attr_show, inv_bias_store, ATTR_GYRO_Z_OFFSET);

static IIO_DEVICE_ATTR(info_gyro_lp_mode, 0644,
			inv_attr_show, NULL, ATTR_GYRO_LP_MODE);
static IIO_DEVICE_ATTR(info_accel_lp_mode, 0644,
			inv_attr_show, NULL, ATTR_ACCEL_LP_MODE);

#ifndef SUPPORT_ONLY_BASIC_FEATURES
static IIO_DEVICE_ATTR(in_step_detector_enable, 0644,
			inv_attr_show, inv_basic_attr_store,
			ATTR_DMP_STEP_DETECTOR_ON);
static IIO_DEVICE_ATTR(in_step_detector_wake_enable, 0644,
			inv_attr_show, inv_basic_attr_store,
			ATTR_DMP_STEP_DETECTOR_WAKE_ON);
static IIO_DEVICE_ATTR(in_step_counter_enable, 0644, inv_attr_show,
			inv_basic_attr_store, ATTR_DMP_STEP_COUNTER_ON);
static IIO_DEVICE_ATTR(in_step_counter_wake_enable, 0644,
			inv_attr_show, inv_basic_attr_store,
			ATTR_DMP_STEP_COUNTER_WAKE_ON);

static IIO_DEVICE_ATTR(event_tilt_enable, 0644,
			inv_attr_show, inv_basic_attr_store,
			ATTR_DMP_TILT_ENABLE);

static IIO_DEVICE_ATTR(event_eis_enable, 0644,
			inv_attr_show, inv_basic_attr_store,
			ATTR_DMP_EIS_ENABLE);

static IIO_DEVICE_ATTR(event_pick_up_enable, 0644,
			inv_attr_show, inv_basic_attr_store,
			ATTR_DMP_PICK_UP_ENABLE);

static IIO_DEVICE_ATTR(in_power_on, 0644,
			inv_attr_show, inv_basic_attr_store,
			ATTR_IN_POWER_ON);
#endif

static const struct attribute *inv_raw_attributes[] = {
	&dev_attr_debug_reg_dump.attr,
	&dev_attr_out_temperature.attr,
	&dev_attr_misc_flush_batch.attr,
	&dev_attr_misc_self_test.attr,
#ifndef SUPPORT_ONLY_BASIC_FEATURES
	&iio_dev_attr_in_power_on.dev_attr.attr,
#endif
	&iio_dev_attr_in_accel_enable.dev_attr.attr,
	&iio_dev_attr_in_accel_wake_enable.dev_attr.attr,
	&iio_dev_attr_info_accel_matrix.dev_attr.attr,
	&iio_dev_attr_in_accel_scale.dev_attr.attr,
	&iio_dev_attr_misc_batchmode_timeout.dev_attr.attr,
	&iio_dev_attr_in_accel_rate.dev_attr.attr,
	&iio_dev_attr_in_accel_wake_rate.dev_attr.attr,
	&iio_dev_attr_info_accel_lp_mode.dev_attr.attr,
};

#ifndef SUPPORT_ONLY_BASIC_FEATURES
static const struct attribute *inv_debug_attributes[] = {
	&iio_dev_attr_debug_lp_en_off.dev_attr.attr,
	&iio_dev_attr_debug_clock_sel.dev_attr.attr,
	&iio_dev_attr_debug_reg_write.dev_attr.attr,
	&iio_dev_attr_debug_reg_write_addr.dev_attr.attr,
};
#endif

static const struct attribute *inv_gyro_attributes[] = {
	&iio_dev_attr_info_anglvel_matrix.dev_attr.attr,
	&iio_dev_attr_in_anglvel_enable.dev_attr.attr,
	&iio_dev_attr_in_anglvel_rate.dev_attr.attr,
#ifndef SUPPORT_ONLY_BASIC_FEATURES
	&iio_dev_attr_in_eis_enable.dev_attr.attr,
#endif
	&iio_dev_attr_in_anglvel_wake_enable.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale.dev_attr.attr,
#ifndef SUPPORT_ONLY_BASIC_FEATURES
	&iio_dev_attr_in_eis_rate.dev_attr.attr,
#endif
	&iio_dev_attr_in_anglvel_wake_rate.dev_attr.attr,
	&iio_dev_attr_info_gyro_sf.dev_attr.attr,
	&iio_dev_attr_info_gyro_lp_mode.dev_attr.attr,
};

static const struct attribute *inv_bias_attributes[] = {
	&iio_dev_attr_in_accel_x_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_accel_y_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_accel_z_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_accel_x_offset.dev_attr.attr,
	&iio_dev_attr_in_accel_y_offset.dev_attr.attr,
	&iio_dev_attr_in_accel_z_offset.dev_attr.attr,
	&iio_dev_attr_in_anglvel_x_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_y_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_z_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_x_offset.dev_attr.attr,
	&iio_dev_attr_in_anglvel_y_offset.dev_attr.attr,
	&iio_dev_attr_in_anglvel_z_offset.dev_attr.attr,
};

#ifndef SUPPORT_ONLY_BASIC_FEATURES
static const struct attribute *inv_pedometer_attributes[] = {
	&iio_dev_attr_event_tilt_enable.dev_attr.attr,
	&iio_dev_attr_event_eis_enable.dev_attr.attr,
	&iio_dev_attr_event_pick_up_enable.dev_attr.attr,
	&iio_dev_attr_in_step_counter_enable.dev_attr.attr,
	&iio_dev_attr_in_step_counter_wake_enable.dev_attr.attr,
	&iio_dev_attr_in_step_detector_enable.dev_attr.attr,
	&iio_dev_attr_in_step_detector_wake_enable.dev_attr.attr,
};
#endif

static struct attribute *inv_attributes[ARRAY_SIZE(inv_raw_attributes) +
#ifndef SUPPORT_ONLY_BASIC_FEATURES
					ARRAY_SIZE(inv_debug_attributes) +
#endif
					ARRAY_SIZE(inv_gyro_attributes) +
					ARRAY_SIZE(inv_bias_attributes) +
#ifndef SUPPORT_ONLY_BASIC_FEATURES
					ARRAY_SIZE(inv_pedometer_attributes) +
#endif
					 + 1];

static const struct attribute_group inv_attribute_group = {
	.name = "mpu",
	.attrs = inv_attributes
};

static const struct iio_info mpu_info = {
#if KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE
	.driver_module = THIS_MODULE,
#endif
	.attrs = &inv_attribute_group,
	.read_event_config =	inv_mpu_read_event_config,
	.write_event_config =	inv_mpu_write_event_config,
	.read_event_value =	inv_mpu_read_event_value,
	.write_event_value =	inv_mpu_write_event_value,


};

/*
 *  inv_check_chip_type() - check and setup chip type.
 */
int inv_check_chip_type(struct iio_dev *indio_dev, const char *name)
{
	int result;
	int t_ind;
	struct inv_chip_config_s *conf;
	struct mpu_platform_data *plat;
	struct inv_mpu_state *st;

	st = iio_priv(indio_dev);
	conf = &st->chip_config;
	plat = &st->plat_data;

	if (!strcmp(name, "iam20680"))
		st->chip_type = IAM20680;
	else
		return -EPERM;
	st->chip_config.has_gyro = 1;

	st->hw = &hw_info[st->chip_type];
	result = inv_mpu_initialize(st);
	if (result)
		return result;

	t_ind = 0;
	memcpy(&inv_attributes[t_ind], inv_raw_attributes,
				sizeof(inv_raw_attributes));
	t_ind += ARRAY_SIZE(inv_raw_attributes);

#ifndef SUPPORT_ONLY_BASIC_FEATURES
	memcpy(&inv_attributes[t_ind], inv_pedometer_attributes,
				sizeof(inv_pedometer_attributes));
	t_ind += ARRAY_SIZE(inv_pedometer_attributes);
#endif

	memcpy(&inv_attributes[t_ind], inv_gyro_attributes,
				sizeof(inv_gyro_attributes));
	t_ind += ARRAY_SIZE(inv_gyro_attributes);

	memcpy(&inv_attributes[t_ind], inv_bias_attributes,
				sizeof(inv_bias_attributes));
	t_ind += ARRAY_SIZE(inv_bias_attributes);

#ifndef SUPPORT_ONLY_BASIC_FEATURES
	memcpy(&inv_attributes[t_ind], inv_debug_attributes,
				sizeof(inv_debug_attributes));
	t_ind += ARRAY_SIZE(inv_debug_attributes);
#endif

	inv_attributes[t_ind] = NULL;

	indio_dev->name = st->hw->name;
	indio_dev->channels = inv_mpu_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_mpu_channels);

	indio_dev->info = &mpu_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	return result;
}
EXPORT_SYMBOL_GPL(inv_check_chip_type);

int inv_create_dmp_sysfs(struct iio_dev *ind)
{
	// dummy
	return 0;
}
EXPORT_SYMBOL_GPL(inv_create_dmp_sysfs);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device ICM20xxx driver");
MODULE_LICENSE("GPL");
