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
#include <linux/math64.h>
#include <linux/iio/events.h>

#include "../inv_mpu_iio.h"

static int inv_check_fsync(struct inv_mpu_state *st)
{
	u8 data[1];

	if (!st->chip_config.eis_enable)
		return 0;
	inv_plat_read(st, REG_FSYNC_INT, 1, data);
	if (data[0] & BIT_FSYNC_INT) {
		pr_debug("fsync\n");
		st->eis.eis_triggered = true;
		st->eis.fsync_delay = 1;
		st->eis.prev_state = 1;
		st->eis.frame_count++;
		st->eis.eis_frame = true;
	}
	st->header_count--;

	return 0;
}

static int inv_push_sensor(struct iio_dev *indio_dev, int ind, u64 t, u8 *d)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int i;
	s32 raw[3];
	s32 calib[3] = { 0, 0, 0 };

	switch (ind) {
	case SENSOR_ACCEL:
		for (i = 0; i < 3; i++)
			raw[i] = (s16)be16_to_cpup((__be16 *) (d + i * 2));
		inv_push_16bytes_buffer(indio_dev, ind, t, raw, 0);
		break;
	case SENSOR_TEMP:
		inv_check_fsync(st);
		break;
	case SENSOR_GYRO:
		for (i = 0; i < 3; i++)
			raw[i] = (s16)be16_to_cpup((__be16 *) (d + i * 2));
		inv_push_gyro_data(indio_dev, raw, calib, t);
		break;
	default:
		break;
	}

	return 0;
}

static int inv_push_20680_data(struct iio_dev *indio_dev, u8 *d)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);
	u8 *dptr;
	int i;

	dptr = d;

	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			inv_get_dmp_ts(st, i);
			if (st->sensor[i].send && (!st->ts_algo.first_sample)) {
				st->sensor[i].sample_calib++;
				inv_push_sensor(indio_dev, i, st->sensor[i].ts, dptr);
			}
			dptr += st->sensor[i].sample_size;
		}
	}
	if (st->ts_algo.first_sample)
		st->ts_algo.first_sample--;
	st->header_count--;

	return 0;
}

static int inv_process_20680_data(struct iio_dev *indio_dev)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int total_bytes, tmp, res, fifo_count, pk_size, i;
	u8 *dptr, *d;
	bool done_flag;
#ifdef SENSOR_DATA_FROM_REGISTERS
	u8 reg;
	int len;
#endif

	fifo_count = inv_get_last_run_time_non_dmp_record_mode(st);
	pr_debug("fifc= %d\n", fifo_count);
	if (!fifo_count) {
		pr_debug("REG_FIFO_COUNT_H size is 0\n");
		return 0;
	}
	pk_size = st->batch.pk_size;
	if (!pk_size)
		return -EINVAL;

	if (fifo_count >= (HARDWARE_FIFO_SIZE / st->batch.pk_size)) {
		pr_warn("fifo overflow pkt count=%d pkt sz=%d\n",
				fifo_count, st->batch.pk_size);
		return -EOVERFLOW;
	}

	fifo_count *= st->batch.pk_size;
	st->fifo_count = fifo_count;
	d = st->fifo_data_store;
	dptr = d;
	total_bytes = fifo_count;

#ifdef SENSOR_DATA_FROM_REGISTERS
	len = 0;
	if (st->sensor[SENSOR_GYRO].on) {
		reg = REG_RAW_GYRO;
		len += BYTES_PER_SENSOR;
		if (st->sensor[SENSOR_ACCEL].on && !st->sensor[SENSOR_TEMP].on)
			len += BYTES_FOR_TEMP;
	}
	if (st->sensor[SENSOR_TEMP].on) {
		reg = REG_RAW_TEMP;
		len += BYTES_FOR_TEMP;
	}
	if (st->sensor[SENSOR_ACCEL].on) {
		reg = REG_RAW_ACCEL;
		len += BYTES_PER_SENSOR;
	}

	if (len == 0) {
		pr_debug("No sensor is enabled\n");
		return 0;
	}

	/* read data registers */
	res = inv_plat_read(st, reg, len, data);
	if (res < 0) {
		pr_err("read data registers is failed\n");
		return res;
	}

	/* copy sensor data to buffer as FIFO data format */
	tmp = 0;
	if (st->sensor[SENSOR_ACCEL].on) {
		for (i = 0; i < BYTES_PER_SENSOR; i++)
			dptr[i] = data[tmp + i];
		dptr += BYTES_PER_SENSOR;
		tmp += BYTES_PER_SENSOR;
	}

	if (st->sensor[SENSOR_TEMP].on) {
		for (i = 0; i < BYTES_FOR_TEMP; i++)
			dptr[i] = data[tmp + i];
		dptr += BYTES_FOR_TEMP;
		tmp += BYTES_FOR_TEMP;
	}

	if (st->sensor[SENSOR_GYRO].on) {
		if (st->sensor[SENSOR_ACCEL].on && !st->sensor[SENSOR_TEMP].on)
			tmp += BYTES_FOR_TEMP;
		for (i = 0; i < BYTES_PER_SENSOR; i++)
			dptr[i] = data[tmp + i];
	}
#else
	while (total_bytes > 0) {
		if (total_bytes < pk_size * MAX_FIFO_PACKET_READ)
			tmp = total_bytes;
		else
			tmp = pk_size * MAX_FIFO_PACKET_READ;
		res = inv_plat_read(st, REG_FIFO_R_W, tmp, dptr);
		if (res < 0) {
			pr_err("read REG_FIFO_R_W is failed\n");
			return res;
		}
		pr_debug("inside: %x, %x, %x, %x, %x, %x, %x, %x\n",
				dptr[0], dptr[1], dptr[2], dptr[3],
				dptr[4], dptr[5], dptr[6], dptr[7]);
		pr_debug("insid2: %x, %x, %x, %x, %x, %x, %x, %x\n",
				dptr[8], dptr[9], dptr[10], dptr[11],
				dptr[12], dptr[13], dptr[14], dptr[15]);

		dptr += tmp;
		total_bytes -= tmp;
	}
#endif /* SENSOR_DATA_FROM_REGISTERS */
	dptr = d;
	pr_debug("dd: %x, %x, %x, %x, %x, %x, %x, %x\n",
			d[0], d[1], d[2], d[3],
			d[4], d[5], d[6], d[7]);
	pr_debug("dd2: %x, %x, %x, %x, %x, %x, %x, %x\n",
			d[8], d[9], d[10], d[11],
			d[12], d[13], d[14], d[15]);
	total_bytes = fifo_count;

	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on)
			st->sensor[i].count =  total_bytes / pk_size;
	}
	st->header_count = 0;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on)
			st->header_count = max(st->header_count,
							st->sensor[i].count);
	}

	st->ts_algo.calib_counter++;
	inv_bound_timestamp(st);

	dptr = d;
	done_flag = false;

	while (!done_flag) {
		pr_debug("total%d, pk=%d\n", total_bytes, pk_size);
		if (total_bytes >= pk_size) {
			res = inv_push_20680_data(indio_dev, dptr);
			if (res)
				return res;
			total_bytes -= pk_size;
			dptr += pk_size;
		} else {
			done_flag = true;
		}
	}

	return 0;
}

/*
 *  _inv_read_fifo() - Transfer data from FIFO to ring buffer.
 */
static void _inv_read_fifo(struct iio_dev *indio_dev)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int result;

	result = wait_event_interruptible_timeout(st->wait_queue,
					st->resume_state, msecs_to_jiffies(300));
	if (result <= 0)
		return;
	mutex_lock(&st->lock);
#ifdef TIMER_BASED_BATCHING
	if (st->batch_timeout) {
		if (inv_plat_single_write(st, REG_INT_ENABLE, st->int_en))
			pr_err("REG_INT_ENABLE write error\n");
	}
#endif
	st->wake_sensor_received = false;
	result = inv_process_20680_data(indio_dev);
	if (result)
		goto err_reset_fifo;
	mutex_unlock(&st->lock);

	if (st->wake_sensor_received)
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_timeout(&st->wake_lock, msecs_to_jiffies(200));
#else
		__pm_wakeup_event(st->wake_lock, 200); /* 200 msecs */
#endif
	return;

err_reset_fifo:
	if ((!st->chip_config.gyro_enable) &&
		(!st->chip_config.accel_enable) &&
		(!st->chip_config.slave_enable) &&
		(!st->chip_config.pressure_enable)) {
		inv_switch_power_in_lp(st, false);
		mutex_unlock(&st->lock);

		return;
	}

	pr_err("error to reset fifo\n");
	inv_switch_power_in_lp(st, true);
	inv_reset_fifo(st, true);
	inv_switch_power_in_lp(st, false);
	mutex_unlock(&st->lock);
}

irqreturn_t inv_read_fifo(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;

	_inv_read_fifo(indio_dev);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

bool inv_mpu_interrupt_handler(struct iio_dev *indio_dev, s64 timestamp)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);
	u8 status;
	u64 wom_code;
	bool data_ready;
	int res;

	mutex_lock(&st->lock);

	res = inv_plat_read(st, REG_INT_STATUS, 1, &status);
	if (res) {
		pr_err("interrupt status register reading error %d\n", res);
		data_ready = false;
		goto exit;
	}

	/* handle WoM event */
	if (status & BIT_WOM_ALL_INT) {
		wom_code = IIO_MOD_EVENT_CODE(IIO_ACCEL, 0, IIO_MOD_X_OR_Y_OR_Z,
				IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING);
		iio_push_event(indio_dev, wom_code, timestamp);
	}

	/* handle data ready bit */
	data_ready = (status & BIT_DATA_RDY_INT) ? true : false;

exit:
	mutex_unlock(&st->lock);
	return data_ready;
}

#ifdef TIMER_BASED_BATCHING
void inv_batch_work(struct work_struct *work)
{
	struct inv_mpu_state *st =
		container_of(work, struct inv_mpu_state, batch_work);

	mutex_lock(&st->lock);
	if (inv_plat_single_write(st, REG_INT_ENABLE, st->int_en | BIT_DATA_RDY_EN))
		pr_err("REG_INT_ENABLE write error\n");
	mutex_unlock(&st->lock);
}
#endif

int inv_flush_batch_data(struct iio_dev *indio_dev, int data)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);

#ifndef SENSOR_DATA_FROM_REGISTERS
	if (st->chip_config.gyro_enable ||
		st->chip_config.accel_enable ||
		st->chip_config.slave_enable ||
		st->chip_config.pressure_enable) {
		st->wake_sensor_received = false;
		inv_process_20680_data(indio_dev);
		if (st->wake_sensor_received)
#ifdef CONFIG_HAS_WAKELOCK
			wake_lock_timeout(&st->wake_lock, msecs_to_jiffies(200));
#else
			__pm_wakeup_event(st->wake_lock, 200); /* 200 msecs */
#endif
		inv_switch_power_in_lp(st, false);
	}
#endif /* SENSOR_DATA_FROM_REGISTERS */
	inv_push_marker_to_buffer(indio_dev, END_MARKER, data);

	return 0;
}
