// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/timekeeping.h>

#include "ssaiot_sc.h"

/* ensure u8 (protocol data-len field) can never overrun buffers */
static_assert(SSAIOT_SC_CMD_MAX_DATA_LEN >= U8_MAX);
static_assert(SSAIOT_SC_RESP_MAX_DATA_LEN >= U8_MAX);

/**
 * ssaiot_sc_xfer() - Execute one system controller command
 * @priv: Driver private structure
 * @cmd: Command code (%SSAIOT_SC_CMD_*)
 * @sensor_id: Target sensor/module (%SSAIOT_SC_SENSOR_*)
 * @tx: Command payload, may be %NULL if @tx_len is 0
 * @tx_len: Command payload length, becomes the DATA_LEN header byte
 * @rx: Buffer for the response payload, may be %NULL if @rx_size is 0
 * @rx_size: How many payload bytes to read into @rx, excluding the STATUS and
 *	     DATA_LEN bytes. The read is always exactly this long.
 * @rx_data_len: Where to store the response's own DATA_LEN, or %NULL to require
 *		 it to equal @rx_size
 * @status: Where to store the in-band STATUS byte, or %NULL to have any status
 *	    other than %SSAIOT_SC_STATUS_OK reported as -EIO
 * @ts: Where to store a CLOCK_REALTIME stamp of when the transfer began, or
 *	%NULL if the caller does not need one
 *
 * Issues the 3-byte command header plus @tx_len payload bytes, then reads
 * %SSAIOT_SC_RESP_HDR_LEN + @rx_size bytes back in the same I2C transfer
 * (write, repeated START, read). The controller executes the command when the
 * write completes and stretches SCL until the response is armed, so no delay
 * between the two phases is required.
 *
 * The controller arms every response at its full 257-byte transmit buffer -
 * STATUS, DATA_LEN and 255 of payload - so any @rx_size completes and need not
 * match what the command produces. Only a read past 257 bytes leaves the
 * controller with nothing to send, and it then keeps stretching SCL until its
 * watchdog fires. @rx_size is a u8, so this function cannot ask for that much.
 * Bytes the controller had ready beyond the read are discarded when the next
 * command arrives.
 *
 * How much of that fixed read is meaningful is a separate question, and
 * @rx_data_len is how a caller asks. Most commands fill the read exactly and
 * pass %NULL, which asks for the strict check: any other length is a firmware
 * or caller bug and is rejected rather than handed out half filled, because the
 * tail of a short response is whatever the controller had left over from the
 * previous command. Callers that can work with another length - a drain that
 * returns fewer records than the buffer holds, or a payload a later firmware
 * has extended past what this driver knows - pass @rx_data_len and are told
 * what the controller meant to send, which may be more than @rx_size. @rx
 * receives all @rx_size bytes that were read either way, of which the first
 * min(@rx_size, DATA_LEN) are the response.
 *
 * @ts is for callers timing what the controller reports against a host clock.
 * It is taken once the bus segment is held rather than on the way in, so that
 * waiting for another user of the bus does not affect it.
 *
 * Return: 0 on success, negative errno on failure.
 */
int ssaiot_sc_xfer(struct ssaiot_sc_priv *priv, u8 cmd, u8 sensor_id,
		   const u8 *tx, u8 tx_len, u8 *rx, u8 rx_size,
		   u8 *rx_data_len, u8 *status, s64 *ts)
{
	u8 tx_buf[SSAIOT_SC_CMD_HDR_LEN + SSAIOT_SC_CMD_MAX_DATA_LEN];
	u8 rx_buf[SSAIOT_SC_RESP_HDR_LEN + SSAIOT_SC_RESP_MAX_DATA_LEN];
	struct i2c_client *client = to_i2c_client(priv->dev);
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = SSAIOT_SC_CMD_HDR_LEN + tx_len,
			.buf = tx_buf,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = SSAIOT_SC_RESP_HDR_LEN + rx_size,
			.buf = rx_buf,
		},
	};
	u8 resp_status, resp_len;
	int ret;

	if ((tx_len && !tx) || (rx_size && !rx))
		return -EINVAL;

	tx_buf[0] = cmd;
	tx_buf[1] = sensor_id;
	tx_buf[2] = tx_len;
	if (tx_len)
		memcpy(&tx_buf[SSAIOT_SC_CMD_HDR_LEN], tx, tx_len);

	/*
	 * One transfer, so the core holds the bus across write, repeated START
	 * and read. That is what keeps the controller's single global response
	 * buffer from being observed by anyone but the caller that armed it -
	 * concurrent callers need no further serialisation here.
	 *
	 * Because the power-off handler goes through here too, i2c_lock_bus()
	 * may not be used unconditionally: that caller runs with interrupts off
	 * and the other CPUs stopped, where the lock has to be tried rather than
	 * waited on. i2c_transfer() picks between the two itself, through a
	 * helper the i2c core keeps private, so the same choice cannot be made
	 * here.
	 *
	 * Take the lock by hand only where a stamp has to be read inside it, and
	 * let i2c_transfer() handle the locking otherwise.
	 */
	if (ts) {
		i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
		*ts = ktime_get_real_ns();
		ret = __i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	} else {
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	}
	if (ret < 0) {
		dev_err_ratelimited(priv->dev,
				    "transfer failed for cmd 0x%02x sensor 0x%02x: %d.\n",
				    cmd, sensor_id, ret);
		return ret;
	}
	if (ret != ARRAY_SIZE(msg))
		return -EIO;

	resp_status = rx_buf[0];
	resp_len = rx_buf[1];

	/*
	 * A mismatch on a fixed-length command means either the caller's
	 * expectation is wrong or the firmware does not implement it. A caller
	 * that took the length decides for itself what to make of one.
	 */
	if (!rx_data_len && resp_len != rx_size) {
		dev_err_ratelimited(priv->dev,
				    "unexpected response length for cmd 0x%02x sensor 0x%02x: "
				    "got %u, expected %u.\n",
				    cmd, sensor_id, resp_len, rx_size);
		return -EPROTO;
	}

	/*
	 * A caller that does not take the status byte cannot act on the
	 * distinctions it draws, so anything but OK becomes a plain failure.
	 * Callers that need to tell a stale reading from a rejected command
	 * take it themselves.
	 */
	if (!status && resp_status != SSAIOT_SC_STATUS_OK)
		return -EIO;

	/* validation done, nothing below can fail */
	if (rx_data_len)
		*rx_data_len = resp_len;

	if (status)
		*status = resp_status;

	if (rx_size)
		memcpy(rx, &rx_buf[SSAIOT_SC_RESP_HDR_LEN], rx_size);

	return 0;
}
EXPORT_SYMBOL_GPL(ssaiot_sc_xfer);
