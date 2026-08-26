// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

#include <linux/i2c.h>
#include <linux/string.h>

#include "ssaiot_sc.h"

/**
 * ssaiot_sc_xfer() - Execute one system controller command
 * @priv: Driver private structure
 * @cmd: Command code (%SSAIOT_SC_CMD_*)
 * @sensor_id: Target sensor/module (%SSAIOT_SC_SENSOR_*)
 * @tx: Command payload, may be %NULL if @tx_len is 0
 * @tx_len: Command payload length, becomes the DATA_LEN header byte
 * @rx: Buffer for the response payload, may be %NULL if @rx_len is 0
 * @rx_len: Response payload length, excluding the STATUS and DATA_LEN bytes
 * @status: Where to store the in-band STATUS byte
 *
 * Issues the 3-byte command header plus @tx_len payload bytes, then reads
 * %SSAIOT_SC_RESP_HDR_LEN + @rx_len bytes back in the same I2C transfer
 * (write, repeated START, read). The controller executes the command when the
 * write completes and stretches SCL until the response is armed, so no delay
 * between the two phases is required.
 *
 * @rx_len must match exactly what the firmware arms for this command - see the
 * response length table in the firmware README. The controller has nothing to
 * send past its armed length and keeps stretching SCL if asked for more, which
 * wedges the bus until its stuck-bus watchdog fires roughly ten seconds later.
 * Reading short is tolerated by the firmware but is reported here as -EPROTO.
 *
 * Return: 0 on success, negative errno on failure.
 */
int ssaiot_sc_xfer(struct ssaiot_sc_priv *priv, u8 cmd, u8 sensor_id,
		   const u8 *tx, u8 tx_len, u8 *rx, u8 rx_len, u8 *status)
{
	u8 tx_buf[SSAIOT_SC_CMD_HDR_LEN + SSAIOT_SC_MAX_DATA_LEN];
	u8 rx_buf[SSAIOT_SC_RESP_HDR_LEN + SSAIOT_SC_MAX_DATA_LEN];
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
			.len = SSAIOT_SC_RESP_HDR_LEN + rx_len,
			.buf = rx_buf,
		},
	};
	u8 resp_status, resp_len;
	int ret;

	if (tx_len > SSAIOT_SC_MAX_DATA_LEN || rx_len > SSAIOT_SC_MAX_DATA_LEN)
		return -EMSGSIZE;

	if ((tx_len && !tx) || (rx_len && !rx) || !status)
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
	 */
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
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
	 * The response length is fixed per command, so a mismatch means either
	 * the caller's expectation is wrong or the firmware does not implement
	 * this command. Reject it rather than hand out a partially filled
	 * buffer - the tail of a short response is whatever the controller had
	 * left over from the previous command.
	 */
	if (resp_len != rx_len) {
		dev_err_ratelimited(priv->dev,
				    "unexpected response length for cmd 0x%02x sensor 0x%02x: "
				    "got %u, expected %u.\n",
				    cmd, sensor_id, resp_len, rx_len);
		return -EPROTO;
	}

	*status = resp_status;

	if (rx_len)
		memcpy(rx, &rx_buf[SSAIOT_SC_RESP_HDR_LEN], rx_len);

	return 0;
}
EXPORT_SYMBOL_GPL(ssaiot_sc_xfer);
