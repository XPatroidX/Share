/**
 * @file ublox_neo_m8.c
 * @author Lorenzo Thomas Contessa <lorenzocontessa.dev@gmail.com>
 * @date 11/05/2023
 * @copyright 2023 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#define DT_DRV_COMPAT u_blox_neom8

#include <ctype.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include <drivers/gnss/ublox_neo_m8.h>
#include <drivers/gnss/bits.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(neom8, CONFIG_NEOM8_LOG_LEVEL);

static int neom8_init(const struct device *dev)
{
	struct neom8_data *data = dev->data;
	const struct neom8_config *cfg = dev->config;
	int rc = 0;

	k_sem_init(&data->lock, 0, 1);

	if (!device_is_ready(cfg->i2c_dev)) {
		LOG_ERR("I2C device %s is not ready", cfg->i2c_dev->name);
		return -ENODEV;
		goto out;
	}

	data->time.hour = 0;
	data->time.min = 0;
	data->time.sec = 0;
	data->longitude_deg = 0;
	data->latitude_deg = 0;
	data->altitude = 0;
	data->ind_latitude = 'A';
	data->ind_longitude = 'A';
	data->satellites = 0;
out:
	k_sem_give(&data->lock);

	return rc;
};

static int read_register(const struct device *dev, uint8_t addr, char *buffer)
{
	const struct neom8_config *cfg = dev->config;

	int rc = i2c_write_read(cfg->i2c_dev, cfg->i2c_addr, &addr, sizeof(addr), buffer, 1);

	return rc;
}

static int neom8_get_available(const struct device *dev)
{
	uint8_t high_byte, low_byte;
	int rc;

	rc = read_register(dev, NBYTES_HIGH_ADDR, &high_byte);
	if (rc) {
		LOG_ERR("Failed to read number of bytes HIGH from %s", dev->name);
		return -1;
	}

	rc = read_register(dev, NBYTES_LOW_ADDR, &low_byte);
	if (rc) {
		LOG_ERR("Failed to read number of bytes LOW from %s", dev->name);
		return -1;
	}

	if (high_byte == 0xff && low_byte == 0xff) {
		return -1;
	}

	return (high_byte << 8) | low_byte;
}

static int ubx_msg_nav_pvt(const struct device *dev, unsigned char *buf, size_t data_len) {
	struct neom8_data *data = dev->data;

    uint8_t valid;
    uint8_t flags;
    uint8_t fixType;

    //gps_mask_t mask = 0;

    if (84 > data_len) {
		printk("Runt NAV-PVT message, payload len %zd\n", data_len);
        return -1;
    }

    //itow = getleu32(buf, 0);
    valid = (unsigned int) getub(buf, 11);
    fixType = (unsigned char) getub(buf, 20);
    flags = (unsigned int) getub(buf, 21);

	data->fixType = fixType;

    if ((valid & UBX_NAV_PVT_VALID_DATE_TIME) == UBX_NAV_PVT_VALID_DATE_TIME) {
		data->date.year = (uint16_t) getleu16(buf, 4);
		data->date.month = (uint8_t) getub(buf, 6);
		data->date.day = (uint8_t) getub(buf, 7);

        data->time.hour = (uint8_t) getub(buf, 8);
        data->time.min = (uint8_t) getub(buf, 9);
        data->time.sec = (uint8_t) getub(buf, 10);
    }

	data->latitude_deg = 1e-7 * getles32(buf, 28);
	data->longitude_deg = 1e-7 * getles32(buf, 24);
	data->altitude = 1e-3 * getles32(buf, 36);
	data->satellites = (uint8_t) getub(buf, 23);

    //return mask;
	return 0;
}

static int neom8_fetch_data(const struct device *dev) {	
	struct neom8_data *data = dev->data;
	uint16_t n_bytes;
	uint8_t c;
	int rc;

	k_sem_take(&data->lock, K_FOREVER);

	n_bytes = neom8_get_available(dev);

	if (n_bytes > 255) {
		n_bytes = 255;
	}

	uint8_t buf[n_bytes];
	size_t dim = n_bytes;
	size_t i = 0;

	while (n_bytes) {
		rc = read_register(dev, DATA_STREAM_ADDR, &c);

		if (rc) {
			LOG_ERR("Failed to read data stream from %s", dev->name);
			goto out;
		}

		buf[i++] = c;
		n_bytes--;
	}

	if (dim < UBX_PREFIX_LEN) {
        rc = 0;
		goto out;
	}

	uint16_t msgid = (buf[2] << 8) | buf[3];
    size_t data_len = (size_t) getles16(buf, 4);

	switch (msgid) {
		case UBX_NAV_PVT: {
			printk("Dimensione: %d\n", data_len);
			ubx_msg_nav_pvt(dev, &buf[UBX_PREFIX_LEN], data_len);
			break;
		}
		default:
			break;
	}
	
	goto out;

out:
	k_sem_give(&data->lock);

	return rc;
}

static int write_register(const struct device *dev, char *buffer, uint16_t length) {
	const struct neom8_config *cfg = dev->config;
	uint8_t data[length];
	int rc = 0;

	memcpy(data, buffer, length);

	rc = i2c_write(cfg->i2c_dev, data, sizeof(data), cfg->i2c_addr);
	return rc;
}

static int neom8_send_ubx(const struct device *dev, uint8_t class, uint8_t id, uint8_t payload[],
			  uint16_t length) {
	struct neom8_data *data = dev->data;
	uint8_t ckA = 0;
	uint8_t ckB = 0;
	uint8_t c;
	uint8_t response[10];
	int rc;

	const unsigned int cmdLength = 8 + length;
	uint8_t cmd[cmdLength];

	cmd[0] = 0xb5;
	cmd[1] = 0x62;
	cmd[2] = class;
	cmd[3] = id;
	cmd[4] = (length & 0xff);
	cmd[5] = (length >> 8);
	memcpy(&cmd[6], payload, length);

	for (unsigned int i = 2; i < (cmdLength - 2); i++) {
		ckA += cmd[i];
		ckB += ckA;
	}

	cmd[cmdLength - 2] = ckA;
	cmd[cmdLength - 1] = ckB;

	k_sem_take(&data->lock, K_FOREVER);

	rc = write_register(dev, cmd, cmdLength);
	if (rc) {
		LOG_ERR("Failed sending UBX frame to %s", dev->name);
		goto out;
	}

	// k_msleep(200); - Rimosso, da testare senza delay
	goto out;
	
	while (0) {
		rc = read_register(dev, DATA_STREAM_ADDR, &c);

		if (rc) {
			LOG_ERR("Failed to read data stream from %s", dev->name);
			goto out;
		}

		if (c == 0xB5) {
			response[0] = c;

			for (int i = 1; i < 10; i++) {
				rc = read_register(dev, DATA_STREAM_ADDR, &c);

				if (rc) {
					LOG_ERR("Failed to read data stream from %s",
						dev->name);
					goto out;
				}

				response[i] = c;
			}

			break;
		}
	}

	if (response[3] == 0x00) {
		rc = NACK;
	} else if (response[3] == 0x01) {
		rc = ACK;
	}

out:
	k_sem_give(&data->lock);
	neom8_fetch_data(dev);

	return rc;
}

static int neom8_set_dm(const struct device *dev, gnss_mode_t mode) {
	struct neom8_api* neo_api = (struct neom8_api *) dev->api;

	uint8_t gps_cfg[36] = { 0 };
	gps_cfg[0] = 0x00;
	gps_cfg[1] = 0x01;
	gps_cfg[2] = mode; // Dynamic Model 0x08 - Airborne 4G

	return neo_api->send_ubx(dev, 0x06, 0x24, gps_cfg, 36);
}


static float neom8_get_latitude(const struct device *dev) {
	struct neom8_data *data = dev->data;
	return data->latitude_deg;
}

static float neom8_get_longitude(const struct device *dev) {
	struct neom8_data *data = dev->data;
	return data->longitude_deg;
}

static float neom8_get_altitude(const struct device *dev) {
	struct neom8_data *data = dev->data;
	return data->altitude;
}

static struct gpsTime neom8_get_time(const struct device *dev) {
	struct neom8_data *data = dev->data;
	return data->time;
}

static struct gpsDate neom8_get_date(const struct device *dev) {
	struct neom8_data *data = dev->data;
	return data->date;
}

static uint8_t neom8_get_satellites(const struct device *dev) {
	struct neom8_data *data = dev->data;
	return data->satellites;
}

static uint8_t neom8_get_fix(const struct device *dev) {
	struct neom8_data *data = dev->data;
	return data->fixType;
}

static const struct neom8_api neom8_api = {
	.fetch_data = neom8_fetch_data,
	.send_ubx = neom8_send_ubx,
	.set_dm = neom8_set_dm,

	.get_fix = neom8_get_fix,
	.get_altitude = neom8_get_altitude,
	.get_latitude = neom8_get_latitude,
	.get_longitude = neom8_get_longitude,
	.get_time = neom8_get_time,
	.get_date = neom8_get_date,
	.get_satellites = neom8_get_satellites
};

#if CONFIG_NEOM8_INIT_PRIORITY <= CONFIG_I2C_INIT_PRIORITY
#error CONFIG_NEOM8_INIT_PRIORITY must be greater than I2C_INIT_PRIORITY
#endif

#define NEOM8_INIT(n)                                                                              \
	static struct neom8_data neom8_data_##n;                                                   \
                                                                                                   \
	static const struct neom8_config neom8_config_##n = {                                      \
		.i2c_dev = DEVICE_DT_GET(DT_INST_BUS(n)), .i2c_addr = DT_INST_REG_ADDR(n)          \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, neom8_init, NULL, &neom8_data_##n, &neom8_config_##n,             \
			      POST_KERNEL, CONFIG_NEOM8_INIT_PRIORITY, &neom8_api);

DT_INST_FOREACH_STATUS_OKAY(NEOM8_INIT);