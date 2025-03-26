/**
 * @file ublox_neo_m8.h
 * @author Lorenzo Thomas Contessa <lorenzocontessa.dev@gmail.com>
 * @date 11/05/2023
 * @copyright 2023 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GNSS_UBLOX_NEO_M8_H_
#define ZEPHYR_INCLUDE_DRIVERS_GNSS_UBLOX_NEO_M8_H_

#define UBX_PREFIX_LEN              6
#define UBX_NAV_PVT_VALID_DATE      0x01
#define UBX_NAV_PVT_VALID_TIME      0x02
#define UBX_NAV_PVT_VALID_DATE_TIME (UBX_NAV_PVT_VALID_DATE | UBX_NAV_PVT_VALID_TIME)

#define UBX_MSGID(cls_, id_) (((cls_)<<8)|(id_))

enum interface_description {
	NBYTES_HIGH_ADDR = 0xFD,
	NBYTES_LOW_ADDR = 0xFE,
	DATA_STREAM_ADDR = 0xFF,

	MAX_NMEA_SIZE = 83,
	MAX_PAYLOAD_SIZE = 256,

	UBX_SEC_UNIQID = 0x03,

	UBX_UPD_SOS = 0x14,
};

enum ack_nack_return_codes {
	NACK = 150,
	ACK = 151,
};

typedef enum {
    UBX_MODE_NOFIX  = 0x00,	/* no fix available */
    UBX_MODE_DR	    = 0x01,	/* Dead reckoning */
    UBX_MODE_2D	    = 0x02,	/* 2D fix */
    UBX_MODE_3D	    = 0x03,	/* 3D fix */
    UBX_MODE_GPSDR  = 0x04,	/* GPS + dead reckoning */
    UBX_MODE_TMONLY = 0x05,	/* Time-only fix */
} ubx_mode_t;

typedef enum {
    UBX_CLASS_NAV = 0x01,     /**< Navigation */
    UBX_CLASS_RXM = 0x02,     /**< Receiver Manager */
    UBX_CLASS_INF = 0x04,     /**< Informative text messages */
    UBX_CLASS_ACK = 0x05,     /**< (Not) Acknowledges for cfg messages */
    UBX_CLASS_CFG = 0x06,     /**< Configuration requests */
    UBX_CLASS_UPD = 0x09,     /**< Firmware updates */
    UBX_CLASS_MON = 0x0a,     /**< System monitoring */
    UBX_CLASS_AID = 0x0b,     /**< AGPS */
    UBX_CLASS_TIM = 0x0d,     /**< Time */
} ubx_classes_t;

typedef enum {
    UBX_NAV_PVT	    = UBX_MSGID(UBX_CLASS_NAV, 0x07)
} ubx_message_t;

typedef enum {
	Portable = 0,
	Stationary = 2,
	Pedestrian,
	Automotiv,
	Sea,
	Airbone1G,
	Airbone2G,
	Airbone4G,
	Wirst,
	Bike,
	LawnMower,
	KickScooter,
} gnss_mode_t;

struct neom8_config {
	const struct device *i2c_dev;
	uint16_t i2c_addr;
};

struct gpsTime {
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
};

struct gpsDate {
	uint8_t day;
	uint8_t month;
	uint16_t year;
};

struct neom8_data {
	const struct device *neom8;
	struct k_sem lock;

	struct gpsTime time;
	struct gpsDate date;

	float longitude_deg;
	float latitude_deg;

	float altitude;

	char ind_longitude;
	char ind_latitude;

	uint8_t satellites;

	uint8_t fixType;
};

typedef int (*neom8_api_fetch_data)(const struct device *dev);
typedef int (*neom8_api_send_ubx)(const struct device *dev, uint8_t class_, uint8_t id,
				  uint8_t payload[], uint16_t length);
typedef int (*neom8_api_cfg_dm)(const struct device *dev, gnss_mode_t mode);

typedef uint8_t (*neom8_api_get_fix)(const struct device *dev);

typedef struct gpsTime (*neom8_api_get_time)(const struct device *dev);
typedef struct gpsDate (*neom8_api_get_date)(const struct device *dev);

typedef float (*neom8_api_get_latitude)(const struct device *dev);
typedef float (*neom8_api_get_longitude)(const struct device *dev);
typedef float (*neom8_api_get_altitude)(const struct device *dev);
typedef uint8_t (*neom8_api_get_satellites)(const struct device *dev);

__subsystem struct neom8_api {
	neom8_api_fetch_data fetch_data;
	neom8_api_send_ubx send_ubx;
    neom8_api_cfg_dm set_dm;

	neom8_api_get_fix get_fix;
	neom8_api_get_time get_time;
	neom8_api_get_date get_date;
	neom8_api_get_latitude get_latitude;
	neom8_api_get_longitude get_longitude;
	neom8_api_get_altitude get_altitude;
	neom8_api_get_satellites get_satellites;
};

#endif /*ZEPHYR_INCLUDE_DRIVERS_GNSS_UBLOX_NEO_M8_H_*/