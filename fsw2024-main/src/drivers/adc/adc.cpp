#include <zephyr/logging/log.h>

#include <cstdio>
#include "adc.hpp"
#include <math.h>

LOG_MODULE_REGISTER(adc, LOG_LEVEL_DBG);

namespace drivers::adc {
	Adc::Adc() {
		int err;
        uint32_t count = 0;
		int16_t buf;
		struct adc_sequence sequence = {
			.buffer = &buf,
			/* buffer size in bytes, not number of samples */
			.buffer_size = sizeof(buf),
		};

		/* Configure channels individually prior to sampling. */
		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
			if (!device_is_ready(adc_channels[i].dev)) {
				printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
				return;
			}

			err = adc_channel_setup_dt(&adc_channels[i]);
			if (err < 0) {
				printk("Could not setup channel #%d (%d)\n", i, err);
				return;
			}
		}
	}

	double Adc::getVoltage() {
		size_t i = VOLTAGE_ADC_CHANNEL;
        int32_t val_mv;
        (void)adc_sequence_init_dt(&adc_channels[i], &sequence);
        err = adc_read_dt(&adc_channels[i], &sequence);

        if (err < 0) {
            printk("Could not read (%d)\n", err);
        }
        /*
        * If using differential mode, the 16 bit value
        * in the ADC sample buffer should be a signed 2's
        * complement value.
        */
        if (adc_channels[i].channel_cfg.differential) {
            val_mv = (int32_t)((int16_t)buf);
        } else {
            val_mv = (int32_t)buf;
        }
        //printk("%"PRId32, val_mv);
        err = adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
        /* conversion to mV may not be supported, skip if not */
        if (err < 0) {
            printk(" (value in mV not available)\n");
        } else {
            // printk(" = %"PRId32" mV\n", val_mv);
        }
        return round((val_mv/1000.0) * 10)/10;
	}

    double Adc::getPitotSpeed() {
		size_t i = PITOT_TUBE_ADC_CHANNEL;
        int32_t val_mv;
        (void)adc_sequence_init_dt(&adc_channels[i], &sequence);

        err = adc_read_dt(&adc_channels[i], &sequence);
        if (err < 0) {
            printk("Could not read (%d)\n", err);
        }
        /*
        * If using differential mode, the 16 bit value
        * in the ADC sample buffer should be a signed 2's
        * complement value.
        */
        if (adc_channels[i].channel_cfg.differential) {
            val_mv = (int32_t)((int16_t)buf);
        } else {
            val_mv = (int32_t)buf;
        }

        err = adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
        /* conversion to mV may not be supported, skip if not */
        if (err < 0) {
            printk(" (value in mV not available)\n");
        } else {
            // printk(" = %"PRId32" mV\n", val_mv);
        }
        double val = (double) round((val_mv/1000.0) * 10)/10;
        double RO = 1.225;
        double radi = (2 * ((100/47) * val - 1) * 1000) / RO;
        double velocity = sqrt(radi);
        return velocity;
	}
}