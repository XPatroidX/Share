/**
 * @file adc.hpp
 * @author Lorenzo Thomas Contessa <lorenzocontessa.dev@gmail.com>
 * @date 30/05/2023
 * @copyright 2023 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#ifndef PAYLOAD_ADC_HPP
#define PAYLOAD_ADC_HPP

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
                    DT_SPEC_AND_COMMA)
};

#define PITOT_TUBE_ADC_CHANNEL 0
#define VOLTAGE_ADC_CHANNEL 1

namespace drivers::adc {
    class Adc {
        public:
            explicit Adc();

            double getVoltage();
            double getPitotSpeed();
        private:
            int err;
            int16_t buf;
            struct adc_sequence sequence = {
                .buffer = &buf,
                /* buffer size in bytes, not number of samples */
                .buffer_size = sizeof(buf),
            };
            
    };
}

#endif