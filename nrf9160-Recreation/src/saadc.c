#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <nrfx_saadc.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/dt-bindings/pinctrl/nrf-pinctrl.h>


//ADC

#include <hal/nrf_saadc.h>
#define ADC_DEVICE_NODE		DT_NODELABEL(adc)
#define ADC_RESOLUTION 14
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 20)
#define ADC_1ST_CHANNEL_ID 1
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1
#define SLEEP_TIME_MS 700
#define ADC_OVERSAMPLING SAADC_OVERSAMPLE_OVERSAMPLE_Over256x

const struct device *adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);


static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_1ST_CHANNEL_ID,
    .differential = 0,
	
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = ADC_1ST_CHANNEL_INPUT,
#endif
};

nrfx_saadc_adv_config_t nrf_adv_1st_cfg = {
	.oversampling = ADC_OVERSAMPLING,
	.burst = NRF_SAADC_BURST_ENABLED,
};

#define BUFFER_SIZE 1
static int16_t m_sample_buffer[BUFFER_SIZE];


static int adc_sample(int *volt, int *raw)
{
	int ret;

	const struct adc_sequence sequence = {
		.channels = BIT(ADC_1ST_CHANNEL_ID),
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (!adc_dev) {
		return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	int32_t adc_voltage = m_sample_buffer[0];

	ret = adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN, ADC_RESOLUTION, &adc_voltage);
	if (ret)
	{
		printk("raw_to_mili Broke!");
		return ret;
	}
	*raw = m_sample_buffer[0];
	*volt = adc_voltage;
	return ret;
}

int get_adc_reading(int *milivolt, int *raw) {

	struct gpio_dt_spec p17 = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
	struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

	if (!device_is_ready(p17.port)) {
		return -1;
	}

	if (!device_is_ready(led1.port)) {
		return -1;
	}
	int err;
	gpio_pin_configure_dt(&p17, GPIO_OUTPUT);
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	err = gpio_pin_set_dt(&p17, 1);
	err = gpio_pin_set_dt(&led1, 1);

	if (err)
	{
		return err;
	}

	err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
	if (err) {
		return err;
	}

	
	
	for (int i = 0; i <= 20; i++) {
		err = adc_sample(milivolt, raw);
		if (err) {
			return err;
		}
		k_msleep(SLEEP_TIME_MS);
	}

	gpio_pin_set_dt(&p17, 0);
	gpio_pin_set_dt(&led1, 0);

	return 0;
}