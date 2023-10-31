#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <stdio.h>
#include <string.h>
#include <nrfx_saadc.h>
#include <nrfx_gpiote.h>
#include <nrfx_example.h>
#include <saadc_examples_common.h>
#include <nrfx_log.h>


#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1
#define BUFFER_SIZE 2UL
#define SAMPLING_ITERATIONS 4UL
#define ADC_RESOLUTION NRF_SAADC_RESOLUTION_14BIT
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 20)
#define SLEEP_TIME_MS 700
#define ADC_OVERSAMPLING SAADC_OVERSAMPLE_OVERSAMPLE_Over256x
#define VDD_GPIO 1.8

static nrf_saadc_value_t m_samples_buffer[BUFFER_SIZE];

static const nrfx_saadc_channel_t nrf_1st_channel = {
	.channel_config = {
		.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
		.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
		.gain = ADC_GAIN,
		.reference = ADC_REFERENCE,
		.acq_time = ADC_ACQUISITION_TIME,
		.mode = NRF_SAADC_MODE_SINGLE_ENDED,
		.burst = NRF_SAADC_BURST_ENABLED,
	},
	.pin_p = ADC_1ST_CHANNEL_INPUT,
	.pin_n = NRF_SAADC_INPUT_DISABLED,
	.channel_index = 0
};


static int adc_sample(nrfx_err_t* status, int *volt, int *raw)
{
	int ret = 0;
	int adc_voltage = 0;

	while ((*status = nrfx_saadc_mode_trigger()) == NRFX_ERROR_BUSY)
	{}
	NRFX_ASSERT(*status == NRFX_SUCCESS);
	adc_voltage = m_samples_buffer[1];
	ret = adc_raw_to_millivolts(VDD_GPIO/4, ADC_GAIN, ADC_GAIN, &adc_voltage);

	*raw = m_samples_buffer[1];
	*volt = adc_voltage;
	return ret;
}

int get_adc_reading(int *milivolt, int *raw) {

	struct gpio_dt_spec p17 = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
	struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

	if (!device_is_ready(p17.port)) return -1;
	if (!device_is_ready(led1.port)) return -1;

	nrfx_err_t status;
    (void)status;

	status = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    NRFX_ASSERT(status == NRFX_SUCCESS);


    status = nrfx_saadc_channel_config(&nrf_1st_channel);
    NRFX_ASSERT(status == NRFX_SUCCESS);

	nrfx_saadc_adv_config_t nrf_adv_1st_cfg = {
		.oversampling = ADC_OVERSAMPLING,
		.burst = NRF_SAADC_BURST_ENABLED,
	};

	uint32_t channel_mask = nrfx_saadc_channels_configured_get();
    status = nrfx_saadc_advanced_mode_set(channel_mask,
                                          ADC_RESOLUTION,
                                          &nrf_adv_1st_cfg,
                                          NULL);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    status = nrfx_saadc_buffer_set(m_samples_buffer, BUFFER_SIZE);
    NRFX_ASSERT(status == NRFX_SUCCESS);

	status = nrfx_saadc_offset_calibrate(NULL);
	NRFX_ASSERT(status == NRFX_SUCCESS);
	printk("Calibration in the blocking manner finished successfully.\n");
	
	int err;
	gpio_pin_configure_dt(&p17, GPIO_OUTPUT);
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	err = gpio_pin_set_dt(&p17, 1);
	err = gpio_pin_set_dt(&led1, 1);

	if (err) return err;
	
	for (int i = 0; i <= 2; i++) {
		err = adc_sample(&status, milivolt, raw);
		if (err) return err;
		k_msleep(SLEEP_TIME_MS);
	}

	gpio_pin_set_dt(&p17, 0);
	gpio_pin_set_dt(&led1, 0);

	return 0;
}