#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include <string.h>

//ADC

#include <hal/nrf_saadc.h>
#define ADC_DEVICE_NODE		DT_NODELABEL(adc)
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID 1
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1
#define ADC_2ND_CHANNEL_ID 2
#define ADC_2ND_CHANNEL_INPUT NRF_SAADC_INPUT_AIN2
#define SLEEP_TIME_MS 700

const struct device *adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_1ST_CHANNEL_ID,
    .differential = 1,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = ADC_1ST_CHANNEL_INPUT,
    .input_negative = ADC_2ND_CHANNEL_INPUT,
#endif
};

#define BUFFER_SIZE 1
static int16_t m_sample_buffer[BUFFER_SIZE];


static int adc_sample(int volt)
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

	/* Print the AIN0 values */

	int32_t adc_voltage = m_sample_buffer[0];
	printk("ADC raw value: %d\n", m_sample_buffer[0]);

	ret = adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN, ADC_RESOLUTION-1, &adc_voltage);
	if (ret)
	{
		printk("raw_to_mili Broke!");
		return ret;
	}
	
	volt = (int) adc_voltage;
	printk("Measured voltage: %d mV\n", volt);

	return ret;
}
/*
int get_adc_reading(float *reading) {

	struct gpio_dt_spec p17 = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
	if (!device_is_ready(p17.port)) {
		return -1;
	}
	gpio_pin_configure_dt(&p17, GPIO_OUTPUT);

	gpio_pin_set_dt(&p17, 1);

	int err;
	float volt = 0;

	err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
	if (err) {
		printk("Error in adc setup: %d\n", err);
		return err;
	}


	NRF_SAADC_NS->TASKS_CALIBRATEOFFSET = 1;
	for (int i = 0; i <= 20; i++) {
		err = adc_sample(volt);
		if (err) {
			printk("Error in adc sampling: %d\n", err);
			return err;
		}
		k_msleep(SLEEP_TIME_MS);
	}
	*reading = volt;

	gpio_pin_set_dt(&p17, 0);
	return 0;
}
*/
int main(void)
{
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

	int volt = 0;

	err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
	if (err) {
		printk("Error in adc setup: %d\n", err);
		return err;
	}

	printk("nrf91 saadc sampling AIN0 (P0.13)\n");
	printk("Example requires secure_boot to have ");
	printk("SAADC set to non-secure!\n");
	printk("If not; BusFault/UsageFault will be triggered\n");

	/* Trigger offset calibration
	 * As this generates a _DONE and _RESULT event
	 * the first result will be incorrect.
	 */
	while (1) {
		err = adc_sample(volt);
		if (err) {
			printk("Error in adc sampling: %d\n", err);
		}
		k_msleep(SLEEP_TIME_MS);
	}
		gpio_pin_set_dt(&p17, 0);
		gpio_pin_set_dt(&led1, 0);
}