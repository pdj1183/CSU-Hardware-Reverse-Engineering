#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <nrfx_saadc.h>
#include <nrfx_gpiote.h>

#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1
#define BUFFER_SIZE 2UL
#define SAMPLING_ITERATIONS 4UL
#define ADC_RESOLUTION NRF_SAADC_RESOLUTION_14BIT
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 20)
#define SLEEP_TIME_MS 700
#define ADC_OVERSAMPLING SAADC_OVERSAMPLE_OVERSAMPLE_Over256x
#define VDD_GPIO 1800
#define MAX_FLOAT_STR_SIZE 15

LOG_MODULE_REGISTER(adc_terminal, CONFIG_ADC_TERMINAL_LOG_LEVEL);

static nrf_saadc_value_t m_samples_buffer[BUFFER_SIZE];

float custom_raw_to_milli(int raw) {
    //Hard coded values, change with Gain/Resolution Stuff
    float denominator = 0.00055556; //.25/VDD_4
    int exponent = 16384; //2^RESOLUTION

    // Calculate [V(P) - V(N)]
    // char denominator_str[MAX_FLOAT_STR_SIZE];
    // snprintf(denominator_str, MAX_FLOAT_STR_SIZE, "%f", denominator);
    // LOG_INF("RAW TO MILLI %d / ((%s) * %d)\n", raw, denominator_str, exponent);
    float voltageDifference = (raw / (denominator * exponent));

    return voltageDifference;
}


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


int main(void)
{
	struct gpio_dt_spec p17 = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
	struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

	if (!device_is_ready(p17.port)) return -1;
	if (!device_is_ready(led1.port)) return -1;
	
	nrfx_err_t status;
    (void)status;


    LOG_INF("Starting the nrf9160 ADC-Terminal with the reworked adc configuration.\n");

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

	int err;
	err = gpio_pin_configure_dt(&p17, GPIO_OUTPUT);
	err = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);


	
	if (err) return err;

    uint16_t sampling_index = 0;
	while (1) {
        	err = gpio_pin_set_dt(&p17, 1);
	        err = gpio_pin_set_dt(&led1, 1);

            status = nrfx_saadc_offset_calibrate(NULL);
            NRFX_ASSERT(status == NRFX_SUCCESS);
            LOG_INF("Calibration in the blocking manner finished successfully.");

            /*
             * If function nrfx_saadc_mode_trigger() returns NRFX_ERROR_BUSY it means that the conversion in the advanced
             * blocking mode is still being performed. Then the function must be called again to continue the conversion.
             */
            while ((status = nrfx_saadc_mode_trigger()) == NRFX_ERROR_BUSY)
            {}
            NRFX_ASSERT(status == NRFX_SUCCESS);
            LOG_INF("Sample %d", sampling_index + 1);
            float volt = 0;

            for (uint32_t buffer_index = 0; buffer_index < BUFFER_SIZE; buffer_index++)
            {   
                volt = m_samples_buffer[buffer_index];
                volt = custom_raw_to_milli(volt);
                char float_str[MAX_FLOAT_STR_SIZE];
                snprintf(float_str, MAX_FLOAT_STR_SIZE, "%f", volt);
                LOG_INF("[Sample %u]", buffer_index);
                LOG_INF("Raw value == %d mV value == %s", m_samples_buffer[buffer_index], float_str);
               
            }

            status = nrfx_saadc_buffer_set(m_samples_buffer, BUFFER_SIZE);
            NRFX_ASSERT(status == NRFX_SUCCESS);

            gpio_pin_set_dt(&p17, 0);
		    gpio_pin_set_dt(&led1, 0);

            sampling_index++;

            k_msleep(SLEEP_TIME_MS);

	}

}