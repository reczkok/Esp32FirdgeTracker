#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ultrasonic.h>
#include <esp_err.h>
#include "ssd1306.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "soc/soc_caps.h"
#include "driver/ledc.h"
#include "ble_wifi_mqtt.h"

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------
        Door General Macros
---------------------------------------------------------------*/
#define DOOR_OPEN_MAX_DISTANCE_CM 10 // 15cm max
#define DOOR_BUZZER_MAX_TIMER 30 // 30 * 1000ms = 30s
#define DOOR_CLOSED_MAX_TIMER 2.5 // 2.5 * 1000ms = 2.5s
/*---------------------------------------------------------------
        Buzzer General Macros
---------------------------------------------------------------*/

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (27) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (500) // Frequency in Hertz. Set frequency at 5 kHz

/*---------------------------------------------------------------
        Display General Macros
---------------------------------------------------------------*/
#define MAX_DISTANCE_CM 500 // 5m max
#define TRIGGER_GPIO 25
#define ECHO_GPIO 26

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_6
#define TAG "adc_cali_example"
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);


/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}


/*---------------------------------------------------------------
        Read temperature from ADC
---------------------------------------------------------------*/
void read_raw_data(float* raw, adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_chan0_handle)
{
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0][0]);
    *raw = (voltage[0][0] - 500) / 10.0;
}

/*---------------------------------------------------------------
        Init buzzer
---------------------------------------------------------------*/

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
/*---------------------------------------------------------------
        Read distance and display
---------------------------------------------------------------*/

void ultrasonic_oled_test(void *pvParameters)
{
    bool isDoorOpen = false;
    int openTimer = 0;
    int closedTimer = 0;


    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    // Update duty to apply the new value
    

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);

    ultrasonic_init(&sensor);

    SSD1306_t dev;
	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
	ssd1306_init(&dev, 132, 64);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
    int counter = 0;
    while (true)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        
        char dist[11];
        float temp;
        read_raw_data(&temp, adc1_handle, adc1_cali_chan0_handle);
        char temperature[9];

        ESP_LOGI(TAG, "Distance: %0.04f cm", distance*100);
        ESP_LOGI(TAG, "Temperature: %0.2f C", temp);

        sprintf(dist, " %0.04f cm", distance*100);
        sprintf(temperature, " %0.2f C", temp);

        ssd1306_display_text(&dev, 0, "---Distance---", 15, true);
        ssd1306_display_text(&dev, 1, dist, 11, false);
        ssd1306_display_text(&dev, 2, "-----Temp-----", 15, true);
        ssd1306_display_text(&dev, 3, temperature, 9, false);
        ssd1306_display_text(&dev, 5, "-----Door-----", 15, true);
        if(isDoorOpen){
            ssd1306_display_text(&dev, 6, "    Open    ", 12, false);
        } else {
            ssd1306_display_text(&dev, 6, "    Closed    ", 14, false);
        }


       if (distance * 100 >= DOOR_OPEN_MAX_DISTANCE_CM) {
            // Door is open
            if (!isDoorOpen) {
                // Door was previously closed, now open
                send_info(temp, false);
                counter = 0;
                isDoorOpen = true;
                openTimer = 0;
            } else {
                // Door remains open
                if (openTimer >= DOOR_BUZZER_MAX_TIMER) {
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                } else {
                    openTimer++;
                }
            }
            closedTimer = 0; // Reset closed timer
        } else {
            // Door is closed
            if (isDoorOpen) {
                // Door was open, now appears closed
                if (closedTimer >= DOOR_CLOSED_MAX_TIMER) {
                    // Confirm door is closed
                    isDoorOpen = false;
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                } else {
                    closedTimer++;
                }
            }
        }
        if (counter == 10) {
            counter = 0;
            send_info(temp, !isDoorOpen);
        } else {
            counter++;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}