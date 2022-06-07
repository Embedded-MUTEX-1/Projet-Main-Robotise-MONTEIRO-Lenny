#include <driver/i2c.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "pca9685.h"

#include "sdkconfig.h"

//definition des sorties PWM de chaque moteur
#define D0M1 0
#define D0M2 1
#define D0M3 2

#define D1M1 3
#define D1M2 4
#define D1M3 5

#define D2M1 6
#define D2M2 7
#define D2M3 8

#define D3M1 9
#define D3M2 10
#define D3M3 11

#define D4M1 12
#define D4M2 13
#define D4M3 14

#define I2C_EXAMPLE_MASTER_SCL_IO   22    /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO   21    /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_FREQ_HZ  100000     /*!< I2C master clock frequency */
#define I2C_EXAMPLE_MASTER_NUM      I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */


#define I2C_ADDRESS     0x40    /*!< lave address for PCA9685 */

#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

#define LED GPIO_NUM_18

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0; //nombre d'essai de reconnexion au reseau WIFI

//fonction gérant les évènements WIFI
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI("Wifi", "Wifi connecté.");
        gpio_set_level(LED, 1);
    }
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 3) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("Wifi", "Wifi deconnecté. Tentative de reconnection");
            gpio_set_level(LED, 0);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI("Wifi","connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("Wifi", "IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

//fonction pour se connecter au wifi
void wifi_init()
{
    // Initialisation
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();

    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // enregistrement des évènements
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT,
                                                    ESP_EVENT_ANY_ID,
                                                    &event_handler,
                                                    NULL,
                                                    &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT,
                                                    IP_EVENT_STA_GOT_IP,
                                                    &event_handler,
                                                    NULL,
                                                     &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Routeur Main Robotisee",
            .password = "12345678",
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI("Wifi", "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("wifi", "connected to ap SSID:%s password:%s",
                 "Routeur Main Robotisee", "12345678");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI("Wifi", "Failed to connect to SSID:%s, password:%s",
                "Routeur Main Robotisee", "12345678");
    } else {
        ESP_LOGE("Wifi", "UNEXPECTED EVENT");
    }
}

//initialisation de la memoire flash
void NVS_Init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

}

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init(void)
{
    ESP_LOGD("i2c", "init");
    i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = GPIO_NUM_21,         // select GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = GPIO_NUM_22,         // select GPIO specific to your project
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,};  // select frequency specific to your project
    // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0));
}
void Servo_write_pos(int num_servo, int angle)
{
    uint16_t consigne = (uint16_t)(2.28 * angle + 102);
    setPWM(num_servo, 0, consigne);
}
void app_main()
{
    i2c_example_master_init();// Initialisation de l'I2C

    set_pca9685_adress(I2C_ADDRESS);//// Initialisation du PWM Expander
    resetPCA9685();//met toutes les consigne à zero
    setFrequencyPCA9685(50);  // fréquence PWM a 50Hz
    turnAllOff();//met toutes les consigne à zero

    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    printf("Finished setup, entering loop now\n");

    NVS_Init();
    wifi_init();

    while(1)
    { 

    }
}
