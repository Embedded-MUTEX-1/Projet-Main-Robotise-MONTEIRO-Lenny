#include <driver/i2c.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "pca9685.h"

#include "sdkconfig.h"

// definition des sorties PWM de chaque moteur
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

#define I2C_EXAMPLE_MASTER_SCL_IO 22        /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO 21        /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */

#define I2C_ADDRESS 0x40 /*!< lave address for PCA9685 */

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)                         \
    do                                             \
    {                                              \
        esp_err_t rc = (x);                        \
        if (rc != ESP_OK)                          \
        {                                          \
            ESP_LOGE("err", "esp_err_t = %d", rc); \
            assert(0 && #x);                       \
        }                                          \
    } while (0);

#define LED GPIO_NUM_18

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define SSID "Routeur Main Robotisee"
#define PASSWORD "12345678"

static int s_retry_num = 0; // nombre d'essai de reconnexion au reseau WIFI

esp_mqtt_client_handle_t mqtt_client;

void CalculeConsigne(int in, int *angle_M1, int *angle_M2, int *angle_M3);
void CalculeConsigne_pouce(int in, int *angle_M1, int *angle_M2, int *angle_M3);
void Servo_write_pos(int num_servo, int angle);

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD("MQTT", "Event dispatched from event loop base=%s, event_id=%d", base, event_id);

    esp_mqtt_event_handle_t event;
    event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT", "Connecté au brokeur\n");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI("MQTT", "Déconnecté du brokeur\n");
        break;
        /* Mis en commentaire car provoque un crash
            case MQTT_EVENT_SUBSCRIBED:
                ESP_LOGI("MQTT", "Souscription au topic %s, msg_id=%d\n", event->topic, event->msg_id);
                break;*/
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI("MQTT", "Annulation souscription au topic %s, msg_id=%d\n", event->topic, event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT", "Publcation sur le topic %s, msg_id=%d\n", event->topic, event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI("MQTT", "Reception de donnees\n");
        printf("TOPIC=%s, Len=%d\r\n", event->topic, event->topic_len);
        printf("DATA=%s, Len=%d\r\n", event->data, event->data_len);

        int M1, M2, M3 = 0; // Angle de consigne de chaque moteur
        int valeur = 0;     // valeur reçu via MQTT
        char topic[10];

        strcpy(topic, event->topic);

        if (strstr(event->topic, "Doigt0") != NULL)
        {
            valeur = atoi(event->data); // conversion strint vers int

            CalculeConsigne_pouce(valeur, &M1, &M2, &M3); // Calcule consigne

            Servo_write_pos(D0M2, M2);
            Servo_write_pos(D0M3, M3);
        }
        else if (strstr(event->topic, "Doigt1") != NULL)
        {
            valeur = atoi(event->data); // conversion strint vers int

            CalculeConsigne(valeur, &M1, &M2, &M3); // Calcule consigne

            Servo_write_pos(D1M1, M1);
            Servo_write_pos(D1M2, M2);
            Servo_write_pos(D1M3, M3);
        }
        else if (strstr(event->topic, "Doigt2") != NULL)
        {
            valeur = atoi(event->data); // conversion strint vers int

            CalculeConsigne(valeur, &M1, &M2, &M3); // Calcule consigne

            Servo_write_pos(D2M1, M1);
            Servo_write_pos(D2M2, M2);
            Servo_write_pos(D2M3, M3);
        }
        else if (strstr(event->topic, "Doigt3") != NULL)
        {
            valeur = atoi(event->data); // conversion strint vers int

            CalculeConsigne(valeur, &M1, &M2, &M3); // Calcule consigne

            Servo_write_pos(D3M1, M1);
            Servo_write_pos(D3M2, M2);
            Servo_write_pos(D3M3, M3);
        }
        else if (strstr(event->topic, "Doigt4") != NULL)
        {
            valeur = atoi(event->data); // conversion strint vers int

            CalculeConsigne(valeur, &M1, &M2, &M3); // Calcule consigne

            Servo_write_pos(D4M1, M1);
            Servo_write_pos(D4M2, M2);
            Servo_write_pos(D4M3, M3);
        }
        // vTaskDelay(100 / portTICK_PERIOD_MS);
        ESP_LOGI("Valeurs", "valeur = %d M1 = %d M2 = %d M3 = %d", valeur, M1, M2, M3);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI("MQTT", "Erreur\n");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGI("MQTT", "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI("MQTT", "Other event id:%d", event->event_id);
        break;
    }
}

// fonction initialisation du client MQTT
void MQTT_Init()
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://192.168.1.100", // adresse du brokeur
        .event_handle = NULL,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg); // config du client
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL); // enregistrement fonction event
    esp_mqtt_client_start(mqtt_client);                                                      // connection du client
}

// fonction gérant les évènements WIFI
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI("Wifi", "Wifi connecté.");
        gpio_set_level(LED, 1);
    }
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < 3)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("Wifi", "Wifi deconnecté. Tentative de reconnection");
            gpio_set_level(LED, 0);
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI("Wifi", "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("Wifi", "IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// fonction pour se connecter au wifi
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
            .ssid = SSID,
            .password = PASSWORD,
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
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI("wifi", "connected to ap SSID:%s password:%s",
                 SSID, PASSWORD);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI("Wifi", "Failed to connect to SSID:%s, password:%s",
                 SSID, PASSWORD);
    }
    else
    {
        ESP_LOGE("Wifi", "UNEXPECTED EVENT");
    }
}

// initialisation de la memoire flash
void NVS_Init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
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
        .sda_io_num = GPIO_NUM_21, // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = GPIO_NUM_22, // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    }; // select frequency specific to your project
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

    i2c_example_master_init(); // Initialisation de l'I2C

    set_pca9685_adress(I2C_ADDRESS); //// Initialisation du PWM Expander
    resetPCA9685();                  // met toutes les consigne à zero
    setFrequencyPCA9685(50);         // fréquence PWM a 50Hz
    turnAllOff();                    // met toutes les consigne à zero

    gpio_set_direction(LED, GPIO_MODE_OUTPUT); // led

    NVS_Init();
    wifi_init();

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Attent que le esp32 se connecte au routeur wifi

    MQTT_Init();

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Attent que le client se connecte au brokeur

    /* Souscription aux topics "Doigt0"(Pouce) "Doigt1", "Doigt2", "Doigt3*, "Doigt4"

      Fonctionnement:
      La carte équipée de capteurs transmettra pour chaque doigt une valeur entre O et 100,
      0 pour un doigt complètement plié, 100 pour un doigt complètement ouvert, en fonction
      des valeurs reçues via MQTT les doigs robotisés doivent plus ou moins s'ouvrir.

      Le traitement des données reçues se fait dans la fonction "mqtt_event_handler".

    */
    esp_mqtt_client_subscribe(mqtt_client, "Doigt0", 0);
    esp_mqtt_client_subscribe(mqtt_client, "Doigt1", 0);
    esp_mqtt_client_subscribe(mqtt_client, "Doigt2", 0);
    esp_mqtt_client_subscribe(mqtt_client, "Doigt3", 0);
    esp_mqtt_client_subscribe(mqtt_client, "Doigt4", 0);

    while (1)
    {
        Servo_write_pos(D0M1, 45); // pas beson du calcul pour le moteur1 du pouce
    }
}
/* Cette fonction permet de calculer les consignes angles de chaque moteur d'un doigts à envoyer
   à la fonction "Servo_write_pos" en fonction la valeur (0 et 100) reçu via le protocole MQTT */
void CalculeConsigne(int in, int *angle_M1, int *angle_M2, int *angle_M3)
{
    *angle_M1 = (int)(1.25 * in + 45);
    *angle_M2 = (int)(0.63 * in + 107);
    *angle_M3 = (int)(1.05 * in + 60);
}
/* Même fonction mais pour le pouce */
void CalculeConsigne_pouce(int in, int *angle_M1, int *angle_M2, int *angle_M3)
{
    *angle_M1 = (int)(1.2 * in + 45);
    *angle_M2 = (int)(1.21 * in + 39);
    *angle_M3 = (int)(1.21 * in + 56);
}
