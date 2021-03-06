#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>

#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

#include "core2forAWS.h"

#include "wifi.h"

void disconnect_callback_handler(AWS_IoT_Client *pClient, void *data);
static void publisher(AWS_IoT_Client *client, char *base_topic, uint16_t base_topic_len);
static void mbox_event_cb(lv_obj_t *obj, lv_event_t evt);
void mqtt_send(void *param);
void getGsrInput(void *parameter);
void barTimerHandler(void *param);
void start_btn_event_handler(lv_obj_t *obj, lv_event_t event);
static void sensor_btnm_event_handler(lv_obj_t *obj, lv_event_t event);
static void home_btn_event_handler(lv_obj_t *obj, lv_event_t event);
static void send_btn_event_handler(lv_obj_t *obj, lv_event_t event);

TaskHandle_t barTimerHandler_handle;

lv_obj_t *sensor_btnm;
lv_obj_t *gsr_chart;
lv_obj_t *gsr_text_area;
lv_obj_t *timer_bar;
lv_obj_t *start_btn;
lv_chart_series_t *gsr_ser1;
lv_obj_t *home_btn;
lv_obj_t *send_btn;
lv_obj_t *mbox1;
lv_obj_t *mic_chart;
lv_chart_series_t *mic_ser1;
lv_obj_t *mic_text_area;
lv_obj_t *gyro_chart;
lv_chart_series_t *gyro_ser1;
lv_chart_series_t *gyro_ser2;
lv_chart_series_t *gyro_ser3;
lv_obj_t *gyro_text_area;

bool isBarTimerComplete = true;

int gsr = 0;
int gsr_avg = 0;
int gsr_sum = 0;

int mic = 0;
int mic_avg = 0;
int mic_sum = 0;

float roll = 0;
float yaw = 0;
float pitch = 0;

int counter = 0;

int list[110];
float gyroList[330];
char *listType;
int listCounter = 0;

static const char *TAG = "MAIN";

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");

char HostAddress[255] = AWS_IOT_MQTT_HOST;

uint32_t port = AWS_IOT_MQTT_PORT;

char *client_id;

void app_main()
{
    Core2ForAWS_Init();
    Core2ForAWS_Display_SetBrightness(80);

    Core2ForAWS_Display_SetBrightness(80);

    Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_LEFT, 0xFF0000);
    Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_RIGHT, 0xFF0000);

    Core2ForAWS_Sk6812_Show();

    home_btn = lv_btn_create(lv_scr_act(), NULL);
    lv_obj_align(home_btn, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);
    lv_obj_set_style_local_value_str(home_btn, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, "Park It!");
    lv_obj_set_event_cb(home_btn, home_btn_event_handler);

    static const char *sensor_btnm_map[] = {"GSR", "Mic", "Gyro", ""};

    sensor_btnm = lv_btnmatrix_create(lv_scr_act(), NULL);
    lv_btnmatrix_set_map(sensor_btnm, sensor_btnm_map);
    lv_obj_align(sensor_btnm, NULL, LV_ALIGN_CENTER, 5, 10);
    lv_obj_set_event_cb(sensor_btnm, sensor_btnm_event_handler);

    initialise_wifi();

    xTaskCreatePinnedToCore(
        barTimerHandler,
        "bar timer handler",
        4096 * 2,
        NULL,
        1,
        &barTimerHandler_handle,
        1);
}

void disconnect_callback_handler(AWS_IoT_Client *pClient, void *data)
{
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if (pClient == NULL)
    {
        return;
    }

    if (aws_iot_is_autoreconnect_enabled(pClient))
    {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    }
    else
    {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if (NETWORK_RECONNECTED == rc)
        {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        }
        else
        {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

int id = 1;
static void publisher(AWS_IoT_Client *client, char *base_topic, uint16_t base_topic_len)
{
    char cPayload[200];
    IoT_Publish_Message_Params paramsQOS1;

    paramsQOS1.qos = QOS1;
    paramsQOS1.payload = (void *)cPayload;
    paramsQOS1.isRetained = 0;
    if (strcmp(listType, "Gyro") != 0)
    {
        for (int i = 0; i < 101; i++)
        {
            char sid[5];
            itoa(id, sid, 10);
            sprintf(cPayload, "{\"id\":\"%s\",\"measurementValue\":%d,\"measurementType\":\"%s\",\"clientID\":\"%s\"}", sid, list[i], listType, client_id);
            paramsQOS1.payloadLen = strlen(cPayload);
            IoT_Error_t rc = aws_iot_mqtt_publish(client, base_topic, base_topic_len, &paramsQOS1);
            if (rc == MQTT_REQUEST_TIMEOUT_ERROR)
            {
                ESP_LOGW(TAG, "QOS1 publish not received.");
                rc = SUCCESS;
            }
            id++;
        }
    }
    else
    {
        for (int i = 0; i < 303; i++)
        {
            char sid[5];
            itoa(id, sid, 10);
            sprintf(cPayload, "{\"id\":\"%s\",\"measurementValue\":%d,\"measurementType\":\"%s\",\"clientID\":\"%s\"}", sid, (int)gyroList[i], "Roll", client_id);
            paramsQOS1.payloadLen = strlen(cPayload);
            IoT_Error_t rc = aws_iot_mqtt_publish(client, base_topic, base_topic_len, &paramsQOS1);
            if (rc == MQTT_REQUEST_TIMEOUT_ERROR)
            {
                ESP_LOGW(TAG, "QOS1 publish not received.");
                rc = SUCCESS;
            }
            id++;
            i++;
            itoa(id, sid, 10);
            sprintf(cPayload, "{\"id\":\"%s\",\"measurementValue\":%d,\"measurementType\":\"%s\",\"clientID\":\"%s\"}", sid, (int)gyroList[i], "Yaw", client_id);
            paramsQOS1.payloadLen = strlen(cPayload);
            rc = aws_iot_mqtt_publish(client, base_topic, base_topic_len, &paramsQOS1);
            if (rc == MQTT_REQUEST_TIMEOUT_ERROR)
            {
                ESP_LOGW(TAG, "QOS1 publish not received.");
                rc = SUCCESS;
            }
            id++;
            i++;
            itoa(id, sid, 10);
            sprintf(cPayload, "{\"id\":\"%s\",\"measurementValue\":%d,\"measurementType\":\"%s\",\"clientID\":\"%s\"}", sid, (int)gyroList[i], "Pitch", client_id);
            paramsQOS1.payloadLen = strlen(cPayload);
            rc = aws_iot_mqtt_publish(client, base_topic, base_topic_len, &paramsQOS1);
            if (rc == MQTT_REQUEST_TIMEOUT_ERROR)
            {
                ESP_LOGW(TAG, "QOS1 publish not received.");
                rc = SUCCESS;
            }
            id++;
        }
    }

    static const char *btns[] = {"Continue", ""};

    mbox1 = lv_msgbox_create(lv_scr_act(), NULL);
    lv_msgbox_set_text(mbox1, "Open the Park It! website to view the updated data");
    lv_msgbox_add_btns(mbox1, btns);
    lv_obj_set_width(mbox1, 200);
    lv_obj_set_event_cb(mbox1, mbox_event_cb);
    lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 0, 0);
    Core2ForAWS_Motor_SetStrength(50);
    lv_obj_set_hidden(mbox1, false);
}

static void mbox_event_cb(lv_obj_t *obj, lv_event_t evt)
{
    if (evt == LV_EVENT_VALUE_CHANGED)
    {
        lv_obj_set_hidden(gsr_chart, true);
        lv_obj_set_hidden(gsr_text_area, true);
        lv_obj_set_hidden(mic_chart, true);
        lv_obj_set_hidden(mic_text_area, true);
        lv_obj_set_hidden(gyro_chart, true);
        lv_obj_set_hidden(gyro_text_area, true);
        lv_obj_set_hidden(timer_bar, true);
        lv_obj_set_hidden(start_btn, true);
        lv_obj_set_hidden(send_btn, true);
        lv_obj_set_hidden(mbox1, true);
        listCounter = 0;
        isBarTimerComplete = true;
        lv_obj_set_hidden(sensor_btnm, false);
        Core2ForAWS_Motor_SetStrength(0);
    }
}

void mqtt_send(void *param)
{
    IoT_Error_t rc = FAILURE;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false;
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;
    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = "#";
    mqttInitParams.pDevicePrivateKeyLocation = "#0";

#define CLIENT_ID_LEN (ATCA_SERIAL_NUM_SIZE * 2)
#define SUBSCRIBE_TOPIC_LEN (CLIENT_ID_LEN + 3)
#define BASE_PUBLISH_TOPIC_LEN (CLIENT_ID_LEN + 2)

    client_id = malloc(CLIENT_ID_LEN + 1);
    ATCA_STATUS ret = Atecc608_GetSerialString(client_id);
    if (ret != ATCA_SUCCESS)
    {
        printf("Failed to get device serial from secure element. Error: %i", ret);
        abort();
    }

    char subscribe_topic[SUBSCRIBE_TOPIC_LEN];
    char base_publish_topic[BASE_PUBLISH_TOPIC_LEN];
    snprintf(subscribe_topic, SUBSCRIBE_TOPIC_LEN, "%s/#", client_id);
    snprintf(base_publish_topic, BASE_PUBLISH_TOPIC_LEN, "%s/", client_id);

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnect_callback_handler;
    mqttInitParams.disconnectHandlerData = NULL;

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if (SUCCESS != rc)
    {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;

    connectParams.pClientID = client_id;
    connectParams.clientIDLen = CLIENT_ID_LEN;
    connectParams.isWillMsgPresent = false;
    ESP_LOGI(TAG, "Connecting to AWS IoT Core at %s:%d", mqttInitParams.pHostURL, mqttInitParams.port);
    do
    {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if (SUCCESS != rc)
        {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while (SUCCESS != rc);
    ESP_LOGI(TAG, "Successfully connected to AWS IoT Core!");
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if (SUCCESS != rc)
    {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    ESP_LOGI(TAG, "\n****************************************\n*  AWS client Id - %s  *\n****************************************\n\n",
             client_id);

    if ((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc))
    {

        rc = aws_iot_mqtt_yield(&client, 100);

        ESP_LOGD(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));

        publisher(&client, base_publish_topic, BASE_PUBLISH_TOPIC_LEN);
    }
}

void getGsrInput(void *parameter)
{
    Core2ForAWS_Port_PinMode(PORT_B_ADC_PIN, ADC);
    gsr = Core2ForAWS_Port_B_ADC_ReadRaw();
    gsr_sum += gsr;
    counter++;
    gsr_avg = gsr_sum / counter;
    list[listCounter] = gsr;
    listCounter++;
}

void getMicInput(void *parameter)
{
    /* If the speaker was initialized, be sure to call Speaker_Deinit() and 
            disable first. */
    Microphone_Init();
    static int8_t i2s_readraw_buf[1024];
    size_t bytes_read;
    i2s_read(MIC_I2S_NUMBER, (char *)i2s_readraw_buf, 1024, &bytes_read, pdMS_TO_TICKS(100));
    Microphone_Deinit();
    int noise_sum = 0;
    for (uint16_t i = 0; i < 1024; i++)
    {
        noise_sum += i2s_readraw_buf[i];
    }
    mic = noise_sum / 1024;
    mic_sum += mic;
    counter++;
    mic_avg = mic_sum / counter;
    list[listCounter] = mic;
    listCounter++;
}

void getGyroInput(void *parameter)
{
    MPU6886_GetGyroData(&roll, &yaw, &pitch);
    gyroList[listCounter] = roll;
    listCounter++;
    gyroList[listCounter] = yaw;
    listCounter++;
    gyroList[listCounter] = pitch;
    listCounter++;
    counter++;
}

void barTimerHandler(void *param)
{
    while (true)
    {
        if (isBarTimerComplete == false)
        {
            if (strcmp(listType, "GSR") == 0)
            {
                getGsrInput(NULL);
                if (counter <= 100)
                {
                    lv_bar_set_value(timer_bar, counter, LV_ANIM_ON);
                }
                else
                {
                    isBarTimerComplete = true;
                    lv_obj_set_hidden(send_btn, false);
                }
                char gsr_text[100];
                sprintf(gsr_text, "GSR: %d\n--------------------\nMoving Average: %d", gsr, gsr_avg);
                lv_chart_set_next(gsr_chart, gsr_ser1, gsr);
                lv_textarea_set_text(gsr_text_area, gsr_text);
                vTaskDelay(10);
            }
            else if (strcmp(listType, "Mic") == 0)
            {
                getMicInput(NULL);
                if (counter <= 100)
                {
                    lv_bar_set_value(timer_bar, counter, LV_ANIM_ON);
                }
                else
                {
                    isBarTimerComplete = true;
                    lv_obj_set_hidden(send_btn, false);
                }
                char mic_text[100];
                sprintf(mic_text, "Mic: %d\n--------------------\nMoving Average: %d", mic, mic_avg);
                lv_chart_set_next(mic_chart, mic_ser1, mic);
                lv_textarea_set_text(mic_text_area, mic_text);
                vTaskDelay(10);
            }
            else
            {
                getGyroInput(NULL);
                if (counter <= 100)
                {
                    lv_bar_set_value(timer_bar, counter, LV_ANIM_ON);
                }
                else
                {
                    isBarTimerComplete = true;
                    lv_obj_set_hidden(send_btn, false);
                }
                char gyro_text[100];
                sprintf(gyro_text, "Roll: %.2f\nYaw: %.2f\nPitch: %.2f\n", roll, yaw, pitch);
                lv_chart_set_next(gyro_chart, gyro_ser1, roll);
                lv_chart_set_next(gyro_chart, gyro_ser2, yaw);
                lv_chart_set_next(gyro_chart, gyro_ser3, pitch);
                lv_textarea_set_text(gyro_text_area, gyro_text);
                vTaskDelay(10);
            }
        }
        else
        {
            gsr = 0;
            gsr_avg = 0;
            gsr_sum = 0;
            mic = 0;
            mic_avg = 0;
            mic_sum = 0;
            roll = (float)0.0;
            yaw = (float)0.0;
            pitch = (float)0.0;
            counter = 0;
        }
    }
}

void start_btn_event_handler(lv_obj_t *obj, lv_event_t event)
{
    gsr = 0;
    gsr_avg = 0;
    gsr_sum = 0;
    mic = 0;
    mic_avg = 0;
    mic_sum = 0;
    roll = (float)0.0;
    yaw = (float)0.0;
    pitch = (float)0.0;
    counter = 0;
    listCounter = 0;
    isBarTimerComplete = false;
}

static void sensor_btnm_event_handler(lv_obj_t *obj, lv_event_t event)
{
    if (event == LV_EVENT_VALUE_CHANGED)
    {
        lv_obj_set_hidden(sensor_btnm, true);
        const char *txt = lv_btnmatrix_get_active_btn_text(obj);
        timer_bar = lv_bar_create(lv_scr_act(), NULL);
        lv_obj_set_hidden(timer_bar, true);
        lv_obj_set_size(timer_bar, 147, 15);
        lv_obj_align(timer_bar, NULL, LV_ALIGN_IN_RIGHT_MID, -10, 40);
        lv_bar_set_anim_time(timer_bar, 2000);
        static lv_style_t style_halo;
        lv_style_init(&style_halo);
        lv_style_set_transition_time(&style_halo, LV_STATE_PRESSED, 400);
        lv_style_set_transition_time(&style_halo, LV_STATE_DEFAULT, 0);
        lv_style_set_transition_delay(&style_halo, LV_STATE_DEFAULT, 200);
        lv_style_set_outline_width(&style_halo, LV_STATE_DEFAULT, 0);
        lv_style_set_outline_width(&style_halo, LV_STATE_PRESSED, 20);
        lv_style_set_outline_opa(&style_halo, LV_STATE_DEFAULT, LV_OPA_COVER);
        lv_style_set_outline_opa(&style_halo, LV_STATE_FOCUSED, LV_OPA_COVER);
        lv_style_set_outline_opa(&style_halo, LV_STATE_PRESSED, LV_OPA_TRANSP);
        lv_style_set_transition_prop_1(&style_halo, LV_STATE_DEFAULT, LV_STYLE_OUTLINE_OPA);
        lv_style_set_transition_prop_2(&style_halo, LV_STATE_DEFAULT, LV_STYLE_OUTLINE_WIDTH);
        start_btn = lv_btn_create(lv_scr_act(), NULL);
        lv_obj_set_hidden(start_btn, true);
        lv_obj_align(start_btn, NULL, LV_ALIGN_IN_RIGHT_MID, -25, 10);
        lv_obj_set_size(start_btn, 140, 35);
        lv_obj_add_style(start_btn, LV_BTN_PART_MAIN, &style_halo);
        lv_obj_set_style_local_value_str(start_btn, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, "Start");
        lv_obj_set_event_cb(start_btn, start_btn_event_handler);
        send_btn = lv_btn_create(lv_scr_act(), NULL);
        lv_obj_align(send_btn, NULL, LV_ALIGN_IN_RIGHT_MID, -25, -30);
        lv_obj_add_style(send_btn, LV_BTN_PART_MAIN, &style_halo);
        lv_obj_set_size(send_btn, 140, 35);
        lv_obj_set_style_local_value_str(send_btn, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, "Send");
        lv_obj_set_event_cb(send_btn, send_btn_event_handler);
        lv_obj_set_hidden(send_btn, true);
        gsr_chart = lv_chart_create(lv_scr_act(), NULL);
        lv_obj_set_hidden(gsr_chart, true);
        gsr_text_area = lv_textarea_create(lv_scr_act(), NULL);
        lv_obj_set_hidden(gsr_text_area, true);
        mic_chart = lv_chart_create(lv_scr_act(), NULL);
        lv_obj_set_hidden(mic_chart, true);
        mic_text_area = lv_textarea_create(lv_scr_act(), NULL);
        lv_obj_set_hidden(mic_text_area, true);
        gyro_chart = lv_chart_create(lv_scr_act(), NULL);
        lv_obj_set_hidden(gyro_chart, true);
        gyro_text_area = lv_textarea_create(lv_scr_act(), NULL);
        lv_obj_set_hidden(gyro_text_area, true);
        if (strcmp(txt, "GSR") == 0)
        {
            lv_obj_set_hidden(mic_chart, true);
            lv_obj_set_hidden(mic_text_area, true);
            lv_obj_set_hidden(gyro_chart, true);
            lv_obj_set_hidden(gyro_text_area, true);
            lv_obj_set_hidden(start_btn, false);
            lv_obj_set_hidden(timer_bar, false);

            listType = "GSR";

            lv_obj_set_hidden(gsr_text_area, false);
            lv_obj_set_size(gsr_text_area, 147, 100);
            lv_obj_align(gsr_text_area, NULL, LV_ALIGN_IN_LEFT_MID, 10, 0);
            lv_textarea_set_cursor_hidden(gsr_text_area, true);
            char gsr_text[100];
            sprintf(gsr_text, "GSR:  \n--------------------\nMoving Average:  ");
            lv_textarea_set_text(gsr_text_area, gsr_text);

            lv_obj_set_hidden(gsr_chart, false);
            lv_obj_set_size(gsr_chart, 300, 50);
            lv_obj_align(gsr_chart, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -5);
            lv_chart_set_type(gsr_chart, LV_CHART_TYPE_LINE);
            lv_chart_set_range(gsr_chart, 0, 4095);

            gsr_ser1 = lv_chart_add_series(gsr_chart, LV_COLOR_RED);

            lv_chart_refresh(gsr_chart);
        }
        else if (strcmp(txt, "Mic") == 0)
        {
            lv_obj_set_hidden(gsr_chart, true);
            lv_obj_set_hidden(gsr_text_area, true);
            lv_obj_set_hidden(gyro_chart, true);
            lv_obj_set_hidden(gyro_text_area, true);
            lv_obj_set_hidden(start_btn, false);
            lv_obj_set_hidden(timer_bar, false);

            listType = "Mic";

            lv_obj_set_hidden(mic_text_area, false);
            lv_obj_set_size(mic_text_area, 147, 100);
            lv_obj_align(mic_text_area, NULL, LV_ALIGN_IN_LEFT_MID, 10, 0);
            lv_textarea_set_cursor_hidden(mic_text_area, true);
            char mic_text[100];
            sprintf(mic_text, "Mic:  \n--------------------\nMoving Average:  ");
            lv_textarea_set_text(mic_text_area, mic_text);

            lv_obj_set_hidden(mic_chart, false);
            lv_obj_set_size(mic_chart, 300, 50);
            lv_obj_align(mic_chart, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -5);
            lv_chart_set_type(mic_chart, LV_CHART_TYPE_LINE);
            lv_chart_set_range(mic_chart, -35, 0);

            mic_ser1 = lv_chart_add_series(mic_chart, LV_COLOR_RED);

            lv_chart_refresh(mic_chart);
        }
        else if (strcmp(txt, "Gyro") == 0)
        {
            lv_obj_set_hidden(gsr_chart, true);
            lv_obj_set_hidden(gsr_text_area, true);
            lv_obj_set_hidden(mic_chart, true);
            lv_obj_set_hidden(mic_text_area, true);
            lv_obj_set_hidden(start_btn, false);
            lv_obj_set_hidden(timer_bar, false);

            listType = "Gyro";

            lv_obj_set_hidden(gyro_text_area, false);
            lv_obj_set_size(gyro_text_area, 147, 100);
            lv_obj_align(gyro_text_area, NULL, LV_ALIGN_IN_LEFT_MID, 10, 0);
            lv_textarea_set_cursor_hidden(gyro_text_area, true);
            char gyro_text[100];
            sprintf(gyro_text, "Roll: \nYaw: \nPitch: \n");
            lv_textarea_set_text(gyro_text_area, gyro_text);

            lv_obj_set_hidden(gyro_chart, false);
            lv_obj_set_size(gyro_chart, 300, 50);
            lv_obj_align(gyro_chart, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -5);
            lv_chart_set_type(gyro_chart, LV_CHART_TYPE_LINE);
            lv_chart_set_range(gyro_chart, -200, 200);

            gyro_ser1 = lv_chart_add_series(gyro_chart, LV_COLOR_RED);
            gyro_ser2 = lv_chart_add_series(gyro_chart, LV_COLOR_GREEN);
            gyro_ser3 = lv_chart_add_series(gyro_chart, LV_COLOR_BLUE);

            lv_chart_refresh(gyro_chart);
        }
        else
        {
            printf("Type not detected\n");
        }
    }
}

static void home_btn_event_handler(lv_obj_t *obj, lv_event_t event)
{
    if (event == LV_EVENT_CLICKED)
    {
        lv_obj_set_hidden(sensor_btnm, false);
        lv_obj_set_hidden(gsr_chart, true);
        lv_obj_set_hidden(gsr_text_area, true);
        lv_obj_set_hidden(mic_chart, true);
        lv_obj_set_hidden(mic_text_area, true);
        lv_obj_set_hidden(gyro_chart, true);
        lv_obj_set_hidden(gyro_text_area, true);
        lv_obj_set_hidden(timer_bar, true);
        lv_obj_set_hidden(start_btn, true);
        lv_obj_set_hidden(send_btn, true);
        listCounter = 0;

        isBarTimerComplete = true;
    }
}

static void send_btn_event_handler(lv_obj_t *obj, lv_event_t event)
{
    if (event == LV_EVENT_CLICKED)
    {
        mqtt_send(NULL);
    }
}