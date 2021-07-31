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

void getGsrInput(void *parameter);
void getAccelData(void *parameter);
static void getMicData(void *parameter);
void barTimerHandler(void *param);
void plotInput(void *parameter);

static QueueHandle_t input_queue = NULL; // queue to store input from sensors
static int INPUT_QUEUE_LEN = 10;         // length of input queue

// struct which is sent to the queue in getGsrInput func
struct InputMessage
{
    int value;      // value retrieved from sensor
    float accel[3]; // special array if type is accel
    char *type;     // type of value (GSR, Heartrate, etc.)
} inputMessage;

/***********************************************/
/* intialize default values for data variables */
int gsr_value = 0;
float accel_value[3] = {0.0f, 0.0f, 0.0f};
int mic_value = 0;
/***********************************************/

// mutex to control whether input message can be changed
// needed to stop multiple tasks from overwriting the inputMessage at the same time
static SemaphoreHandle_t changeInputMessageMutex;

TaskHandle_t getGsrInputHandle;
TaskHandle_t getAccelDataHandle;
TaskHandle_t getMicDataHandle;
TaskHandle_t plotInputHandle;
TaskHandle_t mqtt_send_task_handle;
TaskHandle_t barTimerHandler_handle;

/* The time between each MQTT message publish in milliseconds */
#define PUBLISH_INTERVAL_MS 500

/* The time prefix used by the logger. */
static const char *TAG = "MAIN";

/* CA Root certificate */
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");

/* Default MQTT HOST URL is pulled from the aws_iot_config.h */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/* Default MQTT port is pulled from the aws_iot_config.h */
uint32_t port = AWS_IOT_MQTT_PORT;

char *client_id;

void disconnect_callback_handler(AWS_IoT_Client *pClient, void *data)
{
    ESP_LOGW(TAG, "MQTT Disconnect");
    // ui_textarea_add("Disconnected from AWS IoT Core...", NULL, 0);
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

static void publisher(AWS_IoT_Client *client, char *base_topic, uint16_t base_topic_len)
{
    char cPayload[200];
    IoT_Publish_Message_Params paramsQOS1;

    paramsQOS1.qos = QOS1;
    paramsQOS1.payload = (void *)cPayload;
    paramsQOS1.isRetained = 0;
    // Publish and check if "ack" was sent from AWS IoT Core
    sprintf(cPayload, "{ \"gsr\" : %d, \"mic\" : %d, \"accelZero\" : %f, \"accelOne\" : %f, \"accelTwo\" : %f, \"client_id\" : \"%s\" }",gsr_value,mic_value,accel_value[0],accel_value[1],accel_value[2],client_id);
    paramsQOS1.payloadLen = strlen(cPayload);
    IoT_Error_t rc = aws_iot_mqtt_publish(client, base_topic, base_topic_len, &paramsQOS1);
    if (rc == MQTT_REQUEST_TIMEOUT_ERROR)
    {
        ESP_LOGW(TAG, "QOS1 publish ack not received.");
        rc = SUCCESS;
    }
}

void mqtt_send_task(void *param)
{
    IoT_Error_t rc = FAILURE;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
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

    /* Wait for WiFI to show as connected */
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
    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time for exponential backoff for retries.
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if (SUCCESS != rc)
    {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    ESP_LOGI(TAG, "\n****************************************\n*  AWS client Id - %s  *\n****************************************\n\n",
             client_id);

    // printf("Attempting publish to: %s\n", base_publish_topic);
    while ((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc))
    {

        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 100);
        if (NETWORK_ATTEMPTING_RECONNECT == rc)
        {
            // If the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }

        ESP_LOGD(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(pdMS_TO_TICKS(PUBLISH_INTERVAL_MS));

        // publisher(&client, base_publish_topic, BASE_PUBLISH_TOPIC_LEN);
    }

    // ESP_LOGE(TAG, "An error occurred in the main loop.");
    // abort();
}

void app_main()
{
    Core2ForAWS_Init();
    Core2ForAWS_Display_SetBrightness(80);

    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    xSemaphoreGive(xGuiSemaphore);

    initialise_wifi();

    xTaskCreatePinnedToCore(&mqtt_send_task, "mqtt send task", 4096 * 2, NULL, 1, &mqtt_send_task_handle, 1);

    vTaskSuspend(mqtt_send_task_handle);

    /* 
    initialize input queue with INPUT_QUE_LEN of inputMessage 
    (assign double the memory to avoid LoadProhibited error) 
    */
    input_queue = xQueueCreate(INPUT_QUEUE_LEN, sizeof(inputMessage) * 2);

    // check that input queue was created succesfully
    if (input_queue == NULL)
    {
        printf("(app_main) There was an error in creating the input queue");
    }

    changeInputMessageMutex = xSemaphoreCreateMutex(); // initialize the mutex

    xTaskCreatePinnedToCore(
        getGsrInput,
        "get gsr input",
        4096 * 2,
        NULL,
        1,
        &getGsrInputHandle,
        0);

    vTaskSuspend(getGsrInputHandle);

    xTaskCreatePinnedToCore(
        getAccelData,
        "get accel data",
        4096 * 2,
        NULL,
        1,
        &getAccelDataHandle,
        0);

    vTaskSuspend(getAccelDataHandle);

    xTaskCreatePinnedToCore(
        getMicData,
        "get mic data",
        4096 * 2,
        NULL,
        1,
        &getMicDataHandle,
        0);
    
    vTaskSuspend(getMicDataHandle);

    xTaskCreatePinnedToCore(
        plotInput,
        "plot input",
        4096 * 2,
        NULL,
        1,
        &plotInputHandle,
        0);
    
    xTaskCreatePinnedToCore(
        barTimerHandler,
        "bar timer handler",
        4096*2,
        NULL,
        1,
        &barTimerHandler_handle,
        1
    );
}

// get the GSR input from Port B and send it into the gsr queue
void getGsrInput(void *parameter)
{
    printf("initialized getGSRInput task\n"); // signals that getGsrInput is initialized
    while (true)
    {
        Core2ForAWS_Port_PinMode(PORT_B_ADC_PIN, ADC); // Sets Port B (ADC) as the pinmode
        /**********************************************************/
        /* average 10 input values to get a more accurate reading */
        int gsr_value = 0;
        for (int i = 0; i < 10; i++)
        {
            gsr_value += Core2ForAWS_Port_B_ADC_ReadRaw();
        }
        gsr_value /= 10;
        /*---CRITICAL SECTION---*/
        if (xSemaphoreTake(changeInputMessageMutex, 10) == pdTRUE)
        { // see if task can take the changeInputMessageMutex
            /**********************************************************/
            inputMessage.value = gsr_value; // Sets inputMessage.value to the value inputted by the sensor
            inputMessage.type = "gsr";      // Sets the type of value to "gsr"
            /*******************************************************************************/
            /* send inputMessage struct to queue and output if the send succeded or failed */
            if (xQueueSend(input_queue, &inputMessage, 10) == pdTRUE)
            {
                printf("(getGsrInput) succesfully sent value of %d and type of %s to input queue\n", inputMessage.value, inputMessage.type);
            }
            else
            {
                printf("(getGsrInput) failed to send value of %d and type of %s to input queue\n", inputMessage.value, inputMessage.type);
            }
            /*******************************************************************************/
            xSemaphoreGive(changeInputMessageMutex); // release the mutex at the end of critical section
        }
        else
        {
            printf("(getGsrInput) Wasn't able to take changeInputMessageMutex");
        }
        /*---CRITICAL SECTION---*/
    }
}

// get the data from the 6-axis IMU and push it to input queue
void getAccelData(void *parameter)
{
    float accel[3]; // array of accel in format x,y,z

    printf("intialized getAccelData task\n");
    while (true)
    {
        MPU6886_GetAccelData(&accel[0], &accel[1], &accel[2]); // get accel data
        /*---CRITICAL SECTION---*/
        if (xSemaphoreTake(changeInputMessageMutex, 20) == pdTRUE)
        { // see if task can take the changeInputMessageMutex
            /**********************************************************/
            inputMessage.value = 0; // set value to 0 or false
            inputMessage.accel[0] = accel[0];
            inputMessage.accel[1] = accel[1];
            inputMessage.accel[2] = accel[2]; // Sets inputMessage.accel to the accel gotten from sensor
            inputMessage.type = "accel";      // Sets the type of value to "accel"
            /*******************************************************************************/
            /* send inputMessage struct to queue and output if the send succeded or failed */
            if (xQueueSend(input_queue, &inputMessage, 10) == pdTRUE)
            {
                printf("(getAccelData) succesfully sent value and type to input queue\n");
            }
            else
            {
                printf("(getAccelData) failed to send value and type to input queue\n");
            }
            /*******************************************************************************/
            xSemaphoreGive(changeInputMessageMutex); // release the mutex at the end of critical section
        }
        else
        {
            printf("(getAccelData) Wasn't able to take changeInputMessageMutex");
        }
        /*---CRITICAL SECTION---*/
    }
}

// get the data from the microphone and push it to input queue
static void getMicData(void *parameter)
{
    printf("intialized getMicData task\n");
    while (true)
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
        int noise_value = noise_sum / 1024;
        /*---CRITICAL SECTION---*/
        if (xSemaphoreTake(changeInputMessageMutex, 30) == pdTRUE)
        { // see if task can take the changeInputMessageMutex
            /**********************************************************/
            inputMessage.value = noise_value; // Sets inputMessage.value to the value inputted by the sensor
            inputMessage.type = "mic";        // Sets the type of value to "mic"
            /*******************************************************************************/
            /* send inputMessage struct to queue and output if the send succeded or failed */
            if (xQueueSend(input_queue, &inputMessage, 10) == pdTRUE)
            {
                printf("(getMicData) succesfully sent value of %d and type of %s to input queue\n", inputMessage.value, inputMessage.type);
            }
            else
            {
                printf("(getMicData) failed to send value of %d and type of %s to input queue\n", inputMessage.value, inputMessage.type);
            }
            /*******************************************************************************/
            xSemaphoreGive(changeInputMessageMutex); // release the mutex at the end of critical section
        }
        else
        {
            printf("(getMicData) Wasn't able to take changeInputMessageMutex");
        }
        /*---CRITICAL SECTION---*/
    }
}

lv_obj_t * sensor_btnm;
lv_obj_t * gsr_chart;
lv_obj_t *gsr_text_area;
lv_obj_t * gsr_timer_bar;
lv_obj_t * start_btn;

bool isBarTimerComplete=true;

void barTimerHandler(void *param) {
    int counter = 0;
    while(true){
        if(isBarTimerComplete==false) {
            counter++;
            printf("hello %d\n",counter);
            lv_bar_set_value(gsr_timer_bar, counter, LV_ANIM_ON);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void start_btn_event_handler(lv_obj_t * obj, lv_event_t event) {
    isBarTimerComplete = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    isBarTimerComplete = true;
    lv_bar_set_value(gsr_timer_bar, 100, LV_ANIM_ON);
}

static void sensor_btnm_event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_set_hidden(sensor_btnm,true);
        const char * txt = lv_btnmatrix_get_active_btn_text(obj);
        printf("%s was pressed\n", txt);
        if(strcmp(txt,"GSR")==0) {
            // gsr button was pressed
            gsr_text_area = lv_textarea_create(lv_scr_act(), NULL);
            lv_obj_set_hidden(gsr_text_area,false);
            lv_obj_set_size(gsr_text_area, 147, 100);
            lv_obj_align(gsr_text_area, NULL, LV_ALIGN_IN_LEFT_MID, 10, 0);
            lv_textarea_set_cursor_hidden(gsr_text_area, true);
            char gsr_text[100];
            sprintf(gsr_text,"GSR: %d\n--------------------\nMoving Average: %d",1000,6000);
            lv_textarea_set_text(gsr_text_area,gsr_text);   
            /*********/
            /*************/
            /* Timer Bar */
            gsr_timer_bar = lv_bar_create(lv_scr_act(), NULL);
            lv_obj_set_hidden(gsr_timer_bar,false);
            lv_obj_set_size(gsr_timer_bar, 147, 15);
            lv_obj_align(gsr_timer_bar, NULL, LV_ALIGN_IN_RIGHT_MID, -10, 40);
            lv_bar_set_anim_time(gsr_timer_bar, 1);
            /*************/
            /* Chart */
            gsr_chart = lv_chart_create(lv_scr_act(), NULL);
            lv_obj_set_hidden(gsr_chart,false);
            lv_obj_set_size(gsr_chart, 300, 50);
            lv_obj_align(gsr_chart, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -5);
            lv_chart_set_type(gsr_chart, LV_CHART_TYPE_LINE); 

            lv_chart_series_t * ser1 = lv_chart_add_series(gsr_chart, LV_COLOR_RED);

            lv_chart_set_next(gsr_chart, ser1, 10);

            lv_chart_refresh(gsr_chart);
            /*********/
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
            lv_obj_set_hidden(start_btn,false);
            lv_obj_align(start_btn, NULL, LV_ALIGN_IN_RIGHT_MID, -20, 0);
            lv_obj_add_style(start_btn, LV_BTN_PART_MAIN, &style_halo);

            lv_obj_set_style_local_value_str(start_btn, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, "Start");
            lv_obj_set_event_cb(start_btn,start_btn_event_handler);
        } else if(strcmp(txt,"Mic")==0) {
            // mic button was pressed
        } else {
            // accel button was pressed
        }
    }
}

static void home_btn_event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        lv_obj_set_hidden(sensor_btnm,false);
        lv_obj_set_hidden(gsr_chart,true);
        lv_obj_set_hidden(gsr_text_area,true);
        lv_obj_set_hidden(gsr_timer_bar,true);
        lv_obj_set_hidden(start_btn,true);
        printf("Home Button Clicked\n");
    }
}

// plot input recieved from input queue on line graph on display
void plotInput(void *parameter)
{
    /***************/
    /* Home Button */
    lv_obj_t * home_btn = lv_btn_create(lv_scr_act(), NULL);
    lv_obj_align(home_btn, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);
    lv_obj_set_style_local_value_str(home_btn, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, "Park It!");
    lv_obj_set_event_cb(home_btn, home_btn_event_handler);
    /***************/

    /*************************************/
    /* Sensor Button Matrix on Home Page */
    static const char * sensor_btnm_map[] = {"GSR", "Mic", "Accel",""};

    sensor_btnm = lv_btnmatrix_create(lv_scr_act(), NULL);
    lv_btnmatrix_set_map(sensor_btnm, sensor_btnm_map);
    lv_obj_align(sensor_btnm, NULL, LV_ALIGN_CENTER, 5, 10);
    lv_obj_set_event_cb(sensor_btnm, sensor_btnm_event_handler);
    /*************************************/

    struct InputMessage rMessage; // struct of type InputMessage to store the recieved message

    printf("initialized plotInput task\n"); // signals that plotInput is initialized
    while (true)
    {
        if (input_queue != NULL)
        { // if input queue has some messages in it
            /************************************/
            /*
            recieve InputMessage struct from queue, 
            assign it to rMessage, and 
            print if it succeded or failed 
            */
            if (xQueueReceive(input_queue, &rMessage, 5) == pdTRUE)
            {
                // SUCCESS
                char *accel = "accel";
                if (rMessage.type != accel)
                {
                    printf("(plotInput) succesfully recieved value of %d and type of %s from input queue\n", rMessage.value, rMessage.type);
                    char *gsr = "gsr";
                    if (rMessage.type == gsr)
                    {
                        gsr_value = rMessage.value;
                    }
                    else
                    {
                        mic_value = rMessage.value;
                    }
                }
                else
                {
                    printf("(plotInput) array of floats was recieved, type is accel: ");
                    for (int i = 0; i < 3; i++)
                    {
                        printf("%f ", rMessage.accel[i]);
                    }
                    if (true)
                    { // to eliminate spam input
                        for (int i = 0; i < 3; i++)
                        {
                            accel_value[i] = rMessage.accel[i];
                        }
                    }
                    printf("\n");
                }
            }
            else
            {
                // FAILURE
                // printf("(plotInput) failed to recieve value and type from input queue\n");
            }
            /************************************/
        }
    }
}