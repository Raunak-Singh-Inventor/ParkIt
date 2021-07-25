#include "core2forAWS.h"

static QueueHandle_t input_queue = NULL; // queue to store input from sensors
static int INPUT_QUEUE_LEN = 10; // length of input queue

// strut which is sent to the queue in getGsrInput func
struct InputMessage {
    int value; // value retrieved from sensor
    float accel[3]; // special array if type is accel
    char *type; // type of value (GSR, Heartrate, etc.)
} inputMessage;

// mutex to control whether input message can be changed
// needed to stop multiple tasks from overwriting the inputMessage at the same time
static SemaphoreHandle_t changeInputMessageMutex;

// get the GSR input from Port B and send it into the gsr queue
void getGsrInput(void *parameter) {
    printf("initialized getGSRInput task\n"); // signals that getGsrInput is initialized
    while(true) {
        Core2ForAWS_Port_PinMode(PORT_B_ADC_PIN, ADC); // Sets Port B (ADC) as the pinmode
        /**********************************************************/
        /* average 10 input values to get a more accurate reading */
        int gsr_value = 0;
        for(int i = 0; i<10; i++) {
            gsr_value+=Core2ForAWS_Port_B_ADC_ReadRaw();
        }
        gsr_value /= 10;
        /*---CRITICAL SECTION---*/
        if(xSemaphoreTake(changeInputMessageMutex, 10) == pdTRUE) { // see if task can take the changeInputMessageMutex
            /**********************************************************/
            inputMessage.value = gsr_value; // Sets inputMessage.value to the value inputted by the sensor
            inputMessage.type = "gsr"; // Sets the type of value to "gsr"
            /*******************************************************************************/
            /* send inputMessage struct to queue and output if the send succeded or failed */
            if(xQueueSend(input_queue,&inputMessage,10)==pdTRUE) {
                printf("(getGsrInput) succesfully sent value of %d and type of %s to input queue\n",inputMessage.value,inputMessage.type);
            } else {
                printf("(getGsrInput) failed to send value of %d and type of %s to input queue\n",inputMessage.value,inputMessage.type);
            }
            /*******************************************************************************/
            xSemaphoreGive(changeInputMessageMutex); // release the mutex at the end of critical section
        } else {
            printf("(getGsrInput) Wasn't able to take changeInputMessageMutex");
        }
        /*---CRITICAL SECTION---*/
        vTaskDelay(pdMS_TO_TICKS(700)); // to make the output in the monitor more readable
    }
}

// get the data from the 6-axis IMU and push it to input queue
void getAccelData(void* parameter){
    float accel[3]; // array of accel in format x,y,z
 
    printf("intialized getAccelData task\n");
    while(true){
        MPU6886_GetAccelData(&accel[0], &accel[1], &accel[2]); // get accel data
        /*---CRITICAL SECTION---*/
        if(xSemaphoreTake(changeInputMessageMutex, 20) == pdTRUE) { // see if task can take the changeInputMessageMutex
            /**********************************************************/
            inputMessage.value = 0; // set value to 0 or false
            inputMessage.accel[0] = accel[0];
            inputMessage.accel[1] = accel[1];
            inputMessage.accel[2] = accel[2]; // Sets inputMessage.accel to the accel gotten from sensor
            inputMessage.type = "accel"; // Sets the type of value to "accel"
            /*******************************************************************************/
            /* send inputMessage struct to queue and output if the send succeded or failed */
            if(xQueueSend(input_queue,&inputMessage,10)==pdTRUE) {
                printf("(getAccelData) succesfully sent value and type to input queue\n");
            } else {
                printf("(getAccelData) failed to send value and type to input queue\n");
            }
            /*******************************************************************************/
            xSemaphoreGive(changeInputMessageMutex); // release the mutex at the end of critical section
        } else {
            printf("(getAccelData) Wasn't able to take changeInputMessageMutex");
        }
        /*---CRITICAL SECTION---*/
        vTaskDelay(pdMS_TO_TICKS(500)); // to make the output in monitor more readable
    }
}

// get the data from the microphone and push it to input queue
static void getMicData(void *parameter){
    printf("intialized getMicData task\n");
    while(true) {
        /* If the speaker was initialized, be sure to call Speaker_Deinit() and 
            disable first. */
        Microphone_Init();
    
        static int8_t i2s_readraw_buf[1024];
        size_t bytes_read;
    
        i2s_read(MIC_I2S_NUMBER, (char*)i2s_readraw_buf, 1024, &bytes_read, pdMS_TO_TICKS(100));
        Microphone_Deinit();
    
        int noise_sum = 0;
        for(uint16_t i = 0; i < 1024; i++){
            noise_sum += i2s_readraw_buf[i];
        }
        int noise_value = noise_sum/1024;
        printf("(getMicData) noise_value: %d\n",noise_value);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// plot input recieved from input queue on line graph on display  
void plotInput(void *parameter) {
    struct InputMessage rMessage; // struct of type InputMessage to store the recieved message
    printf("initialized plotInput task\n"); // signals that plotInput is initialized
    while(true) {
        if(input_queue!=NULL) { // if input queue has some messages in it
            /************************************/
            /*
            recieve InputMessage struct from queue, 
            assign it to rMessage, and 
            print if it succeded or failed 
            */
            if(xQueueReceive(input_queue,&rMessage,5)==pdTRUE) {
                // SUCCESS
                char *accel="accel";
                if(rMessage.type!=accel) {
                    printf("(plotInput) succesfully recieved value of %d and type of %s from input queue\n",rMessage.value,rMessage.type);
                } else {
                    printf("(plotInput) array of floats was recieved, type is accel: ");
                    for(int i = 0; i<3; i++) {
                        printf("%f ",rMessage.accel[i]);
                    }
                    printf("\n");
                }
            } else {
                // FAILURE
                // printf("(plotInput) failed to recieve value and type from input queue\n");
            }
            /************************************/
        }
    }
}

void app_main(void){
    Core2ForAWS_Init(); // initialize the Core 2 for AWS BSP lib
    Core2ForAWS_Display_SetBrightness(80);

    /* 
    initialize input queue with INPUT_QUE_LEN of inputMessage 
    (assign double the memory to avoid LoadProhibited error) 
    */
    input_queue = xQueueCreate(INPUT_QUEUE_LEN,sizeof(inputMessage)*2); 

    // check that input queue was created succesfully
    if(input_queue==NULL) {
        printf("(app_main) There was an error in creating the input queue");
    }

    changeInputMessageMutex = xSemaphoreCreateMutex(); // initialize the mutex

    /****************/
    /* Create Tasks */

    xTaskCreatePinnedToCore(
        getGsrInput,
        "get gsr input",
        6000,
        NULL,
        1,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        getAccelData,
        "get accel data",
        6000,
        NULL,
        1,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        getMicData,
        "get mic data",
        4096*2,
        NULL,
        1,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        plotInput,
        "plot input",
        6000,
        NULL,
        1,
        NULL,
        0
    );

    /****************/
}