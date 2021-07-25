#include "core2forAWS.h"

static QueueHandle_t input_queue = NULL; // queue to store input from sensors
static int INPUT_QUEUE_LEN = 10; // length of input queue

// struct which is sent to the queue in getGsrInput func
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
        vTaskDelay(pdMS_TO_TICKS(500)); // to make the output in the monitor more readable
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
        /*---CRITICAL SECTION---*/
        if(xSemaphoreTake(changeInputMessageMutex, 30) == pdTRUE) { // see if task can take the changeInputMessageMutex
            /**********************************************************/
            inputMessage.value = noise_value; // Sets inputMessage.value to the value inputted by the sensor
            inputMessage.type = "mic"; // Sets the type of value to "mic"
            /*******************************************************************************/
            /* send inputMessage struct to queue and output if the send succeded or failed */
            if(xQueueSend(input_queue,&inputMessage,10)==pdTRUE) {
                printf("(getMicData) succesfully sent value of %d and type of %s to input queue\n",inputMessage.value,inputMessage.type);
            } else {
                printf("(getMicData) failed to send value of %d and type of %s to input queue\n",inputMessage.value,inputMessage.type);
            }
            /*******************************************************************************/
            xSemaphoreGive(changeInputMessageMutex); // release the mutex at the end of critical section
        } else {
            printf("(getMicData) Wasn't able to take changeInputMessageMutex");
        }
        /*---CRITICAL SECTION---*/
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// plot input recieved from input queue on line graph on display  
void plotInput(void *parameter) {
    /***************************************************/
    /* Create Main Text Box on screen */
    lv_obj_t * page = lv_page_create(lv_scr_act(), NULL);
    lv_obj_set_size(page, 300, 200);
    lv_obj_align(page, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t * label = lv_label_create(page, NULL);
    lv_label_set_long_mode(label, LV_LABEL_LONG_BREAK);       
    lv_obj_set_width(label, lv_page_get_width_fit(page));  
    /***************************************************/       

    struct InputMessage rMessage; // struct of type InputMessage to store the recieved message

    /***********************************************/
    /* intialize default values for data variables */
    int gsr_value = 0;
    float accel_value[3] = {0.0f,0.0f,0.0f};
    int mic_value = 0;
    /***********************************************/

    char text[100]; // variable to hold text to be displayed on screen

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
                    char *gsr="gsr";
                    if(rMessage.type==gsr) {
                        gsr_value = rMessage.value;
                    } else {
                        mic_value = rMessage.value;
                    }
                } else {
                    printf("(plotInput) array of floats was recieved, type is accel: ");
                    for(int i = 0; i<3; i++) {
                        printf("%f ",rMessage.accel[i]);
                    }
                    if(accel_value[0]!=-57897880075350303304673394688.00) { // to eliminate spam input
                        for(int i = 0; i<3; i++) {
                            accel_value[i] = rMessage.accel[i];
                        }
                    }
                    printf("\n");
                }
            } else {
                // FAILURE
                // printf("(plotInput) failed to recieve value and type from input queue\n");
            }
            /************************************/
        }
        
        // format text string to include the values
        sprintf(text,"Park It!\nGSR: %d\nMic: %d\nAccel: %.2f %.2f %.2f",gsr_value,mic_value,accel_value[0],accel_value[1],accel_value[2]);
        printf("(plotInput):\n%s\n",text);

        lv_label_set_text(label, text);
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
        4096*2,
        NULL,
        1,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        getAccelData,
        "get accel data",
        4096*2,
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
        4096*2,
        NULL,
        1,
        NULL,
        0
    );

    /****************/
}