#include "core2forAWS.h"

static QueueHandle_t input_queue = NULL; // queue to store input from sensors
static int INPUT_QUEUE_LEN = 10; // length of input queue

// strut which is sent to the queue in getGsrInput func
struct InputMessage {
    int value; // value retrieved from sensor
    char *type; // type of value (GSR, Heartrate, etc.)
} inputMessage;

// get the GSR input from Port B and send it into the gsr queue
void getGsrInput(void *parameter) {
    printf("initialized readGSRInput task\n"); // signals that getGsrInput is initialized
    while(true) {
        Core2ForAWS_Port_PinMode(PORT_B_ADC_PIN, ADC); // Sets Port B (ADC) as the pinmode
        /**********************************************************/
        /* average 10 input values to get a more accurate reading */
        int gsr_value = 0;
        for(int i = 0; i<10; i++) {
            gsr_value+=Core2ForAWS_Port_B_ADC_ReadRaw();
        }
        gsr_value /= 10;
        /**********************************************************/
        inputMessage.value = gsr_value; // Sets inputMessage.value to the value inputted by the sensor
        inputMessage.type = "gsr"; // Sets the type of value to "gsr"
        /*******************************************************************************/
        /* send inputMessage struct to queue and output if the send succeded or failed */
        if(xQueueSend(input_queue,&inputMessage,10)==pdTRUE) {
            printf("(readGsrInput) succesfully sent value of %d and type of %s to input queue\n",inputMessage.value,inputMessage.type);
        } else {
            printf("(readGsrInput) failed to send value of %d and type of %s to input queue\n",inputMessage.value,inputMessage.type);
        }
        /*******************************************************************************/
        vTaskDelay(pdMS_TO_TICKS(1000)); // to make the output in the monitor more readable
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
                printf("(plotInput) succesfully recieved value of %d and type of %s from input queue\n",rMessage.value,rMessage.type);
            } else {
                // FAILURE
                printf("(plotInput) failed to recieve value and type from input queue\n");
            }
            /************************************/
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // to make recieve (plotInput) synchronize with send
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
        printf("There was an error in creating the input queue");
    }

    /****************/
    /* Create Tasks */

    xTaskCreatePinnedToCore(
        getGsrInput,
        "read gsr input",
        2000,
        NULL,
        1,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        plotInput,
        "plot input",
        2000,
        NULL,
        1,
        NULL,
        0
    );

    /****************/
}