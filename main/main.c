#include "core2forAWS.h"

QueueHandle_t input_queue; // que to store input from sensors
static int INPUT_QUEUE_LEN = 10; 

struct InputMessage {
    int value; // value retrieved from sensor
    char *type; // type of value (GSR, Heartrate, etc.)
} xMessage;

// get the GSR input from Port B and send it into the gsr queue
void getGsrInput(void *parameter) {
    struct InputMessage *pxInputMessage = malloc((sizeof(char)+sizeof(int))*2); // create pxInputMessage struct
    printf("initialized readGSRInput task");
    while(true) {
        Core2ForAWS_Port_PinMode(PORT_B_ADC_PIN, ADC); // Set Port B (ADC) as the pinmode
        // average 10 input values to get a more accurate reading
        int gsr_value = 0;
        for(int i = 0; i<10; i++) {
            gsr_value+=Core2ForAWS_Port_B_ADC_ReadRaw();
        }
        gsr_value /= 10;
        // set pxInputMessage.value and pxInputMessage.type
        pxInputMessage->value = gsr_value; 
        pxInputMessage->type = "GSR"; 
        // send pxInputMessage struct to queue and print if it succeded or failed
        if(xQueueSend(input_queue,(void *) pxInputMessage,10)!=pdTRUE) {
            printf("(readGsrInput) failed to send value of %d and type of %s to input queue",pxInputMessage->value,pxInputMessage->type);
        } else {
            printf("(readGsrInput) succesfully sent value of %d and type of %s to input queue",pxInputMessage->value,pxInputMessage->type);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // to make the output in the monitor more readable
    }
}

void app_main(void){
    Core2ForAWS_Init(); // initialize the Core 2 for AWS BSP lib

    // initialize input queue with INPUT_QUE_LEN of InputMessages
    input_queue = xQueueCreate(INPUT_QUEUE_LEN,sizeof(struct InputMessage *)); 

    /****************/
    /* Create Tasks */

    xTaskCreatePinnedToCore(
        getGsrInput,
        "read gsr input",
        5000,
        NULL,
        1,
        NULL,
        0
    );

    /****************/
}