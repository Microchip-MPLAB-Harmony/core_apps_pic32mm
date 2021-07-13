/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_task2.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app_task2.h"
#include "definitions.h"
#include <string.h>
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_TASK2_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_TASK2_DATA app_task2Data;
extern SemaphoreHandle_t uartMutexLock;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_TASK2_Initialize ( void )

  Remarks:
    See prototype in app_task2.h.
 */

void APP_TASK2_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_task2Data.state = APP_TASK2_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_TASK2_Tasks ( void )

  Remarks:
    See prototype in app_task2.h.
 */

void APP_TASK2_Tasks ( void )
{
    TickType_t timeNow;
    
    while (1)
    {        
        /* Task2 is running (<-) now */
        xSemaphoreTake(uartMutexLock, portMAX_DELAY);        
        UART2_Write((uint8_t*)"           Tsk2-P2 <-\r\n", 23);
        xSemaphoreGive(uartMutexLock); 
        
        /* Work done by task2 for 10 ticks */
        timeNow = xTaskGetTickCount();
        while ((xTaskGetTickCount() - timeNow) < 10);
        
        /* Task2 is exiting (->) now */
        xSemaphoreTake(uartMutexLock, portMAX_DELAY);        
        UART2_Write((uint8_t*)"           Tsk2-P2 ->\r\n", 23);
        xSemaphoreGive(uartMutexLock);   
        
        /* Run the task again after 250 msec */
        vTaskDelay(250 / portTICK_PERIOD_MS );        
    }
}


/*******************************************************************************
 End of File
 */
