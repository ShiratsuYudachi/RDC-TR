#include "AppConfig.h"   // Include our customized configuration
#include "DJIMotor.hpp"  // Include DR16
#include "DR16.hpp"
#include "FreeRTOS.h"  // Include FreeRTOS.h
#include "PID.hpp"     // Include PID
#include "main.h"
#include "task.h"  // Include task
#include "RemoteControl.hpp"

/*Allocate the stack for our PID task*/
StackType_t uxPIDTaskStack[configMINIMAL_STACK_SIZE];
StackType_t uxDR16TaskStack[configMINIMAL_STACK_SIZE];
StackType_t uxRCTaskStack[configMINIMAL_STACK_SIZE];
/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;
StaticTask_t xDR16TaskTCB;
StaticTask_t xRCTaskTCB;

/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
void userTask(void *)
{
    /* Your user layer codes begin here*/
    /*=================================================*/

    // initialize the PID controller

    //initialize the CAN bus

    //initialize the motor

    /* Your user layer codes end here*/
    /*=================================================*/
    while (true)
    {
        /* Your user layer codes in loop begin here*/
        /*=================================================*/

        // set the target velocity of the motor

        /* Your user layer codes in loop end here*/
        /*=================================================*/

        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}

/**
 * @todo In case you like it, please implement your own tasks
 */
void dr16Task(void *)
{
    DR16::init();
    DR16::startReceiveRC();

    DR16::DR16Data *data = DR16::getDR16Data();

    while (1)
    {
        DR16::connectionCheck();

        vTaskDelay(1);
    }
}

void rcTask(void *)
{
    while (1)
    {
        RemoteControl::classis_set_update();
        RemoteControl::forward_kinematics();
        RemoteControl::classis_pid_calculate();

        vTaskDelay(1);
    }
}

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
*/
void startUserTasks()
{
    // DJIMotor::init();  // Initalize the DJIMotor driver
    // DR16::init();      // Intialize the DR16 driver

    xTaskCreateStatic(userTask,
                      "user_default ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      1,
                      uxPIDTaskStack,
                      &xPIDTaskTCB);  // Add the main task into the scheduler
    
    xTaskCreateStatic(dr16Task,
                      "dr16Task",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      1,
                      uxDR16TaskStack,
                      &xDR16TaskTCB);

    xTaskCreateStatic(rcTask,
                      "rcTask",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      1,
                      uxRCTaskStack,
                      &xRCTaskTCB);
}