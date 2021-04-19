/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* UART header file*/
#include <ti/drivers/UART.h>

/* Timer header file*/
#include <ti/drivers/Timer.h>

/* I2C header file*/
#include <ti/drivers/I2C.h>



// Task Scheduler setup
typedef struct task {
    int state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*TickFct)(int);
} task;

task tasks[3];

const unsigned char tasksNum = 3;
const unsigned long tasksPeriodGCD = 100;
const unsigned periodCheckButtons = 200;
const unsigned periodCheckTemperature = 500;
const unsigned periodDisplayOutput = 1000;

enum ButtonActions {EX_BN_Start, EX_BN_IncreaseTemp, EX_BN_DecreaseTemp};
int TickFct_checkButtons(int state);

enum TempActions {EX_SM_Start, EX_SM_IncreaseTemp, EX_SM_DecreaseTemp };
int TickFct_checkTemperature(int state, int16_t setpoint_int);

enum DisplayActions{EX_DS_Start, EX_DS_Display};
int TickFct_displayOutput(int state, int16_t setpoint_int, bool active_heat, int seconds);

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global Variables
I2C_Handle i2c;

// Driver handles - Global Variables
UART_Handle uart;

// UART Global Variables
char            output[64];
int             bytesToSend;

int16_t         temperature;
bool            active_heat;
int             seconds;


// Driver handles - global variables
Timer_Handle        timer0;

#define DISPLAY(x) UART_write(uart, &output, x);

void initI2C(void);
void readTemp(void);
void initTimer(void);
void initUART(void);
void init(void);
bool checkTemp(int16_t, int16_t);
void toggleLED(bool active_heat);

volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void gpioButtonFxn0(uint_least8_t index)
{
    tasks[0].state = EX_BN_IncreaseTemp;
}

void gpioButtonFxn1(uint_least8_t index)
{
    tasks[0].state = EX_BN_DecreaseTemp;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    char    setpoint[2];
    const char echoPrompt[] = "Enter initial Set Point:\r\n";
    size_t size = 2;
    int seconds = 0;
    active_heat = false;

    unsigned char i=0;
    tasks[i].state = EX_BN_Start;
    tasks[i].period = periodCheckButtons;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &TickFct_checkButtons;
    ++i;
    tasks[i].state = EX_SM_Start;
    tasks[i].period = periodCheckTemperature;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &TickFct_checkTemperature;
    ++i;
    tasks[i].period = periodDisplayOutput;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &TickFct_displayOutput;

    init();

    UART_write(uart, echoPrompt, sizeof(echoPrompt));

    // Set initial set point
    UART_read(uart,  &setpoint, size);
    int16_t setpoint_int = atoi(setpoint);

    unsigned long BN_elapsedTime = 200;
    unsigned long SM_elapsedTime = 500;
    unsigned long DS_elapsedTime = 1000;

    // Loop Forever
    while(1)
    {
        if (BN_elapsedTime >= 200)
        {
           TickFct_checkButtons(tasks[0].state);
           BN_elapsedTime = 0;
        }

        if (SM_elapsedTime >= 500)
        {
            TickFct_checkTemperature(tasks[1].state, setpoint_int);
            SM_elapsedTime = 0;
        }

        if (DS_elapsedTime >= 1000)
        {
            TickFct_displayOutput(tasks[2].state, setpoint_int, active_heat, seconds);
            DS_elapsedTime = 0;
        }



        while (!TimerFlag){}
        TimerFlag = 0;
        BN_elapsedTime += tasksPeriodGCD;
        SM_elapsedTime += tasksPeriodGCD;
        DS_elapsedTime += tasksPeriodGCD;
        seconds += 1;
    }
}

int TickFct_checkButtons(int state)
{
    switch (state)
    {
        case EX_BN_IncreaseTemp:
            temperature++;
            break;
        case EX_BN_DecreaseTemp:
            temperature--;
            break;
        default:
            break;
    }

    return EX_BN_Start;
}

int TickFct_checkTemperature(int state, int16_t setpoint_int)
{
    active_heat = checkTemp(setpoint_int, temperature);
    toggleLED(active_heat);

    return state;
}

int TickFct_displayOutput(int state, int16_t setpoint_int, bool active_heat, int seconds)
{
    DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint_int, active_heat, seconds))
    return EX_DS_Start;
}

// Make sure you call initUART() before calling this function
void initI2C(void)
{
    int8_t          i, found;
    I2C_Params      i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Initialize the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while(1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    /*
     * Boards were shipped with different sensors.
     * Welcome to the world of embedded systems.
     * Try to determine which sensor we have.
     * Scan through the possible sensor addresses.
     */

    // Common I2C transaction setup
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i = 0; i < 3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }

        DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if (found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id ,i2cTransaction.slaveAddress));
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

void readTemp(void)
{
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data
         * see TMP sensor datasheet
         */

        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
         * If the MSB is set '1', then we have 2's complement negative
         * value which needs to be sign executed
         */

        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
}

void initTimer(void)
{
    Timer_Params        params;

    // Initialize the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL)
    {
        // Failed to initialize Timer
        while(1){}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR)
    {
        // Failed to start timer
        while(1){}
    }
}

void initUART(void)
{
    UART_Params uartParams;

    // Initialize the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL)
    {
        // UART failed to open
        while(1);
    }
}

void init(void)
{
    GPIO_init();
    initUART();
    initI2C();
    initTimer();

    readTemp();

    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    // Install Button callback
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    // Enable Interrupts
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    GPIO_write(CONFIG_GPIO_LED_0, 0);
}

bool checkTemp(int16_t setpoint_int, int16_t temperature)
{
    if (setpoint_int > temperature)
    {
        tasks[0].state = EX_SM_IncreaseTemp;
        return true;
    }
    else if (setpoint_int < temperature)
    {
        tasks[0].state = EX_SM_DecreaseTemp;
        return false;
    }
    else {
        tasks[0].state = EX_SM_Start;
        return false;
    }
}

void toggleLED(bool active_heat)
{
    if (active_heat)
    {
        GPIO_write(CONFIG_GPIO_LED_0, 1);
    }
    else
    {
        GPIO_write(CONFIG_GPIO_LED_0, 0);
    }
}
