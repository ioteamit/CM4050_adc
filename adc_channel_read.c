/*********************************************************************************

Copyright(c) 2016 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/
/*!
* @file      adc_channel_read.c
* @brief     Example to demonstrate ADC Driver
*
* @details
*            This is the primary source file for the ADC example, showing how
*            to read an analog input channel.
*
*/

/*=============  I N C L U D E S   =============*/

/* ADC example include */
#include "adc_channel_read.h"
/* Managed drivers and/or services include */
#include <drivers/adc/adi_adc.h>

#include "common.h"
#include "drivers/pwr/adi_pwr.h"
#include <assert.h>
#include <stdio.h>
#include <drivers/general/adi_drivers_general.h>


#ifdef ADI_DBM_TEST
#include <drivers/gpio/adi_gpio.h>

#define ACC_IO_PORT         ADI_GPIO_PORT0
#define ACC_IO_PORT2        ADI_GPIO_PORT2
#define ACC_RESET_PIN       ADI_GPIO_PIN_14
#define ACC_TEMP_PIN        ADI_GPIO_PIN_7
#define ACC_RANGE_PIN       ADI_GPIO_PIN_9
#define ACC_ST1_PIN         ADI_GPIO_PIN_3
#define ACC_ST2_PIN         ADI_GPIO_PIN_12

#define MAGN_IO_PORT        ADI_GPIO_PORT0
#define MAGN_ERROR_PIN      ADI_GPIO_PIN_4
#define MAGN_OR_PIN         ADI_GPIO_PIN_5
#define MAGN_PWR_ENA_PIN    ADI_GPIO_PIN_8

#define DUST_IO_PORT        ADI_GPIO_PORT0
#define DUST_RESET_PIN      ADI_GPIO_PIN_1


#define ANL_IO_PORT         ADI_GPIO_PORT1
#define ANL_SELECTION_PIN   ADI_GPIO_PIN_3
#define ACC 0
#define MGN 1

#define TEMP_IO_PORT        ADI_GPIO_PORT1
#define TEMP_ENA_PIN        ADI_GPIO_PIN_5

#define MIC_IO_PORT         ADI_GPIO_PORT1
#define MIC_ENA_PIN         ADI_GPIO_PIN_4
#endif

/*=============  D A T A  =============*/

#ifdef ADI_DBM_TEST
static uint32_t ports;
    static uint8_t gpioMemory[ADI_GPIO_MEMORY_SIZE];
#endif

/* ADC Handle */
ADI_ALIGNED_PRAGMA(4)
ADI_ADC_HANDLE hDevice ADI_ALIGNED_ATTRIBUTE(4);

/* Memory Required for adc driver */
ADI_ALIGNED_PRAGMA(4)
static uint8_t DeviceMemory[ADI_ADC_MEMORY_SIZE] ADI_ALIGNED_ATTRIBUTE(4);

/* ADC Data Buffer */
ADI_ALIGNED_PRAGMA(4)
static uint16_t ADC_DataBuffer[ADC_NUM_SAMPLES] ADI_ALIGNED_ATTRIBUTE(4) = {0};

/*=============  E X T E R N A L    F U N C T I O N S  =============*/
extern int32_t adi_initpinmux(void);


/*=============  L O C A L    F U N C T I O N S  =============*/

static void usleep(uint32_t usec);
static void WriteSamplesToFile (void);

/*=============  C O D E  =============*/

void ADC_SampleData(void)
{
    ADI_ADC_RESULT  eResult = ADI_ADC_SUCCESS;
    ADI_ADC_BUFFER Buffer;
    bool bCalibrationDone = false;

    /* Open the ADC device */
    eResult = adi_adc_Open(ADC_DEV_NUM, DeviceMemory, sizeof(DeviceMemory), &hDevice);
    DEBUG_RESULT("Failed to open ADC device",eResult, ADI_ADC_SUCCESS);

    /* Power up ADC */
    eResult = adi_adc_PowerUp (hDevice, true);
    DEBUG_RESULT("Failed to power up ADC", eResult, ADI_ADC_SUCCESS);

    /* Set ADC reference */
#ifdef ADI_DBM_TEST
    eResult = adi_adc_SetVrefSource (hDevice, ADI_ADC_VREF_SRC_EXT);
#else
    eResult = adi_adc_SetVrefSource (hDevice, ADI_ADC_VREF_SRC_INT_2_50_V);
#endif    
    DEBUG_RESULT("Failed to set ADC reference", eResult, ADI_ADC_SUCCESS);

    /* Enable ADC sub system */
    eResult = adi_adc_EnableADCSubSystem (hDevice, true);
    DEBUG_RESULT("Failed to enable ADC sub system", eResult, ADI_ADC_SUCCESS);

    /* Wait for 5.0ms */
    usleep (5000);

    /* Start calibration */
    eResult = adi_adc_StartCalibration (hDevice);
    DEBUG_RESULT("Failed to start calibration", eResult, ADI_ADC_SUCCESS);

    /* Wait until calibration is done */
    while (!bCalibrationDone)
    {
        eResult = adi_adc_IsCalibrationDone (hDevice, &bCalibrationDone);
        DEBUG_RESULT("Failed to get the calibration status", eResult, ADI_ADC_SUCCESS);
    }

    /* Set the delay time */
    eResult = adi_adc_SetDelayTime ( hDevice, 10);
    DEBUG_RESULT("Failed to set the Delay time ", eResult, ADI_ADC_SUCCESS);

    /* Set the acquisition time. (Application need to change it based on the impedence) */
    eResult = adi_adc_SetAcquisitionTime ( hDevice, 10);
    DEBUG_RESULT("Failed to set the acquisition time ", eResult, ADI_ADC_SUCCESS);

    /* Populate the buffer structure */
    Buffer.nBuffSize = sizeof(ADC_DataBuffer);
#ifdef ADI_DBM_TEST
    ports = ADI_ADC_CHANNEL_0 | ADI_ADC_CHANNEL_1 | ADI_ADC_CHANNEL_2 | ADI_ADC_CHANNEL_3 | ADI_ADC_CHANNEL_4 | ADI_ADC_CHANNEL_5 | ADI_ADC_CHANNEL_6 | ADI_ADC_CHANNEL_7 ;
        Buffer.nChannels = ports;
#else
        Buffer.nChannels = ADI_ADC_CHANNEL_0;
#endif    
    Buffer.nNumConversionPasses = ADC_NUM_SAMPLES;
    Buffer.pDataBuffer = ADC_DataBuffer;

    /* Submit the buffer to the driver */
    eResult = adi_adc_SubmitBuffer (hDevice, &Buffer);
    DEBUG_RESULT("Failed to submit buffer ", eResult, ADI_ADC_SUCCESS);

    /* Enable the ADC */
    eResult = adi_adc_Enable (hDevice, true);
    DEBUG_RESULT("Failed to enable the ADC for sampling ", eResult, ADI_ADC_SUCCESS);

    ADI_ADC_BUFFER* pAdcBuffer;
    eResult = adi_adc_GetBuffer (hDevice, &pAdcBuffer);
    DEBUG_RESULT("Failed to Get Buffer ", eResult, ADI_ADC_SUCCESS);

    /* Disable the ADC */
    eResult = adi_adc_Enable (hDevice, false);
    DEBUG_RESULT("Failed to disable ADC for sampling ", eResult, ADI_ADC_SUCCESS);

    /* Close the ADC */
    eResult = adi_adc_Close (hDevice);
    DEBUG_RESULT("Failed to close ADC", eResult, ADI_ADC_SUCCESS);
}
#ifdef ADI_DBM_TEST
static ADI_GPIO_RESULT digitalOut(const char port, const uint16_t pin, bool high) {
        if (high) {
            return adi_gpio_SetHigh((ADI_GPIO_PORT)port, (ADI_GPIO_DATA)pin);
        } else {
            return adi_gpio_SetLow((ADI_GPIO_PORT)port, (ADI_GPIO_DATA)pin);
        }
    }

static void initGPIO(void) {

  /* init the GPIO service */
        if(ADI_GPIO_SUCCESS != adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE))
        {
            DEBUG_MESSAGE("adi_gpio_Init failed\n");
            return;
        }
        
        adi_gpio_OutputEnable((ADI_GPIO_PORT)ACC_IO_PORT, (ADI_GPIO_DATA)ACC_RESET_PIN, true);
        adi_gpio_OutputEnable((ADI_GPIO_PORT)ANL_IO_PORT, (ADI_GPIO_DATA)ANL_SELECTION_PIN, true);
          
        digitalOut((ADI_GPIO_PORT)ACC_IO_PORT, (ADI_GPIO_DATA)ACC_RESET_PIN, true);
        
        /**
          MGN = 0.8
          ACC = 2
        **/
        digitalOut((ADI_GPIO_PORT)ANL_IO_PORT, (ADI_GPIO_DATA)ANL_SELECTION_PIN, MGN);
}
#endif
/*********************************************************************
*
*   Function:   main
*
*********************************************************************/
int main (void)
{
    /* test system initialization */
    common_Init();

    if (adi_pwr_Init()!= ADI_PWR_SUCCESS)
    {
        DEBUG_MESSAGE("\n Failed to intialize the power service \n");
    }
    if (ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_HCLK,1))
    {
        DEBUG_MESSAGE("Failed to intialize the power service\n");
    }
    if (ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_PCLK,1))
    {
        DEBUG_MESSAGE("Failed to intialize the power service\n");
    }

    /* Configure the Pin Mux for ADC */
    adi_initpinmux();
    
#ifdef ADI_DBM_TEST
    initGPIO();
#endif
    /* Sample the ADC Channel 0 */
    ADC_SampleData();

    /* Write the samples to file */
    WriteSamplesToFile();

    /* Test Passed */
    common_Pass();
}

/* Write the samples to file */
static void WriteSamplesToFile (void)
{
#ifdef WRITE_SAMPLES_TO_FILE
    int x;
    FILE* fp;

    fp = fopen("./ADC_Out.dat", "w");
    if (fp == NULL) {
        common_Fail("Failed open file\n");
        return;
    }
    for (x = 0; x < ADC_NUM_SAMPLES; x++) {
        fprintf(fp,"%d\n", ADC_DataBuffer[x]);
    }

    fclose(fp);
#endif /* WRITE_SAMPLES_TO_FILE */
}

/* Approximately wait for minimum 1 usec */
static void usleep(uint32_t usec)
{
    volatile int y = 0;
    while (y++ < usec) {
        volatile int x = 0;
        while (x < 16) {
        x++;
        }
    }
}

/*****/
