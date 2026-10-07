/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the ADC die temperature calculation 
*              Example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include <stdio.h>
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cy_device_headers.h"

/******************************************************************************
* Macros
*******************************************************************************/

#define CY_SAR2_MACRO                           PASS0_SAR0
#define CY_SAR2_MMIO_MACRO                      PASS0_EPASS_MMIO
#define CY_SAR2_PCLK                            PCLK_PASS0_CLOCK_SAR0

#define ADC_LOGICAL_CHANNEL                     0u
#define ADC_CH_NUM_OF_ITERATION                 16u
#define ADC_CH_AVG_RSHIFT_VALUE                 4u
#define ADC_CH_SAMPLE_TIME                      120u
#define ADC_CH_RANGE_DETECT_LOW                 0x0000u
#define ADC_CH_RANGE_DETECT_HIGH                0x0FFFu

#define CY_SAR2_IN_VBG                          CY_SAR2_PIN_ADDRESS_VBG
#define CY_SAR2_IN_TEMP                         CY_SAR2_PIN_ADDRESS_VTEMP



/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Observe these values in the debugger after each 1 s update. */
static volatile uint16_t g_adc_vbg_raw_value = 0u;
static volatile uint16_t g_adc_vtemp_raw_value = 0u;
static volatile double g_die_temperature_c = 0.0;
static cy_stc_scb_uart_context_t g_debug_uart_context;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

static void adc_configure_channel(cy_en_sar2_pin_address_t channel_address);
static uint16_t adc_read_channel(cy_en_sar2_pin_address_t channel_address);
static void init_adc(void);
static void init_debug_uart(void);
static void print_temperature_uart(void);
static double read_die_temperature(void);


/*******************************************************************************
* Function Definitions
*******************************************************************************/

static void adc_configure_channel(cy_en_sar2_pin_address_t channel_address)
{
    cy_stc_sar2_channel_config_t channel_config = ADC_channel_0_config;

    /* Override channel input so VBG/VTEMP can be selected at runtime. */
    channel_config.pinAddress    = channel_address;
    channel_config.portAddress   = CY_SAR2_PORT_ADDRESS_SARMUX0;
    channel_config.extMuxEnable  = false;
    channel_config.interruptMask = CY_SAR2_GRP_COMPLETE;

    Cy_SAR2_Channel_Init(CY_SAR2_MACRO, ADC_LOGICAL_CHANNEL, &channel_config);
    Cy_SAR2_Channel_Enable(CY_SAR2_MACRO, ADC_LOGICAL_CHANNEL);
}

static uint16_t adc_read_channel(cy_en_sar2_pin_address_t channel_address)
{
    uint32_t intr_status;
    uint32_t result_status;

    adc_configure_channel(channel_address);
    Cy_SAR2_Channel_SoftwareTrigger(CY_SAR2_MACRO, ADC_LOGICAL_CHANNEL);

    do
    {
    }
    while ((Cy_SAR2_Channel_GetInterruptStatusMasked(CY_SAR2_MACRO, ADC_LOGICAL_CHANNEL) & CY_SAR2_GRP_COMPLETE) == 0u);

    intr_status = Cy_SAR2_Channel_GetInterruptStatusMasked(CY_SAR2_MACRO, ADC_LOGICAL_CHANNEL);
    Cy_SAR2_Channel_ClearInterrupt(CY_SAR2_MACRO, ADC_LOGICAL_CHANNEL, intr_status);

    return Cy_SAR2_Channel_GetResult(CY_SAR2_MACRO, ADC_LOGICAL_CHANNEL, &result_status);
}

static void init_adc(void)
{
    Cy_SysClk_PeriphAssignDivider(CY_SAR2_PCLK, CY_SYSCLK_DIV_16_BIT, 0u);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 0u, 5u);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 0u);

    Cy_SAR2_Init(CY_SAR2_MACRO, &ADC_config);
    Cy_SAR2_Channel_SetInterruptMask(CY_SAR2_MACRO, ADC_LOGICAL_CHANNEL, CY_SAR2_GRP_COMPLETE);
    Cy_SAR2_Enable(CY_SAR2_MACRO);
}

static void init_debug_uart(void)
{
    Cy_SCB_UART_Init(UART_HW, &UART_config, &g_debug_uart_context);
    Cy_SCB_UART_Enable(UART_HW);
    Cy_SCB_UART_PutString(UART_HW, "\x1b[2J\x1b[;H");
    Cy_SCB_UART_PutString(UART_HW, "-------------------------------------------------------------\r\n");
    Cy_SCB_UART_PutString(UART_HW, "ADC Die Temperature Calculation\r\n");
    Cy_SCB_UART_PutString(UART_HW, "Open Terminal window at 115200-8-N-1 on the KitProg3 COM port\r\n");
    Cy_SCB_UART_PutString(UART_HW, "-------------------------------------------------------------\r\n\r\n");
}

static void print_temperature_uart(void)
{
    char message[256];
    long temp_centi;
    unsigned long temp_abs_centi;
    const char * sign = "";

    temp_centi = (long)(g_die_temperature_c * 100.0);
    if ((g_die_temperature_c < 0.0) && (((double)temp_centi) > (g_die_temperature_c * 100.0)))
    {
        temp_centi--;
    }

    if (temp_centi < 0)
    {
        sign = "-";
        temp_abs_centi = (unsigned long)(-temp_centi);
    }
    else
    {
        temp_abs_centi = (unsigned long)temp_centi;
    }

    snprintf(message,
             sizeof(message),
             "adcVbgRawValue   : %u\r\n"
             "adcVtempRawValue : %u\r\n"
             "Die Temperature  : %s%lu.%02lu C\r\n\r\n",
             (unsigned int)g_adc_vbg_raw_value,
             (unsigned int)g_adc_vtemp_raw_value,
             sign,
             temp_abs_centi / 100u,
             temp_abs_centi % 100u);

    Cy_SCB_UART_PutString(UART_HW, message);
}

static double read_die_temperature(void)
{
    Cy_SAR2_SetReferenceBufferMode(CY_SAR2_MMIO_MACRO, CY_SAR2_REF_BUF_MODE_ON);
    g_adc_vbg_raw_value = adc_read_channel(CY_SAR2_IN_VBG);
    g_adc_vtemp_raw_value = adc_read_channel(CY_SAR2_IN_TEMP);

    return Cy_SAR2_CalculateDieTemperature(CY_SAR2_VDDA_4_5V_TO_5_5V,
                                           g_adc_vtemp_raw_value,
                                           g_adc_vbg_raw_value);
}

/*******************************************************************************
* Function Name: main
*********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    init_debug_uart();
    init_adc();

    for (;;)
    {
        g_die_temperature_c = read_die_temperature();
        print_temperature_uart();
        Cy_SysLib_Delay(1000);
    }
}

/* [] END OF FILE */
