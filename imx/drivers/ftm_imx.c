/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ftm_imx.h"


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*!
 * brief Ungates the FTM clock and configures the peripheral for basic operation.
 *
 * note This API should be called at the beginning of the application which is using the FTM driver.
 *      If the FTM instance has only TPM features, please use the TPM driver.
 *
 * param base   FTM peripheral base address
 * param config Pointer to the user configuration structure.
 *
 * return 0 indicates success; Else indicates failure.
 */
int32_t FTM_Init(FTM_Type *base, const ftm_config_t *config)
{
    assert(config);


    /* Configure the fault mode, enable FTM mode and disable write protection */
    base->CNTIN = 0x0000;
    base->MOD = 0xFFFF;
    base->MODE = FTM_MODE_FTMEN_MASK | FTM_MODE_WPDIS_MASK;
    base->QDCTRL = 0;
    /* Set the clock prescale factor */
    base->SC = FTM_SC_PS(config->prescale);

    base->SYNC = 0;

    /* Setup the counter operation */
    base->CONF = (FTM_CONF_BDMMODE(config->bdmMode) | (config->useGlobalTimeBase ? FTM_CONF_GTBEEN_MASK : 0));

    /* Initial state of channel output */
    base->OUTINIT = config->chnlInitState;

    /* Channel polarity */
    base->POL = config->chnlPolarity;

    /* Set the external trigger sources */
    base->EXTTRIG = config->extTriggers;
    base->COMBINE = 0;
    base->DEADTIME = 0;

    return 0;
}

/*!
 * brief Gates the FTM clock.
 *
 * param base FTM peripheral base address
 */
void FTM_Deinit(FTM_Type *base)
{
    /* Set clock source to none to disable counter */
    base->SC &= ~(FTM_SC_CLKS_MASK);
}

/*!
 * brief  Fills in the FTM configuration structure with the default settings.
 *
 * The default values are:
 * code
 *   config->prescale = kFTM_Prescale_Divide_1;
 *   config->bdmMode = kFTM_BdmMode_0;
 *   config->pwmSyncMode = kFTM_SoftwareTrigger;
 *   config->reloadPoints = 0;
 *   config->faultMode = kFTM_Fault_Disable;
 *   config->faultFilterValue = 0;
 *   config->deadTimePrescale = kFTM_Deadtime_Prescale_1;
 *   config->deadTimeValue =  0;
 *   config->extTriggers = 0;
 *   config->chnlInitState = 0;
 *   config->chnlPolarity = 0;
 *   config->useGlobalTimeBase = false;
 *   config->hwTriggerResetCount = false;
 *   config->swTriggerResetCount = true;
 * endcode
 * param config Pointer to the user configuration structure.
 */
void FTM_GetDefaultConfig(ftm_config_t *config)
{
    assert(config != NULL);

    /* Initializes the configure structure to zero. */
    (void)memset(config, 0, sizeof(*config));

    /* Divide FTM clock by 1 */
    config->prescale = kFTM_Prescale_Divide_1;
    /* FTM behavior in BDM mode */
    config->bdmMode = kFTM_BdmMode_0;
    /* Software trigger will be used to update registers */
    config->pwmSyncMode = (uint32_t)kFTM_SoftwareTrigger;
    /* No intermediate register load */
    config->reloadPoints = 0;
    /* Fault control disabled for all channels */
    config->faultMode = kFTM_Fault_Disable;
    /* Disable the fault filter */
    config->faultFilterValue = 0;
    /* Divide the system clock by 1 */
    config->deadTimePrescale = kFTM_Deadtime_Prescale_1;
    /* No counts are inserted */
    config->deadTimeValue = 0;
    /* No external trigger */
    config->extTriggers = 0;
    /* Initialization value is 0 for all channels */
    config->chnlInitState = 0;
    /* Active high polarity for all channels */
    config->chnlPolarity = 0;
    /* Use internal FTM counter as timebase */
    config->useGlobalTimeBase = false;
    /* Set hardware trigger activation counter sync to false */
    config->hwTriggerResetCount = false;
    /* Set software trigger activation counter sync to true */
    config->swTriggerResetCount = true;
}

/*!
 * brief Updates the edge level selection for a channel.
 *
 * param base       FTM peripheral base address
 * param chnlNumber The channel number
 * param captureMode Specifies which edge to capture
 */
void FTM_UpdateChnlEdgeLevelSelect(FTM_Type *base, ftm_chnl_t chnlNumber, ftm_input_capture_edge_t level)
{
    uint32_t reg = base->C[chnlNumber].CSC;

    /* Clear the field and write the new level value */
    reg &= ~(FTM_CSC_ELSA_MASK | FTM_CSC_ELSB_MASK);
    reg |= (level) & (FTM_CSC_ELSA_MASK | FTM_CSC_ELSB_MASK);

    base->C[chnlNumber].CSC = reg;
}

/*!
 * brief Enables capturing an input signal on the channel using the function parameters.
 *
 * When the edge specified in the captureMode argument occurs on the channel, the FTM counter is
 * captured into the CnV register. The user has to read the CnV register separately to get this
 * value. The filter function is disabled if the filterVal argument passed in is 0. The filter
 * function is available only for channels 0, 1, 2, 3.
 *
 * param base        FTM peripheral base address
 * param chnlNumber  The channel number
 * param captureMode Specifies which edge to capture
 * param filterValue Filter value, specify 0 to disable filter. Available only for channels 0-3.
 */
void FTM_SetupInputCapture(FTM_Type *base,
                           ftm_chnl_t chnlNumber,
                           ftm_input_capture_edge_t captureMode,
                           uint32_t filterValue)
{
    uint32_t reg;

    /* Clear the combine bit for the channel pair */
    base->COMBINE &=
        ~(1UL << (FTM_COMBINE_COMBINE0_SHIFT + (FTM_COMBINE_COMBINE1_SHIFT * ((uint32_t)chnlNumber >> 1))));
    /* Clear the dual edge capture mode because it's it's higher priority */
    base->COMBINE &=
        ~(1UL << (FTM_COMBINE_DECAPEN0_SHIFT + (FTM_COMBINE_COMBINE1_SHIFT * ((uint32_t)chnlNumber >> 1))));
#if !(defined(FSL_FEATURE_FTM_HAS_NO_QDCTRL) && FSL_FEATURE_FTM_HAS_NO_QDCTRL)
    /* Clear the quadrature decoder mode beacause it's higher priority */
    base->QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;
#endif

    reg = base->C[chnlNumber].CSC;
    reg &= ~(FTM_CSC_MSA_MASK | FTM_CSC_MSB_MASK | FTM_CSC_ELSA_MASK | FTM_CSC_ELSB_MASK);
    reg |= (uint32_t)captureMode;

    /* Set the requested input capture mode */
    base->C[chnlNumber].CSC = reg;
    /* Input filter available only for channels 0, 1, 2, 3 */
    if (chnlNumber < kFTM_Chnl_4)
    {
        reg = base->FILTER;
        reg &= ~((uint32_t)FTM_FILTER_CH0FVAL_MASK << (FTM_FILTER_CH1FVAL_SHIFT * (uint32_t)chnlNumber));
        reg |= (filterValue << (FTM_FILTER_CH1FVAL_SHIFT * (uint32_t)chnlNumber));
        base->FILTER = reg;
    }
}

/*!
 * brief Configures the FTM to generate timed pulses.
 *
 * When the FTM counter matches the value of compareVal argument (this is written into CnV reg),
 * the channel output is changed based on what is specified in the compareMode argument.
 *
 * param base         FTM peripheral base address
 * param chnlNumber   The channel number
 * param compareMode  Action to take on the channel output when the compare condition is met
 * param compareValue Value to be programmed in the CnV register.
 */
void FTM_SetupOutputCompare(FTM_Type *base,
                            ftm_chnl_t chnlNumber,
                            ftm_output_compare_mode_t compareMode,
                            uint32_t compareValue)
{
    uint32_t reg;

    reg = base->C[chnlNumber].CSC;
    reg &= ~(FTM_CSC_MSA_MASK | FTM_CSC_MSB_MASK | FTM_CSC_ELSA_MASK | FTM_CSC_ELSB_MASK);
    reg |= (uint32_t)compareMode;
    /* Setup the channel output behaviour when a match occurs with the compare value */
    base->C[chnlNumber].CSC = reg;

    /* Set output on match to the requested level */
    base->C[chnlNumber].CV = compareValue & 0xFFFF;
}

/*!
 * brief Enables the selected FTM interrupts.
 *
 * param base FTM peripheral base address
 * param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::ftm_interrupt_enable_t
 */
void FTM_EnableInterrupts(FTM_Type *base, uint32_t mask)
{
    uint32_t chnlInts  = (mask & 0xFFU);
    uint8_t chnlNumber = 0;

    /* Enable the timer overflow interrupt */
    if ((mask & (uint32_t)kFTM_TimeOverflowInterruptEnable) != 0U)
    {
        base->SC |= FTM_SC_TOIE_MASK;
    }

    /* Enable the channel interrupts */
    while (chnlInts != 0U)
    {
        if ((chnlInts & 0x1U) != 0U)
        {
            base->C[chnlNumber].CSC |= FTM_CSC_CHIE_MASK;
        }
        chnlNumber++;
        chnlInts = chnlInts >> 1U;
    }
}

/*!
 * brief Disables the selected FTM interrupts.
 *
 * param base FTM peripheral base address
 * param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::ftm_interrupt_enable_t
 */
void FTM_DisableInterrupts(FTM_Type *base, uint32_t mask)
{
    uint32_t chnlInts  = (mask & 0xFFU);
    uint8_t chnlNumber = 0;

    /* Disable the timer overflow interrupt */
    if ((mask & (uint32_t)kFTM_TimeOverflowInterruptEnable) != 0U)
    {
        base->SC &= ~FTM_SC_TOIE_MASK;
    }

    /* Disable the channel interrupts */
    while (chnlInts != 0U)
    {
        if ((chnlInts & 0x01U) != 0U)
        {
            base->C[chnlNumber].CSC &= ~FTM_CSC_CHIE_MASK;
        }
        chnlNumber++;
        chnlInts = chnlInts >> 1U;
    }
}

/*!
 * brief Gets the enabled FTM interrupts.
 *
 * param base FTM peripheral base address
 *
 * return The enabled interrupts. This is the logical OR of members of the
 *         enumeration ::ftm_interrupt_enable_t
 */
uint32_t FTM_GetEnabledInterrupts(FTM_Type *base)
{
    uint32_t enabledInterrupts = 0;
    int8_t chnlCount           = 8;

    /* The CHANNEL_COUNT macro returns -1 if it cannot match the FTM instance */
    assert(chnlCount != -1);

    /* Check if timer overflow interrupt is enabled */
    if ((base->SC & FTM_SC_TOIE_MASK) != 0U)
    {
        enabledInterrupts |= (uint32_t)kFTM_TimeOverflowInterruptEnable;
    }

    /* Check if the channel interrupts are enabled */
    while (chnlCount > 0)
    {
        chnlCount--;
        if ((base->C[chnlCount].CSC & FTM_CSC_CHIE_MASK) != 0x00U)
        {
            enabledInterrupts |= (1UL << (uint32_t)chnlCount);
        }
    }

    return enabledInterrupts;
}

/*!
 * brief Gets the FTM status flags.
 *
 * param base FTM peripheral base address
 *
 * return The status flags. This is the logical OR of members of the
 *         enumeration ::ftm_status_flags_t
 */
uint32_t FTM_GetStatusFlags(FTM_Type *base)
{
    uint32_t statusFlags = 0;

    /* Check the timer flag */
    if ((base->SC & FTM_SC_TOF_MASK) != 0U)
    {
        statusFlags |= (uint32_t)kFTM_TimeOverflowFlag;
    }

    /* Check channel trigger flag */
    if ((base->EXTTRIG & FTM_EXTTRIG_TRIGF_MASK) != 0U)
    {
        statusFlags |= (uint32_t)kFTM_ChnlTriggerFlag;
    }

    /* Lower 8 bits contain the channel status flags */
    statusFlags |= (base->STATUS & 0xFFU);

    return statusFlags;
}

/*!
 * brief Clears the FTM status flags.
 *
 * param base FTM peripheral base address
 * param mask The status flags to clear. This is a logical OR of members of the
 *             enumeration ::ftm_status_flags_t
 */
void FTM_ClearStatusFlags(FTM_Type *base, uint32_t mask)
{
    /* Clear the timer overflow flag by writing a 0 to the bit while it is set */
    if ((mask & (uint32_t)kFTM_TimeOverflowFlag) != 0U)
    {
        base->SC &= ~FTM_SC_TOF_MASK;
    }

    /* Clear the channel status flags by writing a 0 to the bit */
    base->STATUS &= ~(mask & 0xFFU);
}
