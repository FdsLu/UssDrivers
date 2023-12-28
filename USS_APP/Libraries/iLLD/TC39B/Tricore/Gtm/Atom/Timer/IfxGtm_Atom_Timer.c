/**
 * \file IfxGtm_Atom_Timer.c
 * \brief GTM TIMER details
 *
 * \version iLLD_1_0_1_16_0_1
 * \copyright Copyright (c) 2022 Infineon Technologies AG. All rights reserved.
 *
 *
 *
 *                                 IMPORTANT NOTICE
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees. If and as long as no such terms
 * of use are agreed, use of this file is subject to following:
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer, must
 * be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are
 * solely in the form of machine-executable object code generated by a source
 * language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "IfxGtm_Atom_Timer.h"
#include "_Utilities/Ifx_Assert.h"
#include "IfxGtm_bf.h"
#include "stddef.h"
#include "string.h"

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

boolean IfxGtm_Atom_Timer_acknowledgeTimerIrq(IfxGtm_Atom_Timer *driver)
{
    boolean event;
    event = IfxGtm_Atom_Ch_isZeroNotification(driver->atom, driver->timerChannel);

    if (event)
    {
        IfxGtm_Atom_Ch_clearZeroNotification(driver->atom, driver->timerChannel);
    }
    else
    {}

    return event;
}


boolean IfxGtm_Atom_Timer_acknowledgeTriggerIrq(IfxGtm_Atom_Timer *driver)
{
    boolean event;
    event = IfxGtm_Atom_Ch_isOneNotification(driver->atom, driver->triggerChannel);

    if (event)
    {
        IfxGtm_Atom_Ch_clearOneNotification(driver->atom, driver->triggerChannel);
    }
    else
    {}

    return event;
}


void IfxGtm_Atom_Timer_addToChannelMask(IfxGtm_Atom_Timer *driver, IfxGtm_Atom_Ch channel)
{
    driver->channelsMask    |= 1 << channel;
    driver->agcDisableUpdate = IfxGtm_Atom_Agc_buildFeature(0, driver->channelsMask, IFX_GTM_ATOM_AGC_GLB_CTRL_UPEN_CTRL0_OFF);
    driver->agcApplyUpdate   = IfxGtm_Atom_Agc_buildFeature(driver->channelsMask, 0, IFX_GTM_ATOM_AGC_GLB_CTRL_UPEN_CTRL0_OFF);
}


void IfxGtm_Atom_Timer_applyUpdate(IfxGtm_Atom_Timer *driver)
{
    IfxGtm_Atom_Agc_setChannelsUpdate(driver->agc, driver->agcApplyUpdate);
}


void IfxGtm_Atom_Timer_disableUpdate(IfxGtm_Atom_Timer *driver)
{
    IfxGtm_Atom_Agc_setChannelsUpdate(driver->agc, driver->agcDisableUpdate);
}


float32 IfxGtm_Atom_Timer_getFrequency(IfxGtm_Atom_Timer *driver)
{
    return 1.0f / IfxStdIf_Timer_tickToS(driver->base.clockFreq, driver->base.period);
}


float32 IfxGtm_Atom_Timer_getInputFrequency(IfxGtm_Atom_Timer *driver)
{
    return driver->base.clockFreq;
}


Ifx_TimerValue IfxGtm_Atom_Timer_getOffset(IfxGtm_Atom_Timer *driver)
{
    return driver->offset;
}


Ifx_TimerValue IfxGtm_Atom_Timer_getPeriod(IfxGtm_Atom_Timer *driver)
{
    return driver->base.period;
}


volatile uint32 *IfxGtm_Atom_Timer_getPointer(IfxGtm_Atom_Timer *driver)
{
    return IfxGtm_Atom_Ch_getTimerPointer(driver->atom, driver->timerChannel);
}


float32 IfxGtm_Atom_Timer_getResolution(IfxGtm_Atom_Timer *driver)
{
    return 1.0f / driver->base.clockFreq;
}


Ifx_TimerValue IfxGtm_Atom_Timer_getTrigger(IfxGtm_Atom_Timer *driver)
{
    return IfxGtm_Atom_Ch_getCompareOne(driver->atom, driver->triggerChannel) - 1;
}


volatile uint32 *IfxGtm_Atom_Timer_getTriggerPointer(IfxGtm_Atom_Timer *driver)
{
    return IfxGtm_Atom_Ch_getCompareOnePointer(driver->atom, driver->triggerChannel);
}


boolean IfxGtm_Atom_Timer_init(IfxGtm_Atom_Timer *driver, const IfxGtm_Atom_Timer_Config *config)
{
    boolean                 result = TRUE;
    IfxGtm_Atom_Timer_Base *base   = &driver->base;

    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config->base.countDir == IfxStdIf_Timer_CountDir_up); /* only this mode is supported */

    driver->gtm          = config->gtm;
    driver->atomIndex    = config->atom;
    driver->atom         = &config->gtm->ATOM[config->atom];
    driver->timerChannel = config->timerChannel;

/*
 *  IfxGtm_Dtm dtmIndex = IfxGtm_Dtm_getAtomChannelDtmIndex(config->atom, config->timerChannel);
 *  boolean dtmAvailable = FALSE;
 *  if (dtmIndex != IfxGtm_Dtm_none)
 *  {
 *      driver->dtm         = &config->gtm->CDTM.DTM[dtmIndex];
 *      driver->dtmChannel  = (IfxGtm_Dtm_Ch) (config->timerChannel % 4);
 *  dtmAvailable = TRUE;
 *  }
 *  else
 *  {
 *      dtmAvailable = FALSE;
 *  }
 */

    base->triggerEnabled = config->base.trigger.enabled;

    if (base->triggerEnabled)
    {
        if (config->triggerOut != NULL_PTR)
        {
            driver->triggerChannel = config->triggerOut->channel;
        }
        else
        {
            result = FALSE;
            IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, result); /* triggerOut is required */
        }
    }
    else
    {
        driver->triggerChannel = driver->timerChannel; // Set to timer channel to disable its use
    }

    driver->agc              = (Ifx_GTM_ATOM_AGC *)&driver->atom->AGC.GLB_CTRL;

    driver->channelsMask     = 0;
    driver->agcApplyUpdate   = 0;
    driver->agcDisableUpdate = 0;

    /* Initialize the timer part */
    IfxGtm_Atom_Ch_configurePwmMode(driver->atom, driver->timerChannel, config->clock,
        (Ifx_ActiveState)config->base.trigger.risingEdgeAtPeriod, IfxGtm_Atom_Ch_ResetEvent_onCm0,
        IfxGtm_Atom_Ch_OutputTrigger_generate);

    IfxGtm_Atom_Timer_updateInputFrequency(driver);

    if ((config->base.minResolution > 0) && ((1.0f / base->clockFreq) > config->base.minResolution))
    {
        result = FALSE;
        IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);
    }
    else
    {}

    IfxGtm_Atom_Timer_setFrequency(driver, config->base.frequency);
    driver->offset = IfxStdIf_Timer_sToTick(driver->base.clockFreq, 1.0f / config->base.frequency * config->base.startOffset);

    IfxGtm_Atom_Ch_setCounterValue(driver->atom, driver->timerChannel, driver->offset);

    /* Initialize the trigger part */
    IfxGtm_Atom_Timer_addToChannelMask(driver, driver->timerChannel);

    if (base->triggerEnabled)
    {
        IfxGtm_Atom_Ch triggerChannel     = driver->triggerChannel;
        uint16         triggerChannelMask = 1 << triggerChannel;

        IfxGtm_Atom_Ch_setSignalLevel(driver->atom, triggerChannel, config->base.trigger.risingEdgeAtPeriod ? Ifx_ActiveState_high : Ifx_ActiveState_low);

        IfxGtm_Atom_Ch_setCounterValue(driver->atom, triggerChannel, driver->offset);

        if (triggerChannel != driver->timerChannel)
        {
            IfxGtm_Atom_Ch_configurePwmMode(driver->atom, triggerChannel, config->clock,
                (Ifx_ActiveState)config->base.trigger.risingEdgeAtPeriod, IfxGtm_Atom_Ch_ResetEvent_onTrigger,
                IfxGtm_Atom_Ch_OutputTrigger_forward);
            IfxGtm_Atom_Agc_enableChannels(driver->agc, triggerChannelMask, 0, FALSE);
            IfxGtm_Atom_Timer_addToChannelMask(driver, driver->triggerChannel);
        }
        else
        {}

        /* Signal must go out of the GTM even if the port outpout is not enabled */
        IfxGtm_Atom_Agc_enableChannelsOutput(driver->agc, triggerChannelMask, 0, FALSE);

        if ((config->base.trigger.outputEnabled) && (config->initPins == TRUE))
        {
            /* Initialize the port */
            IfxGtm_PinMap_setAtomTout(config->triggerOut, config->base.trigger.outputMode, config->base.trigger.outputDriver);
        }
        else
        {}

        IfxGtm_Atom_Timer_setTrigger(driver, config->base.trigger.triggerPoint);
    }
    else
    {}

    /* Interrupt configuration */
    {
        volatile Ifx_SRC_SRCR *src;
        boolean                timerHasIrq   = config->base.isrPriority > 0;
        boolean                triggerHasIrq = (config->base.trigger.isrPriority > 0) && base->triggerEnabled;

        if (driver->triggerChannel == driver->timerChannel)
        {
            IfxGtm_Atom_Ch_setNotification(driver->atom, driver->timerChannel, timerHasIrq ? config->irqModeTimer : config->irqModeTrigger, timerHasIrq, triggerHasIrq);
            src = IfxGtm_Atom_Ch_getSrcPointer(driver->gtm, config->atom, driver->timerChannel);
            IfxSrc_init(src, timerHasIrq ? config->base.isrProvider : config->base.trigger.isrProvider, timerHasIrq ? config->base.isrPriority : config->base.trigger.isrPriority);
            IfxSrc_enable(src);
        }
        else
        {
            IfxGtm_IrqMode irqMode = IfxGtm_IrqMode_pulseNotify;

            if (timerHasIrq)
            {
                IfxGtm_Atom_Ch_setNotification(driver->atom, driver->timerChannel, irqMode, TRUE, FALSE);
                src = IfxGtm_Atom_Ch_getSrcPointer(driver->gtm, config->atom, driver->timerChannel);
                IfxSrc_init(src, config->base.isrProvider, config->base.isrPriority);
                IfxSrc_enable(src);
            }

            if (triggerHasIrq)
            {
                IfxGtm_Atom_Ch_setNotification(driver->atom, driver->triggerChannel, irqMode, FALSE, TRUE);
                src = IfxGtm_Atom_Ch_getSrcPointer(driver->gtm, config->atom, driver->triggerChannel);
                IfxSrc_init(src, config->base.trigger.isrProvider, config->base.trigger.isrPriority);
                IfxSrc_enable(src);
            }
        }
    }

    /* Transfer the shadow registers */
    IfxGtm_Atom_Agc_setChannelsForceUpdate(driver->agc, driver->channelsMask, 0, 0, 0);
    IfxGtm_Atom_Agc_trigger(driver->agc);
    IfxGtm_Atom_Agc_setChannelsForceUpdate(driver->agc, 0, driver->channelsMask, 0, 0);

/*
 *  if (dtmAvailable)
 *  {
 *  // bypassing the DTM functionality
 *  IfxGtm_Dtm_setClockSource(driver->dtm, config->dtmClockSource);
 *  IfxGtm_Dtm_setOutput0DeadTimePath(driver->dtm, driver->dtmChannel, IfxGtm_Dtm_DeadTimePath_feedThrough);
 *
 *  IfxGtm_Dtm_setOutput1Select(driver->dtm, driver->dtmChannel, IfxGtm_Dtm_Output1Select_specialFunction);
 *  IfxGtm_Dtm_setOutput1Function(driver->dtm, driver->dtmChannel, IfxGtm_Dtm_Output1Function_dtmInputSignal);
 *  IfxGtm_Dtm_setOutput1DeadTimePath(driver->dtm, driver->dtmChannel, IfxGtm_Dtm_DeadTimePath_enable);
 *  }
 */

    return result;
}


void IfxGtm_Atom_Timer_initConfig(IfxGtm_Atom_Timer_Config *config, Ifx_GTM *gtm)
{
    IfxStdIf_Timer_initConfig(&config->base);
    config->gtm            = gtm;
    config->atom           = IfxGtm_Atom_0;
    config->timerChannel   = IfxGtm_Atom_Ch_0;
    config->triggerOut     = NULL_PTR;
    config->clock          = IfxGtm_Cmu_Clk_0;
    config->base.countDir  = IfxStdIf_Timer_CountDir_up;
    config->irqModeTimer   = IfxGtm_IrqMode_level;
    config->irqModeTrigger = IfxGtm_IrqMode_level;
    config->initPins       = TRUE;

//    config->dtmClockSource = IfxGtm_Dtm_ClockSource_cmuClock1;
}


void IfxGtm_Atom_Timer_run(IfxGtm_Atom_Timer *driver)
{
    IfxGtm_Atom_Agc_enableChannels(driver->agc, driver->channelsMask, 0, TRUE);
}


boolean IfxGtm_Atom_Timer_setFrequency(IfxGtm_Atom_Timer *driver, float32 frequency)
{
    Ifx_TimerValue period = IfxStdIf_Timer_sToTick(driver->base.clockFreq, 1.0f / frequency);

    return IfxGtm_Atom_Timer_setPeriod(driver, period);
}


boolean IfxGtm_Atom_Timer_setPeriod(IfxGtm_Atom_Timer *driver, Ifx_TimerValue period)
{
    driver->base.period = period;
    IfxGtm_Atom_Ch_setCompareZeroShadow(driver->atom, driver->timerChannel, period);

    if (driver->triggerChannel != driver->timerChannel)
    {
        IfxGtm_Atom_Ch_setCompareZeroShadow(driver->atom, driver->triggerChannel, period);
    }

    return TRUE;
}


void IfxGtm_Atom_Timer_setSingleMode(IfxGtm_Atom_Timer *driver, boolean enabled)
{
    IfxGtm_Atom_Ch_setOneShotMode(driver->atom, driver->timerChannel, enabled);
}


void IfxGtm_Atom_Timer_setTrigger(IfxGtm_Atom_Timer *driver, Ifx_TimerValue triggerPoint)
{
    IfxGtm_Atom_Ch_setCompareOneShadow(driver->atom, driver->triggerChannel, triggerPoint + 1);
}


boolean IfxGtm_Atom_Timer_stdIfTimerInit(IfxStdIf_Timer *stdif, IfxGtm_Atom_Timer *driver)
{
    /* Ensure the stdif is reset to zeros */
    memset(stdif, 0, sizeof(IfxStdIf_Timer));

    /* *INDENT-OFF* Note: this file was indented manually by the author. */
    /* Set the API link */
    stdif->driver               = driver;
    stdif->getFrequency         =(IfxStdIf_Timer_GetFrequency        )&IfxGtm_Atom_Timer_getFrequency;
    stdif->getPeriod            =(IfxStdIf_Timer_GetPeriod           )&IfxGtm_Atom_Timer_getPeriod;
    stdif->getResolution        =(IfxStdIf_Timer_GetResolution       )&IfxGtm_Atom_Timer_getResolution;
    stdif->getTrigger           =(IfxStdIf_Timer_GetTrigger          )&IfxGtm_Atom_Timer_getTrigger;
    stdif->setFrequency         =(IfxStdIf_Timer_SetFrequency        )&IfxGtm_Atom_Timer_setFrequency;
    stdif->updateInputFrequency =(IfxStdIf_Timer_UpdateInputFrequency)&IfxGtm_Atom_Timer_updateInputFrequency;
    stdif->applyUpdate          =(IfxStdIf_Timer_ApplyUpdate         )&IfxGtm_Atom_Timer_applyUpdate;
    stdif->disableUpdate        =(IfxStdIf_Timer_DisableUpdate       )&IfxGtm_Atom_Timer_disableUpdate;
    stdif->getInputFrequency    =(IfxStdIf_Timer_GetInputFrequency   )&IfxGtm_Atom_Timer_getInputFrequency;
    stdif->run                  =(IfxStdIf_Timer_Run                 )&IfxGtm_Atom_Timer_run;
    stdif->setPeriod            =(IfxStdIf_Timer_SetPeriod           )&IfxGtm_Atom_Timer_setPeriod;
    stdif->setSingleMode        =(IfxStdIf_Timer_SetSingleMode       )&IfxGtm_Atom_Timer_setSingleMode;
    stdif->setTrigger           =(IfxStdIf_Timer_SetTrigger          )&IfxGtm_Atom_Timer_setTrigger;
    stdif->stop                 =(IfxStdIf_Timer_Stop                )&IfxGtm_Atom_Timer_stop;
    stdif->ackTimerIrq          =(IfxStdIf_Timer_AckTimerIrq         )&IfxGtm_Atom_Timer_acknowledgeTimerIrq;
    stdif->ackTriggerIrq        =(IfxStdIf_Timer_AckTriggerIrq       )&IfxGtm_Atom_Timer_acknowledgeTriggerIrq;
    /* *INDENT-ON* */

    return TRUE;
}


void IfxGtm_Atom_Timer_stop(IfxGtm_Atom_Timer *driver)
{
    IfxGtm_Atom_Agc_enableChannels(driver->agc, 0, driver->channelsMask, TRUE);
}


void IfxGtm_Atom_Timer_updateInputFrequency(IfxGtm_Atom_Timer *driver)
{
    driver->base.clockFreq = IfxGtm_Atom_Ch_getClockFrequency(driver->gtm, driver->atom, driver->timerChannel);
}
