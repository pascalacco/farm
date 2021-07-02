/**
  ******************************************************************************
  * @file    stm32l475e_iot01_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio driver for the B-L475E-IOT01
  *          board.
  *          J'ai ajouté une sortie mono DAC (Sortie D13 arduino) 
  @verbatim
  How To use this driver:
  -----------------------
     + Call the function BSP_AUDIO_OUT_Init() for AUDIO OUT initialization: 
        Instance : Select the output instance. Can only be 0 (DAC).
        AudioInit: Audio Out structure to select the following parameters.
                   - Device: Select the output device (headphone, speaker, ..).
                   - SampleRate: Select the output sample rate (8Khz .. 96Khz).
                   - BitsPerSample: Select the output resolution (16 or 32bits per sample).
                   - ChannelsNbr: Select the output channels number(1 for mono, 2 for stereo).
                   - Volume: Select the output volume(0% .. 100%).
                                     
      This function configures all the hardware required for the audio application (codec, I2C, SAI, 
      GPIOs, DMA and interrupt if needed). This function returns BSP_ERROR_NONE if configuration is OK.
 
      User can update the DAC or the clock configurations by overriding the weak MX functions MX_SAI1_Init()
      and MX_DAC_ClockConfig().
   
   + Call the function BSP_EVAL_AUDIO_OUT_Play() to play audio stream:
        Instance : Select the output instance. Can only be 0 (DAC).
        pBuf: pointer to the audio data file address.
        NbrOfBytes: Total size of the buffer to be sent in Bytes.

   + Call the function BSP_AUDIO_OUT_Pause() to pause playing.
   + Call the function BSP_AUDIO_OUT_Resume() to resume playing.
       Note. After calling BSP_AUDIO_OUT_Pause() function for pause, only BSP_AUDIO_OUT_Resume() should be called
          for resume (it is not allowed to call BSP_AUDIO_OUT_Play() in this case).
       Note. This function should be called only when the audio file is played or paused (not stopped).
   + Call the function BSP_AUDIO_OUT_Stop() to stop playing.
   + Call the function BSP_AUDIO_OUT_Mute() to mute the player.
   + Call the function BSP_AUDIO_OUT_UnMute() to unmute the player.
   + Call the function BSP_AUDIO_OUT_IsMute() to get the mute state(BSP_AUDIO_MUTE_ENABLED or BSP_AUDIO_MUTE_DISABLED).
   + Call the function BSP_AUDIO_OUT_SetDevice() to update the AUDIO OUT device.
   + Call the function BSP_AUDIO_OUT_GetDevice() to get the AUDIO OUT device.
   + Call the function BSP_AUDIO_OUT_SetSampleRate() to update the AUDIO OUT sample rate.
   + Call the function BSP_AUDIO_OUT_GetSampleRate() to get the AUDIO OUT sample rate.
   + Call the function BSP_AUDIO_OUT_SetBitsPerSample() to update the AUDIO OUT resolution.
   + Call the function BSP_AUDIO_OUT_GetBitPerSample() to get the AUDIO OUT resolution.
   + Call the function BSP_AUDIO_OUT_SetChannelsNbr() to update the AUDIO OUT number of channels.
   + Call the function BSP_AUDIO_OUT_GetChannelsNbr() to get the AUDIO OUT number of channels.
   + Call the function BSP_AUDIO_OUT_SetVolume() to update the AUDIO OUT volume.
   + Call the function BSP_AUDIO_OUT_GetVolume() to get the AUDIO OUT volume.
   + Call the function BSP_AUDIO_OUT_GetState() to get the AUDIO OUT state.

   + BSP_AUDIO_OUT_SetDevice(), BSP_AUDIO_OUT_SetSampleRate(), BSP_AUDIO_OUT_SetBitsPerSample() and
     BSP_AUDIO_OUT_SetChannelsNbr() cannot be called while the state is AUDIO_OUT_STATE_PLAYING.
   + For each mode, you may need to implement the relative callback functions into your code.
      The Callback functions are named AUDIO_OUT_XXX_CallBack() and only their prototypes are declared in 
      the stm32l475e_iot01_audio.h file. 

   + Call the function BSP_AUDIO_IN_Init() for AUDIO IN initialization:
        Instance : Select the input instance. Can be only 0 (DFSDM).
        AudioInit: Audio In structure to select the following parameters.
                   - Device: Select the input device (digital mic1, mic2, mic1 & mic2).
                   - SampleRate: Select the input sample rate (8Khz .. 96Khz).
                   - BitsPerSample: Select the input resolution (16 or 32bits per sample).
                   - ChannelsNbr: Select the input channels number(1 for mono, 2 for stereo).
                   - Volume: Select the input volume(0% .. 100%).

      This function configures all the hardware required for the audio application (DFSDM
      GPIOs, DMA and interrupt if needed). This function returns BSP_ERROR_NONE if configuration is OK.

      User can update the DFSDM or the clock configurations by overriding the weak MX functions MX_DFSDM1_Init()
      and MX_DFDSM1_ClockConfig()
      User can override the default MSP configuration and register his own MSP callbacks (defined at application level)
      by calling BSP_AUDIO_IN_RegisterMspCallbacks() function.
      User can restore the default MSP configuration by calling BSP_AUDIO_IN_RegisterDefaultMspCallbacks().
      To use these two functions, user have to enable USE_HAL_DFSDM_REGISTER_CALLBACKS within stm32l4xx_hal_conf.h file.

   + Call the function BSP_EVAL_AUDIO_IN_Record() to record audio stream. The recorded data are stored to user buffer in raw
        (L, R, L, R ...).
        Instance : Select the input instance. Can be only 0 (DFSDM).
        pBuf: pointer to user buffer.
        NbrOfBytes: Total size of the buffer to be sent in Bytes.

   + Call the function BSP_AUDIO_IN_Pause() to pause recording.
   + Call the function BSP_AUDIO_IN_Resume() to resume recording.
   + Call the function BSP_AUDIO_IN_Stop() to stop recording.
   + Call the function BSP_AUDIO_IN_SetDevice() to update the AUDIO IN device.
   + Call the function BSP_AUDIO_IN_GetDevice() to get the AUDIO IN device.
   + Call the function BSP_AUDIO_IN_SetSampleRate() to update the AUDIO IN sample rate.
   + Call the function BSP_AUDIO_IN_GetSampleRate() to get the AUDIO IN sample rate.
   + Call the function BSP_AUDIO_IN_SetBitPerSample() to update the AUDIO IN resolution.
   + Call the function BSP_AUDIO_IN_GetBitPerSample() to get the AUDIO IN resolution.
   + Call the function BSP_AUDIO_IN_SetChannelsNbr() to update the AUDIO IN number of channels.
   + Call the function BSP_AUDIO_IN_GetChannelsNbr() to get the AUDIO IN number of channels.
   + Call the function BSP_AUDIO_IN_SetVolume() to update the AUDIO IN volume.
   + Call the function BSP_AUDIO_IN_GetVolume() to get the AUDIO IN volume.
   + Call the function BSP_AUDIO_IN_GetState() to get the AUDIO IN state.

   + For each mode, you may need to implement the relative callback functions into your code.
      The Callback functions are named AUDIO_IN_XXX_CallBack() and only their prototypes are declared in
      the stm32l475e_iot01_audio.h file (refer to the example for more details on the callbacks implementations).

   + The driver API and the callback functions are at the end of the stm32l475e_iot01_audio.h file.

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l475e_iot01_audio.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L475E_IOT01
  * @{
  */

/** @defgroup STM32L475E_IOT01_AUDIO STM32L475E_IOT01 AUDIO
  * @{
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Private_Macros STM32L475E_IOT01 AUDIO_IN Private Macros
  * @{
  */
#define DFSDM_OVER_SAMPLING(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (256U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (256U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (32U) : (16U)

#define DFSDM_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (32U) : (32U)

#define DFSDM_FILTER_ORDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (DFSDM_FILTER_SINC3_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (DFSDM_FILTER_SINC3_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (DFSDM_FILTER_SINC3_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (DFSDM_FILTER_SINC3_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (DFSDM_FILTER_SINC3_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (DFSDM_FILTER_SINC4_ORDER) : (DFSDM_FILTER_SINC5_ORDER)

#define DFSDM_MIC_BIT_SHIFT(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (5U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (6U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (3U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (3U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (6U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (0U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (2U) : (4U)

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Exported_Variables STM32L475E_IOT01 AUDIO_IN_OUT Exported Variables
  * @{
  */
/* Audio in and out context */
AUDIO_OUT_Ctx_t Audio_Out_Ctx[AUDIO_OUT_INSTANCES_NBR] = {{AUDIO_OUT_HEADPHONE,
                                                           AUDIO_FREQUENCY_8K,
                                                           AUDIO_RESOLUTION_16b,
                                                           50U,
                                                           1U,
                                                           AUDIO_MUTE_DISABLED,
                                                           AUDIO_OUT_STATE_RESET}};
AUDIO_IN_Ctx_t  Audio_In_Ctx[AUDIO_IN_INSTANCES_NBR] = {{AUDIO_IN_DIGITAL_MIC,
                                                         AUDIO_FREQUENCY_8K,
                                                         AUDIO_RESOLUTION_16b,
                                                         2U,
                                                         NULL,
                                                         0U,
                                                         50U,
                                                         AUDIO_IN_STATE_RESET}};

/* Audio in DFSDM handles */
DFSDM_Channel_HandleTypeDef haudio_in_dfsdm_channel[2] = {{0}, {0}};
DFSDM_Filter_HandleTypeDef  haudio_in_dfsdm_filter[2] = {{0}, {0}};

/* Audio in DFSDM internal buffers and global varibales */
int32_t  Audio_DigMic1RecBuff[BSP_AUDIO_IN_DEFAULT_BUFFER_SIZE];
int32_t  Audio_DigMic2RecBuff[BSP_AUDIO_IN_DEFAULT_BUFFER_SIZE];
uint32_t Audio_DmaDigMic1RecHalfBuffCplt;
uint32_t Audio_DmaDigMic1RecBuffCplt;
uint32_t Audio_DmaDigMic2RecHalfBuffCplt;
uint32_t Audio_DmaDigMic2RecBuffCplt;


/* Audio driver */
AUDIO_Drv_t *Audio_Drv = NULL;
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Private_Variables STM32L475E_IOT01 AUDIO_IN_OUT Private Variables
  * @{
  */
/* Audio in DMA handles used by DFSDM */
static DMA_HandleTypeDef hDmaDfsdm[2];

#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
static uint32_t AudioIn_IsMspCbValid[AUDIO_IN_INSTANCES_NBR] = {0};
#endif
#if (USE_HAL_SAI_REGISTER_CALLBACKS == 1)
static uint32_t AudioOut_IsMspCbValid[AUDIO_OUT_INSTANCES_NBR] = {0};
#endif

// DAC et TIMx handlers

static TIM_HandleTypeDef htim;
static DAC_HandleTypeDef hdac;


/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Private_Function_Prototypes STM32L475E_IOT01 AUDIO_IN_OUT Private Function Prototypes
  * @{
  */
static int32_t DFSDM_DeInit(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel);
static void    DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
static void    DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
static void    DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
static void    DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
static void    DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
static void    DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
static void    DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */


HAL_StatusTypeDef HAL_DAC_Transmit_DMA(DAC_HandleTypeDef* hdac, uint16_t* pData, uint16_t NbrOfDMAData);
HAL_StatusTypeDef HAL_DAC_DMAPause(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_DMAResume(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_DMAStop(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_Mute(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_UnMute(DAC_HandleTypeDef* hdac);


/**
  * @}
  */

/** @addtogroup STM32L475E_IOT01_AUDIO_OUT_Exported_Functions
  * @{
  */
/**
  * @brief  Initialize the audio out peripherals.
  * @param  Instance Audio out instance.
  * @param  AudioInit Audio out init structure.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_Init(uint32_t Instance, BSP_AUDIO_Init_t *AudioInit)
{
	int32_t status = BSP_ERROR_NONE;

	if (Instance >= AUDIO_OUT_INSTANCES_NBR)
	{
		status = BSP_ERROR_WRONG_PARAM;
	}
	else if ((Instance == 0U) && ((AudioInit->BitsPerSample == AUDIO_RESOLUTION_32b) || (AudioInit->BitsPerSample == AUDIO_RESOLUTION_8b)))
	{
		status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
	}
	else if ((Instance == 0U) && (Audio_In_Ctx[0].State != AUDIO_IN_STATE_RESET) &&
			((Audio_In_Ctx[0].SampleRate != AudioInit->SampleRate) || (Audio_In_Ctx[0].BitsPerSample != AudioInit->BitsPerSample)))
	{
		status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
	}
	else if ((Instance == 0U) && (Audio_In_Ctx[1].State != AUDIO_IN_STATE_RESET) &&
			(Audio_In_Ctx[1].SampleRate != AudioInit->SampleRate))
	{
		status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
	}
	else if (Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_RESET)
	{
		status = BSP_ERROR_BUSY;
	}
	else
	{

		if (status == BSP_ERROR_NONE)
		{
			/* Fill audio out context structure */
			Audio_Out_Ctx[Instance].Device         = AudioInit->Device;
			Audio_Out_Ctx[Instance].SampleRate     = AudioInit->SampleRate;
			Audio_Out_Ctx[Instance].BitsPerSample  = AudioInit->BitsPerSample;
			Audio_Out_Ctx[Instance].ChannelsNbr    = AudioInit->ChannelsNbr;
			Audio_Out_Ctx[Instance].Volume         = AudioInit->Volume;
		}

		if (status == BSP_ERROR_NONE)
		{
			if (MX_DAC1_ClockConfig(&hdac, AudioInit->SampleRate) != HAL_OK)
			{
				status = BSP_ERROR_CLOCK_FAILURE;
			}
		}

		if (status == BSP_ERROR_NONE)
		{
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 0)
			//TODO DAC_MspInit(&hdac);
#else
			/* Register the SAI MSP Callbacks */
			if (AudioOut_IsMspCbValid[Instance] == 0U)
			{
				if (BSP_AUDIO_OUT_RegisterDefaultMspCallbacks(Instance) != BSP_ERROR_NONE)
				{
					status = BSP_ERROR_PERIPH_FAILURE;
				}
			}
#endif /* (USE_HAL_SAI_REGISTER_CALLBACKS == 0) */
		}

		if (status == BSP_ERROR_NONE)
		{

			/* DAC peripheral initialization */
			if (MX_DAC1_Init(&hdac) != HAL_OK)
			{
				status = BSP_ERROR_PERIPH_FAILURE;
			}
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)     //TODO
			/* Register SAI TC, HT and Error callbacks */
			else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_TX_COMPLETE_CB_ID, SAI_TxCpltCallback) != HAL_OK)
			{
				status = BSP_ERROR_PERIPH_FAILURE;
			}
			else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_TX_HALFCOMPLETE_CB_ID, SAI_TxHalfCpltCallback) != HAL_OK)
			{
				status = BSP_ERROR_PERIPH_FAILURE;
			}
			else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_ERROR_CB_ID, SAI_ErrorCallback) != HAL_OK)
			{
				status = BSP_ERROR_PERIPH_FAILURE;
			}
#endif /* (USE_HAL_SAI_REGISTER_CALLBACKS == 1) */
			else
			{
				/* Update audio out context state */
				Audio_Out_Ctx[Instance].State = AUDIO_OUT_STATE_STOP;
			}
		} // PAS d'ERREUR
	}

	return status;
}

/**
  * @brief  De-initialize the audio out peripherals.
  * @param  Instance Audio out instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_DeInit(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else if (Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_RESET)
  {
    /* Reset audio codec if not currently used by audio in instance 0 */
    if (Audio_In_Ctx[0].State == AUDIO_IN_STATE_RESET)
    {
     //TODO ?
    }

    if (status == BSP_ERROR_NONE)
    {
      /* DAC peripheral de-initialization */
      if (HAL_DAC_DeInit(&hdac) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      /* De-initialize audio codec if not currently used by audio in instance 0 */
      else
      {
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 0)
        //TODO DAC_MspDeInit(&hdac);
#endif /* (USE_HAL_DAC_REGISTER_CALLBACKS == 0) */
        if (Audio_In_Ctx[0].State == AUDIO_IN_STATE_RESET)
        {

        }
      }

      if (status == BSP_ERROR_NONE)
      {
        /* Update audio out context */
        Audio_Out_Ctx[Instance].State  = AUDIO_OUT_STATE_RESET;
        Audio_Out_Ctx[Instance].IsMute = 0U;
      }
    }
  }
  else
  {
    /* Nothing to do */
  }
  return status;
}

/**
  * @brief  Start playing audio stream from a data buffer for a determined size.
  * @param  Instance Audio out instance.
  * @param  pData Pointer on data buffer.
  * @param  NbrOfBytes Size of buffer in bytes. Maximum size is 65535 bytes.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_Play(uint32_t Instance, uint8_t* pData, uint32_t NbrOfBytes)
{
  int32_t  status = BSP_ERROR_NONE;
  uint16_t NbrOfDmaDatas;

  if ((Instance >= AUDIO_OUT_INSTANCES_NBR) || (pData == NULL) || (NbrOfBytes > 65535U))
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Compute number of DMA data to transfer according resolution */
    if (Audio_Out_Ctx[Instance].BitsPerSample == AUDIO_RESOLUTION_16b)
    {
      NbrOfDmaDatas = (uint16_t) (NbrOfBytes / 2U);
    }
    else /* AUDIO_RESOLUTION_24b */
    {
      NbrOfDmaDatas = (uint16_t) (NbrOfBytes / 4U);
    }

    /* Initiate a DMA transfer of audio samples towards the serial audio interface */
    if (HAL_DAC_Transmit_DMA(&hdac,(uint16_t *) pData, NbrOfDmaDatas) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
     else
    {
      /* Update audio out state */
      Audio_Out_Ctx[Instance].State = AUDIO_OUT_STATE_PLAYING;
    }
  }
  return status;
}

/**
  * @brief  Pause playback of audio stream.
  * @param  Instance Audio out instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_Pause(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Pause DMA transfer of audio samples towards the DAC */
  else if (HAL_DAC_DMAPause(&hdac) != HAL_OK)
  {
    status = BSP_ERROR_PERIPH_FAILURE;
  }
  else
  {
    /* Update audio out state */
    Audio_Out_Ctx[Instance].State = AUDIO_OUT_STATE_PAUSE;
  }
  return status;
}

/**
  * @brief  Resume playback of audio stream.
  * @param  Instance Audio out instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_Resume(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_PAUSE)
  {
    status = BSP_ERROR_BUSY;
  }

  /* Resume DMA transfer of audio samples towards the serial audio interface */
  else if (HAL_DAC_DMAResume(&hdac) != HAL_OK)
  {
    status = BSP_ERROR_PERIPH_FAILURE;
  }
  else
  {
    /* Update audio out state */
    Audio_Out_Ctx[Instance].State = AUDIO_OUT_STATE_PLAYING;
  }
  return status;
}

/**
  * @brief  Stop playback of audio stream.
  * @param  Instance Audio out instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_Stop(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State == AUDIO_OUT_STATE_STOP)
  {
    /* Nothing to do */
  }
  else if ((Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_PLAYING) &&
           (Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_PAUSE))
  {
    status = BSP_ERROR_BUSY;
  }

  /* Stop DMA transfer of audio samples towards the serial audio interface */
  else if (HAL_DAC_DMAStop(&hdac) != HAL_OK)
  {
    status = BSP_ERROR_PERIPH_FAILURE;
  }
  else
  {
    /* Update audio out state */
    Audio_Out_Ctx[Instance].State = AUDIO_OUT_STATE_STOP;
  }
  return status;
}

/**
  * @brief  Mute playback of audio stream.
  * @param  Instance Audio out instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_Mute(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State == AUDIO_OUT_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Check audio out mute status */
  else if (Audio_Out_Ctx[Instance].IsMute == 1U)
  {
    /* Nothing to do */
  }
  /* Call the audio codec mute function */
  else if (HAL_DAC_Mute(&hdac) != HAL_OK)
  {
    status = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    /* Update audio out mute status */
    Audio_Out_Ctx[Instance].IsMute = 1U;
  }
  return status;
}

/**
  * @brief  Unmute playback of audio stream.
  * @param  Instance Audio out instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_UnMute(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State == AUDIO_OUT_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Check audio out mute status */
  else if (Audio_Out_Ctx[Instance].IsMute == 0U)
  {
    /* Nothing to do */
  }
  /* Call the audio codec mute function */
  else if (HAL_DAC_UnMute(&hdac) != HAL_OK)
  {
    status = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    /* Update audio out mute status */
    Audio_Out_Ctx[Instance].IsMute = 0U;
  }
  return status;
}

/**
  * @brief  Check audio out mute status.
  * @param  Instance Audio out instance.
  * @param  IsMute Pointer to mute status. Value is 1 for mute, 0 for unmute status.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_IsMute(uint32_t Instance, uint32_t *IsMute)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State == AUDIO_OUT_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current audio out mute status */
  else
  {
    *IsMute = Audio_Out_Ctx[Instance].IsMute;
  }
  return status;
}

/**
  * @brief  Set audio out volume.
  * @param  Instance Audio out instance.
  * @param  Volume Volume level in percentage from 0% to 100%.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t status = BSP_ERROR_NONE;

  if ((Instance >= AUDIO_OUT_INSTANCES_NBR) || (Volume > 100U))
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State == AUDIO_OUT_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Call the audio codec volume control function */
    if (Audio_Drv->SetVolume(&hdac, (uint8_t) Volume) < 0)
    {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Store volume on audio out context */
      Audio_Out_Ctx[Instance].Volume = Volume;
    }
  }
  return status;
}

/**
  * @brief  Get audio out volume.
  * @param  Instance Audio out instance.
  * @param  Volume Pointer to volume level in percentage from 0% to 100%.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State == AUDIO_OUT_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current audio out volume */
  else
  {
    *Volume = Audio_Out_Ctx[Instance].Volume;
  }
  return status;
}

/**
  * @brief  Set audio out sample rate.
  * @param  Instance Audio out instance.
  * @param  SampleRate Sample rate of the audio out stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t SampleRate)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Check if record on instance 0 is on going and corresponding sample rate */
  else if ((Audio_In_Ctx[0].State != AUDIO_IN_STATE_RESET) && 
           (Audio_In_Ctx[0].SampleRate != SampleRate))
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  /* Check if record on instance 1 is on going and corresponding sample rate */
  else if ((Audio_In_Ctx[1].State != AUDIO_IN_STATE_RESET) && 
           (Audio_In_Ctx[1].SampleRate != SampleRate))
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  /* Check if sample rate is modified */
  else if (Audio_Out_Ctx[Instance].SampleRate == SampleRate)
  {
    /* Nothing to do */
  }
  else
  {
    /* Update DAC1 clock config */
    if (MX_DAC1_ClockConfig(&hdac, SampleRate) != HAL_OK)
    {
      status = BSP_ERROR_CLOCK_FAILURE;
    }
    /* Re-initialize SAI1 with new sample rate */
    else if (MX_DAC1_Init(&hdac) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
#if (USE_HAL_SAI_REGISTER_CALLBACKS == 1)
    /* Register SAI TC, HT and Error callbacks */
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_TX_COMPLETE_CB_ID, SAI_TxCpltCallback) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_TX_HALFCOMPLETE_CB_ID, SAI_TxHalfCpltCallback) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_ERROR_CB_ID, SAI_ErrorCallback) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
#endif /* (USE_HAL_SAI_REGISTER_CALLBACKS == 1) */
    /* Store new sample rate on audio out context */
    else
    {
      Audio_Out_Ctx[Instance].SampleRate = SampleRate;
    }
  }
  return status;
}

/**
  * @brief  Get audio out sample rate.
  * @param  Instance Audio out instance.
  * @param  SampleRate Pointer to sample rate of the audio out stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State == AUDIO_OUT_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current audio out sample rate */
  else
  {
    *SampleRate = Audio_Out_Ctx[Instance].SampleRate;
  }
  return status;
}

/**
  * @brief  Set audio out device.
  * @param  Instance Audio out instance.
  * @param  Device Device of the audio out stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t status = BSP_ERROR_NONE;

  UNUSED(Device);

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Nothing to do because there is only one device (AUDIO_OUT_HEADPHONE) */
  }
  return status;
}

/**
  * @brief  Get audio out device.
  * @param  Instance Audio out instance.
  * @param  Device Pointer to device of the audio out stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_GetDevice(uint32_t Instance, uint32_t *Device)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State == AUDIO_OUT_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current audio out device */
  else
  {
    *Device = Audio_Out_Ctx[Instance].Device;
  }
  return status;
}

/**
  * @brief  Set bits per sample for the audio out stream.
  * @param  Instance Audio out instance.
  * @param  BitsPerSample Bits per sample of the audio out stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else if ((Instance == 0U) && ((BitsPerSample == AUDIO_RESOLUTION_32b) || (BitsPerSample == AUDIO_RESOLUTION_8b)))
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  else if ((Instance == 0U) && (Audio_In_Ctx[0].State != AUDIO_IN_STATE_RESET) && 
           (Audio_In_Ctx[0].BitsPerSample != BitsPerSample))
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Store new bits per sample on audio out context */
    Audio_Out_Ctx[Instance].BitsPerSample = BitsPerSample;

    /* Update data size, frame length and active frame length parameters of SAI handle */ 
    if (BitsPerSample == AUDIO_RESOLUTION_24b)
    {
   /* TODO
      haudio_out_sai.Init.DataSize               = DAC_DATASIZE_24;
      haudio_out_sai.FrameInit.FrameLength       = 64;
      haudio_out_sai.FrameInit.ActiveFrameLength = 32;
      */
    }
    else
    {
    	   /* TODO
      haudio_out_sai.Init.DataSize               = DAC_DATASIZE_16;
      haudio_out_sai.FrameInit.FrameLength       = 32;
      haudio_out_sai.FrameInit.ActiveFrameLength = 16;
      */
    }

#if (USE_HAL_SAI_REGISTER_CALLBACKS == 0)
    //TODO SAI_MspInit(&haudio_out_sai);
#else
    /* Update SAI state only to keep current MSP functions */
    haudio_out_sai.State = HAL_SAI_STATE_RESET;
#endif /* (USE_HAL_SAI_REGISTER_CALLBACKS == 0) */

    /* Re-initialize SAI1 with new parameters */
    if (HAL_DAC_Init(&hdac) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
#if (USE_HAL_SAI_REGISTER_CALLBACKS == 1)
    /* Register SAI TC, HT and Error callbacks */
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_TX_COMPLETE_CB_ID, SAI_TxCpltCallback) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_TX_HALFCOMPLETE_CB_ID, SAI_TxHalfCpltCallback) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_ERROR_CB_ID, SAI_ErrorCallback) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
#endif /* (USE_HAL_SAI_REGISTER_CALLBACKS == 1) */
    else
    {
      /* Nothing to do */
    }
  }
  return status;
}

/**
  * @brief  Get bits per sample for the audio out stream.
  * @param  Instance Audio out instance.
  * @param  BitsPerSample Pointer to bits per sample of the audio out stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State == AUDIO_OUT_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current bits per sample of audio out stream */
  else
  {
    *BitsPerSample = Audio_Out_Ctx[Instance].BitsPerSample;
  }
  return status;
}

/**
  * @brief  Set channels number for the audio out stream.
  * @param  Instance Audio out instance.
  * @param  ChannelNbr Channels number of the audio out stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State != AUDIO_OUT_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
	/* TODO
    // Update mono or stereo mode of SAI handle
    haudio_out_sai.Init.MonoStereoMode = (ChannelNbr == 1U) ? SAI_MONOMODE : SAI_STEREOMODE;

    // Re-initialize SAI1 with new parameter
    if (HAL_SAI_Init(&haudio_out_sai) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }

#if (USE_HAL_SAI_REGISTER_CALLBACKS == 1)
    // Register SAI TC, HT and Error callbacks
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_TX_COMPLETE_CB_ID, SAI_TxCpltCallback) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_TX_HALFCOMPLETE_CB_ID, SAI_TxHalfCpltCallback) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_ERROR_CB_ID, SAI_ErrorCallback) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
#endif // (USE_HAL_SAI_REGISTER_CALLBACKS == 1)
    else
    {
      // Store new channels number on audio out context
      Audio_Out_Ctx[Instance].ChannelsNbr = ChannelNbr;
    }
    TODO */
  }
  return status;
}

/**
  * @brief  Get channels number for the audio out stream.
  * @param  Instance Audio out instance.
  * @param  ChannelNbr Pointer to channels number of the audio out stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio out state */
  else if (Audio_Out_Ctx[Instance].State == AUDIO_OUT_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current channels number of audio out stream */
  else
  {
    *ChannelNbr = Audio_Out_Ctx[Instance].ChannelsNbr;
  }
  return status;
}

/**
  * @brief  Get current state for the audio out stream.
  * @param  Instance Audio out instance.
  * @param  State Pointer to state of the audio out stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_GetState(uint32_t Instance, uint32_t *State)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Get the current state of audio out stream */
  else
  {
    *State = Audio_Out_Ctx[Instance].State;
  }
  return status;
}

#if (USE_HAL_SAI_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register default BSP AUDIO OUT msp callbacks.
  * @param  Instance AUDIO OUT Instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_OUT_RegisterDefaultMspCallbacks(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit callbacks */
    if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_MSPINIT_CB_ID, SAI_MspInit) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    } 
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_MSPDEINIT_CB_ID, SAI_MspDeInit) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      AudioOut_IsMspCbValid[Instance] = 1U;
    }
  }
  /* Return BSP status */
  return status;
}

/**
  * @brief  Register BSP AUDIO OUT msp callbacks.
  * @param  Instance AUDIO OUT Instance.
  * @param  CallBacks Pointer to MspInit/MspDeInit callback functions.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_RegisterMspCallbacks(uint32_t Instance, BSP_AUDIO_OUT_Cb_t *CallBacks)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit callbacks */
    if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_MSPINIT_CB_ID, CallBacks->pMspInitCb) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_SAI_RegisterCallback(&haudio_out_sai, HAL_SAI_MSPDEINIT_CB_ID, CallBacks->pMspDeInitCb) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      AudioOut_IsMspCbValid[Instance] = 1U;
    }
  }
  /* Return BSP status */
  return status; 
}
#endif /* (USE_HAL_SAI_REGISTER_CALLBACKS == 1) */

/**
  * @brief  Manage the BSP audio out transfer complete event.
  * @param  Instance Audio out instance.
  * @retval None.
  */
__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
}

/**
  * @brief  Manage the BSP audio out half transfer complete event.
  * @param  Instance Audio out instance.
  * @retval None.
  */
__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
}

/**
  * @brief  Manages the BSP audio out error event.
  * @param  Instance Audio out instance.
  * @retval None.
  */
__weak void BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
}

/**
  * @brief  BSP AUDIO OUT interrupt handler.
  * @param  Instance Audio out instance.
  * @param  Device Device of the audio out stream.
  * @retval None.
  */
void BSP_AUDIO_OUT_IRQHandler(uint32_t Instance, uint32_t Device)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  UNUSED(Device);

  //TODO HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
}




/**
  * @}
  */ 


/** @addtogroup STM32L475E_IOT01_AUDIO_IN_Exported_Functions
  * @{
  */
/**
  * @brief  Initialize the audio in peripherals.
  * @param  Instance Audio in instance.
  * @param  AudioInit Audio in init structure.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t *AudioInit)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  else if (AudioInit->BitsPerSample != AUDIO_RESOLUTION_16b)
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  else if (((AudioInit->Device == AUDIO_IN_DIGITAL_MIC) && (AudioInit->ChannelsNbr != 2U)) ||
           ((AudioInit->Device == AUDIO_IN_DIGITAL_MIC1) && (AudioInit->ChannelsNbr != 1U)) ||
           ((AudioInit->Device == AUDIO_IN_DIGITAL_MIC2) && (AudioInit->ChannelsNbr != 1U)))
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  else
  {
    /* Fill audio in context structure */
    Audio_In_Ctx[Instance].Device         = AudioInit->Device;
    Audio_In_Ctx[Instance].SampleRate     = AudioInit->SampleRate;
    Audio_In_Ctx[Instance].BitsPerSample  = AudioInit->BitsPerSample;
    Audio_In_Ctx[Instance].ChannelsNbr    = AudioInit->ChannelsNbr;
    Audio_In_Ctx[Instance].Volume         = AudioInit->Volume;

    /* Set DFSDM instances */
    haudio_in_dfsdm_channel[0].Instance = DFSDM1_Channel2;
    haudio_in_dfsdm_channel[1].Instance = DFSDM1_Channel1;
    haudio_in_dfsdm_filter[0].Instance  = DFSDM1_Filter0;
    haudio_in_dfsdm_filter[1].Instance  = DFSDM1_Filter1;

    /* Configure DFSDM clock according to the requested audio frequency */
    if (MX_DFSDM1_ClockConfig(&haudio_in_dfsdm_channel[0], AudioInit->SampleRate) != HAL_OK)
    {
      status = BSP_ERROR_CLOCK_FAILURE;
    }
    else
    {
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
      if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
      {
        DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[0]);
        DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[0]);
      }
      if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
      {
        DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[1]);
        DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[1]);
      }
#else
      /* Register the DFSDM MSP Callbacks */
      if(AudioIn_IsMspCbValid[Instance] == 0U)
      {
        if(BSP_AUDIO_IN_RegisterDefaultMspCallbacks(Instance) != BSP_ERROR_NONE)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      if (status == BSP_ERROR_NONE)
      {
        /* Prepare DFSDM peripheral initialization */
        MX_DFSDM_InitTypeDef mxDfsdmInit;
        if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
        {
          mxDfsdmInit.ChannelInstance = DFSDM1_Channel2;
          mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
          mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_FALLING;
          mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_2;
          mxDfsdmInit.FilterInstance  = DFSDM1_Filter0;
          mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
          mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(Audio_In_Ctx[Instance].SampleRate);
          if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0], &mxDfsdmInit) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
          if (status == BSP_ERROR_NONE)
          {
            /* Register DFSDM filter TC, HT and Error callbacks */
            if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
            else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
            else
            {
              if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
              {
                status = BSP_ERROR_PERIPH_FAILURE;
              }
            }
          }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
        }
        if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
        {
          mxDfsdmInit.ChannelInstance = DFSDM1_Channel1;
          mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
          mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_RISING;
          mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_1;
          mxDfsdmInit.FilterInstance  = DFSDM1_Filter1;
          if (Audio_In_Ctx[Instance].Device == AUDIO_IN_DIGITAL_MIC2)
          {
            mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
          }
          else
          {
            mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SYNC_TRIGGER;
          }
          mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(Audio_In_Ctx[Instance].SampleRate);
          if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1], &mxDfsdmInit) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
          if (status == BSP_ERROR_NONE)
          {
            /* Register DFSDM filter TC, HT and Error callbacks */
            if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
            else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
            else
            {
              if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
              {
                status = BSP_ERROR_PERIPH_FAILURE;
              }
            }
          }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
        }
        if (status == BSP_ERROR_NONE)
        {
          /* Initialise transfer control flag */
          Audio_DmaDigMic1RecHalfBuffCplt = 0;
          Audio_DmaDigMic1RecBuffCplt     = 0;
          Audio_DmaDigMic2RecHalfBuffCplt = 0;
          Audio_DmaDigMic2RecBuffCplt     = 0;

          /* Update audio in context state */
          Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_STOP;
        }
      }
    }
  }
  return status;
}

/**
  * @brief  De-initialize the audio in peripherals.
  * @param  Instance Audio in instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_DeInit(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_RESET)
  {
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      /* DFSDM peripheral de-initialization */
      if (DFSDM_DeInit(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1]) != BSP_ERROR_NONE)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
        DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[1]);
        DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[1]);
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      }
    }

    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      /* DFSDM peripheral de-initialization */
      if (DFSDM_DeInit(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0]) != BSP_ERROR_NONE)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
        DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[0]);
        DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[0]);
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      }
    }

    if (status == BSP_ERROR_NONE)
    {
      /* Update audio in context */
      Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_RESET;
    }
  }
  else
  {
    /* Nothing to do */
  }
  return status;
}

/**
  * @brief  Start recording audio stream to a data buffer for a determined size.
  * @param  Instance Audio in instance.
  * @param  pData Pointer on data buffer.
  * @param  NbrOfBytes Size of buffer in bytes. Maximum size is 65535 bytes.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t* pData, uint32_t NbrOfBytes)
{
  int32_t  status = BSP_ERROR_NONE;

  if ((Instance >= AUDIO_IN_INSTANCES_NBR) || (pData == NULL) || (NbrOfBytes > 65535U))
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check the internal buffer size */
  else if ((NbrOfBytes / 2U) > BSP_AUDIO_IN_DEFAULT_BUFFER_SIZE)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    Audio_In_Ctx[Instance].pBuff = pData;
    Audio_In_Ctx[Instance].Size  = NbrOfBytes;

    /* Initialise transfer control flag */
    Audio_DmaDigMic1RecHalfBuffCplt = 0;
    Audio_DmaDigMic1RecBuffCplt     = 0;
    Audio_DmaDigMic2RecHalfBuffCplt = 0;
    Audio_DmaDigMic2RecBuffCplt     = 0;

    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      /* Call the Media layer start function for MIC2 channel */
      if(HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_filter[1],
                                           Audio_DigMic2RecBuff,
                                          (Audio_In_Ctx[Instance].Size / (2U * Audio_In_Ctx[Instance].ChannelsNbr))) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }

    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      /* Call the Media layer start function for MIC1 channel */
      if(HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_filter[0],
                                           Audio_DigMic1RecBuff,
                                          (Audio_In_Ctx[Instance].Size / (2U * Audio_In_Ctx[Instance].ChannelsNbr))) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (status == BSP_ERROR_NONE)
    {
      /* Update audio in state */
      Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_RECORDING;
    }
  }
  return status;
}

/**
  * @brief  Pause record of audio stream.
  * @param  Instance Audio in instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_Pause(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_RECORDING)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Call the Media layer stop function */
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      if(HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_filter[1]) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      if(HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_filter[0]) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (status == BSP_ERROR_NONE)
    {
      /* Update audio in state */
      Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_PAUSE;
    }
  }
  return status;
}

/**
  * @brief  Resume record of audio stream.
  * @param  Instance Audio in instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_Resume(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_PAUSE)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Initialise transfer control flag */
    Audio_DmaDigMic1RecHalfBuffCplt = 0;
    Audio_DmaDigMic1RecBuffCplt     = 0;
    Audio_DmaDigMic2RecHalfBuffCplt = 0;
    Audio_DmaDigMic2RecBuffCplt     = 0;

    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      /* Call the Media layer start function for MIC2 channel */
      if(HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_filter[1],
                                           Audio_DigMic2RecBuff,
                                          (Audio_In_Ctx[Instance].Size / (2U * Audio_In_Ctx[Instance].ChannelsNbr))) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      /* Call the Media layer start function for MIC1 channel */
      if(HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_filter[0],
                                           Audio_DigMic1RecBuff,
                                          (Audio_In_Ctx[Instance].Size / (2U * Audio_In_Ctx[Instance].ChannelsNbr))) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (status == BSP_ERROR_NONE)
    {
      /* Update audio in state */
      Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_RECORDING;
    }
  }
  return status;
}

/**
  * @brief  Stop record of audio stream.
  * @param  Instance Audio in instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_Stop(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    /* Nothing to do */
  }
  else if ((Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_RECORDING) &&
           (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_PAUSE))
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Call the Media layer stop function */
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      if(HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_filter[1]) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      if(HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_filter[0]) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (status == BSP_ERROR_NONE)
    {
      /* Update audio in state */
      Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_STOP;
    }
  }
  return status;
}

/**
  * @brief  Set audio in volume.
  * @param  Instance Audio in instance.
  * @param  Volume Volume level in percentage from 0% to 100%.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t status;

  if ((Instance >= AUDIO_IN_INSTANCES_NBR) || (Volume > 100U))
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Feature not supported */
  else
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  return status;
}

/**
  * @brief  Get audio in volume.
  * @param  Instance Audio in instance.
  * @param  Volume Pointer to volume level in percentage from 0% to 100%.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  int32_t status;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Feature not supported */
  else
  {
    *Volume = 0U;
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  return status;
}

/**
  * @brief  Set audio in sample rate.
  * @param  Instance Audio in instance.
  * @param  SampleRate Sample rate of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Check if sample rate is modified */
  else if (Audio_In_Ctx[Instance].SampleRate == SampleRate)
  {
    /* Nothing to do */
  }
  else
  {
    /* Configure DFSDM clock according to the requested audio frequency */
    if (MX_DFSDM1_ClockConfig(&haudio_in_dfsdm_channel[0], SampleRate) != HAL_OK)
    {
      status = BSP_ERROR_CLOCK_FAILURE;
    }
    else
    {
      MX_DFSDM_InitTypeDef mxDfsdmInit;

      /* DeInitialize DFSDM */
      if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
      {
        if (DFSDM_DeInit(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1]) != BSP_ERROR_NONE)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
        else
        {
          DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[1]);
          DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[1]);
        }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      }
      if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
      {
        if (DFSDM_DeInit(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0]) != BSP_ERROR_NONE)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
        else
        {
          DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[0]);
          DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[0]);
        }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      }

      /* ReInitialize DFSDM with new sample rate */
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
      if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
      {
        DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[0]);
        DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[0]);
      }
      if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
      {
        DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[1]);
        DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[1]);
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
      {
        mxDfsdmInit.ChannelInstance = DFSDM1_Channel2;
        mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(SampleRate);
        mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
        mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_FALLING;
        mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(SampleRate);
        mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_2;
        mxDfsdmInit.FilterInstance  = DFSDM1_Filter0;
        mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
        mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(SampleRate);
        mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(SampleRate);
        if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0], &mxDfsdmInit) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
        if (status == BSP_ERROR_NONE)
        {
          /* Register DFSDM filter TC, HT and Error callbacks */
          if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
          else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
          else
          {
            if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
          }
        }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
      }
      if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
      {
        mxDfsdmInit.ChannelInstance = DFSDM1_Channel1;
        mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(SampleRate);
        mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
        mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_RISING;
        mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(SampleRate);
        mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_1;
        mxDfsdmInit.FilterInstance  = DFSDM1_Filter1;
        if (Audio_In_Ctx[Instance].Device == AUDIO_IN_DIGITAL_MIC2)
        {
          mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
        }
        else
        {
          mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SYNC_TRIGGER;
        }
        mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(SampleRate);
        mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(SampleRate);
        if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1], &mxDfsdmInit) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
        if (status == BSP_ERROR_NONE)
        {
          /* Register DFSDM filter TC, HT and Error callbacks */
          if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
          else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
          else
          {
            if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
          }
        }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
      }
    }
    /* Store new sample rate on audio in context */
    if (status == BSP_ERROR_NONE)
    {
      Audio_In_Ctx[Instance].SampleRate = SampleRate;
    }
  }
  return status;
}

/**
  * @brief  Get audio in sample rate.
  * @param  Instance Audio in instance.
  * @param  SampleRate Pointer to sample rate of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State == AUDIO_IN_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current audio in sample rate */
  else
  {
    *SampleRate = Audio_In_Ctx[Instance].SampleRate;
  }
  return status;
}

/**
  * @brief  Set audio in device.
  * @param  Instance Audio in instance.
  * @param  Device Device of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t status = BSP_ERROR_NONE;

  UNUSED(Device);

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else if (Audio_In_Ctx[Instance].Device != Device)
  {
    MX_DFSDM_InitTypeDef mxDfsdmInit;

    /* DeInitialize DFSDM for previous microphones */
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      if (DFSDM_DeInit(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1]) != BSP_ERROR_NONE)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
      else
      {
        DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[1]);
        DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[1]);
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
    }
    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      if (DFSDM_DeInit(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0]) != BSP_ERROR_NONE)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
      else
      {
        DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[0]);
        DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[0]);
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
    }

    /* ReInitialize DFSDM for new microphones */
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
    if (((Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[0]);
      DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[0]);
    }
    if (((Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
    {
      DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[1]);
      DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[1]);
    }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
    if (((Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      mxDfsdmInit.ChannelInstance = DFSDM1_Channel2;
      mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
      mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_FALLING;
      mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_2;
      mxDfsdmInit.FilterInstance  = DFSDM1_Filter0;
      mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
      mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(Audio_In_Ctx[Instance].SampleRate);
      if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0], &mxDfsdmInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
      if (status == BSP_ERROR_NONE)
      {
        /* Register DFSDM filter TC, HT and Error callbacks */
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
        else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
        else
        {
          if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
        }
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
    }
    if (((Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
    {
      mxDfsdmInit.ChannelInstance = DFSDM1_Channel1;
      mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
      mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_RISING;
      mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_1;
      mxDfsdmInit.FilterInstance  = DFSDM1_Filter1;
      if (Device == AUDIO_IN_DIGITAL_MIC2)
      {
        mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
      }
      else
      {
        mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SYNC_TRIGGER;
      }
      mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(Audio_In_Ctx[Instance].SampleRate);
      if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1], &mxDfsdmInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
      if (status == BSP_ERROR_NONE)
      {
        /* Register DFSDM filter TC, HT and Error callbacks */
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
        else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
        else
        {
          if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
        }
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
    }
    if (status == BSP_ERROR_NONE)
    {
      /* Store new device on context and update channel numbers */
      Audio_In_Ctx[Instance].Device = Device;
      Audio_In_Ctx[Instance].ChannelsNbr = (Device == AUDIO_IN_DIGITAL_MIC) ? 2U : 1U;
    }
  }
  return status;
}

/**
  * @brief  Get audio in device.
  * @param  Instance Audio in instance.
  * @param  Device Pointer to device of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State == AUDIO_IN_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current audio in device */
  else
  {
    *Device = Audio_In_Ctx[Instance].Device;
  }
  return status;
}

/**
  * @brief  Set bits per sample for the audio in stream.
  * @param  Instance Audio in instance.
  * @param  BitsPerSample Bits per sample of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else if (BitsPerSample != AUDIO_RESOLUTION_16b)
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Nothing to do because there is only one bits per sample supported (AUDIO_RESOLUTION_16b) */
  }
  return status;
}

/**
  * @brief  Get bits per sample for the audio in stream.
  * @param  Instance Audio in instance.
  * @param  BitsPerSample Pointer to bits per sample of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State == AUDIO_IN_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current bits per sample of audio in stream */
  else
  {
    *BitsPerSample = Audio_In_Ctx[Instance].BitsPerSample;
  }
  return status;
}

/**
  * @brief  Set channels number for the audio in stream.
  * @param  Instance Audio in instance.
  * @param  ChannelNbr Channels number of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else if (((Audio_In_Ctx[Instance].Device == AUDIO_IN_DIGITAL_MIC) && (ChannelNbr != 2U)) ||
           ((Audio_In_Ctx[Instance].Device == AUDIO_IN_DIGITAL_MIC1) && (ChannelNbr != 1U)) ||
           ((Audio_In_Ctx[Instance].Device == AUDIO_IN_DIGITAL_MIC2) && (ChannelNbr != 1U)))
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Nothing to do because channels are already configurated and can't be modified */
  }
  return status;
}

/**
  * @brief  Get channels number for the audio in stream.
  * @param  Instance Audio in instance.
  * @param  ChannelNbr Pointer to channels number of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State == AUDIO_IN_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current channels number of audio in stream */
  else
  {
    *ChannelNbr = Audio_In_Ctx[Instance].ChannelsNbr;
  }
  return status;
}

/**
  * @brief  Get current state for the audio in stream.
  * @param  Instance Audio in instance.
  * @param  State Pointer to state of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Get the current state of audio in stream */
  else
  {
    *State = Audio_In_Ctx[Instance].State;
  }
  return status;
}

#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register default BSP AUDIO IN msp callbacks.
  * @param  Instance AUDIO IN Instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_RegisterDefaultMspCallbacks(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit callbacks */
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
    {
      if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[0], HAL_DFSDM_CHANNEL_MSPINIT_CB_ID, DFSDM_ChannelMspInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_MSPINIT_CB_ID, DFSDM_FilterMspInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[0], HAL_DFSDM_CHANNEL_MSPDEINIT_CB_ID, DFSDM_ChannelMspDeInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_MSPDEINIT_CB_ID, DFSDM_FilterMspDeInit) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
      }
    }

    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
    {
      if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[1], HAL_DFSDM_CHANNEL_MSPINIT_CB_ID, DFSDM_ChannelMspInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_MSPINIT_CB_ID, DFSDM_FilterMspInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[1], HAL_DFSDM_CHANNEL_MSPDEINIT_CB_ID, DFSDM_ChannelMspDeInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_MSPDEINIT_CB_ID, DFSDM_FilterMspDeInit) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
      }
    }

    if (status == BSP_ERROR_NONE)
    {
      AudioIn_IsMspCbValid[Instance] = 1U;
    }
  }
  /* Return BSP status */
  return status;
}

/**
  * @brief  Register BSP AUDIO IN msp callbacks.
  * @param  Instance AUDIO IN Instance.
  * @param  CallBacks Pointer to MspInit/MspDeInit callback functions.
  * @retval BSP status
  */
int32_t BSP_AUDIO_IN_RegisterMspCallbacks(uint32_t Instance, BSP_AUDIO_IN_Cb_t *CallBacks)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit callbacks */
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
    {
      if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[0], HAL_DFSDM_CHANNEL_MSPINIT_CB_ID, CallBacks->pMspChannelInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_MSPINIT_CB_ID, CallBacks->pMspFilterInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[0], HAL_DFSDM_CHANNEL_MSPDEINIT_CB_ID, CallBacks->pMspChannelDeInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_MSPDEINIT_CB_ID, CallBacks->pMspFilterDeInitCb) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
      }
    }

    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
    {
      if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[1], HAL_DFSDM_CHANNEL_MSPINIT_CB_ID, CallBacks->pMspChannelInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_MSPINIT_CB_ID, CallBacks->pMspFilterInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[1], HAL_DFSDM_CHANNEL_MSPDEINIT_CB_ID, CallBacks->pMspChannelDeInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_MSPDEINIT_CB_ID, CallBacks->pMspFilterDeInitCb) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
      }
    }

    if (status == BSP_ERROR_NONE)
    {
      AudioIn_IsMspCbValid[Instance] = 1U;
    }
  }
  /* Return BSP status */
  return status;
}
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */

/**
  * @brief  Manage the BSP audio in transfer complete event.
  * @param  Instance Audio in instance.
  * @retval None.
  */
__weak void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
}

/**
  * @brief  Manage the BSP audio in half transfer complete event.
  * @param  Instance Audio in instance.
  * @retval None.
  */
__weak void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
}

/**
  * @brief  Manages the BSP audio in error event.
  * @param  Instance Audio in instance.
  * @retval None.
  */
__weak void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
}

/**
  * @brief  BSP AUDIO IN interrupt handler.
  * @param  Instance Audio in instance.
  * @param  Device Device of the audio in stream.
  * @retval None.
  */
void BSP_AUDIO_IN_IRQHandler(uint32_t Instance, uint32_t Device)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);

  if (Device == AUDIO_IN_DIGITAL_MIC1)
  {
    HAL_DMA_IRQHandler(haudio_in_dfsdm_filter[0].hdmaReg);
  }
  else
  {
    HAL_DMA_IRQHandler(haudio_in_dfsdm_filter[1].hdmaReg);
  }
}

/**
  * @brief  DFSDM1 clock Config.
  * @param  hDfsdmChannel DFSDM channel handle.
  * @param  SampleRate Audio sample rate used to record the audio stream.
  * @note   The SAI PLL configuration done within this function assumes that
  *         the SAI PLL input is MSI clock and that MSI is already enabled at 4 MHz.
  * @retval HAL status.
  */
__weak HAL_StatusTypeDef MX_DFSDM1_ClockConfig(DFSDM_Channel_HandleTypeDef *hDfsdmChannel, uint32_t SampleRate)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmChannel);

  HAL_StatusTypeDef        status;
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;

  /* Configure the SAI PLL according to the requested audio frequency */
  /* Retrieve actual RCC configuration */
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);

  /* Set the PLL configuration according to the audio frequency */
  if((SampleRate == AUDIO_FREQUENCY_11K) || (SampleRate == AUDIO_FREQUENCY_22K) || (SampleRate == AUDIO_FREQUENCY_44K) ||
     (SampleRate == AUDIO_FREQUENCY_88K) || (SampleRate == AUDIO_FREQUENCY_176K))
  {
    /* SAI1 clock config
    PLLSAI1_VCO = (4 Mhz / PLLSAI1M) * PLLSAI1N = 4 * 48 = VCO_192M
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 192/17 = 11.294 Mhz */
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1Source   = RCC_PLLSOURCE_MSI;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1M        = 1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 48;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 17;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_32K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K or AUDIO_FREQUENCY_192K */
  {
    /* SAI1 clock config
    PLLSAI1_VCO = (4 Mhz / PLLSAI1M) * PLLSAI1N = 4 * 86 = VCO_344M
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 344/7 = 49.142 Mhz */
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1Source   = RCC_PLLSOURCE_MSI;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1M        = 1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 86;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 7;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
  }
  status = HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);

  return status;
}

/**
  * @brief  Initialize DFSDM1.
  * @param  hDfsdmFilter  DFSDM filter handle.
  * @param  hDfsdmChannel DFSDM channel handle.
  * @param  MXInit DFSDM configuration structure.
  * @retval HAL_status.
  */
__weak HAL_StatusTypeDef MX_DFSDM1_Init(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel, MX_DFSDM_InitTypeDef *MXInit)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* MIC channels initialization */
  hDfsdmChannel->Instance                      = MXInit->ChannelInstance;
  hDfsdmChannel->Init.OutputClock.Activation   = ENABLE;
  hDfsdmChannel->Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hDfsdmChannel->Init.OutputClock.Divider      = MXInit->ClockDivider;
  hDfsdmChannel->Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hDfsdmChannel->Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
  hDfsdmChannel->Init.Input.Pins               = MXInit->DigitalMicPins;
  hDfsdmChannel->Init.SerialInterface.Type     = MXInit->DigitalMicType;
  hDfsdmChannel->Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hDfsdmChannel->Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
  hDfsdmChannel->Init.Awd.Oversampling         = 10;
  hDfsdmChannel->Init.Offset                   = 0;
  hDfsdmChannel->Init.RightBitShift            = MXInit->RightBitShift;

  if(HAL_OK != HAL_DFSDM_ChannelInit(hDfsdmChannel))
  {
    status = HAL_ERROR;
  }
  else
  {
    /* MIC filters  initialization */
    hDfsdmFilter->Instance                          = MXInit->FilterInstance;
    hDfsdmFilter->Init.RegularParam.Trigger         = MXInit->RegularTrigger;
    hDfsdmFilter->Init.RegularParam.FastMode        = ENABLE;
    hDfsdmFilter->Init.RegularParam.DmaMode         = ENABLE;
    hDfsdmFilter->Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
    hDfsdmFilter->Init.InjectedParam.ScanMode       = DISABLE;
    hDfsdmFilter->Init.InjectedParam.DmaMode        = DISABLE;
    hDfsdmFilter->Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
    hDfsdmFilter->Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
    hDfsdmFilter->Init.FilterParam.SincOrder        = MXInit->SincOrder;
    hDfsdmFilter->Init.FilterParam.Oversampling     = MXInit->Oversampling;
    hDfsdmFilter->Init.FilterParam.IntOversampling  = 1;

    if(HAL_DFSDM_FilterInit(hDfsdmFilter) != HAL_OK)
    {
      status = HAL_ERROR;
    }
    else
    {
      /* Configure regular channel */
      if(HAL_DFSDM_FilterConfigRegChannel(hDfsdmFilter, MXInit->Channel4Filter, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
      {
        status = HAL_ERROR;
      }
    }
  }

  return status;
}
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_Private_Functions STM32L475E_IOT01 AUDIO_IN Private Functions
  * @{
  */
/**
  * @brief  DeInitialize DFSDM.
  * @param  hDfsdmFilter  DFSDM filter handle.
  * @param  hDfsdmChannel DFSDM channel handle.
  * @retval BSP status.
  */
static int32_t DFSDM_DeInit(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel)
{
  int32_t status = BSP_ERROR_NONE;

  /* MIC filters Deinitialization */
  if(HAL_DFSDM_FilterDeInit(hDfsdmFilter) != HAL_OK)
  {
    status = BSP_ERROR_PERIPH_FAILURE;
  }
  else
  {
    /* MIC channels Deinitialization */
    if(HAL_OK != HAL_DFSDM_ChannelDeInit(hDfsdmChannel))
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Initialize DFSDM channel MSP.
  * @param  hdfsdm_channel DFSDM channel handle.
  * @retval None.
  */
static void DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel)
{
  if (((hdfsdm_channel->Instance == DFSDM1_Channel2) && ((Audio_In_Ctx[0].Device & AUDIO_IN_DIGITAL_MIC1) != 0U)) || \
      ((hdfsdm_channel->Instance == DFSDM1_Channel1) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable DFSDM clock */
    AUDIO_DFSDM1_CLK_ENABLE();

    /* DFSDM pins configuration: DFSDM1_CKOUT, DFSDM1_DATIN2 pins */
    AUDIO_DFSDM1_CKOUT_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = AUDIO_DFSDM1_CKOUT_GPIO_AF;
    GPIO_InitStruct.Pin       = AUDIO_DFSDM1_CKOUT_GPIO_PIN;
    HAL_GPIO_Init(AUDIO_DFSDM1_CKOUT_GPIO_PORT, &GPIO_InitStruct);

    AUDIO_DFSDM1_DATIN2_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Alternate = AUDIO_DFSDM1_DATIN2_GPIO_AF;
    GPIO_InitStruct.Pin       = AUDIO_DFSDM1_DATIN2_GPIO_PIN;
    HAL_GPIO_Init(AUDIO_DFSDM1_DATIN2_GPIO_PORT, &GPIO_InitStruct);
  }
}

/**
  * @brief  DeInitialize DFSDM channel MSP.
  * @param  hdfsdm_channel DFSDM channel handle.
  * @retval None.
  */
static void DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel)
{
  if (((hdfsdm_channel->Instance == DFSDM1_Channel2) && ((Audio_In_Ctx[0].Device & AUDIO_IN_DIGITAL_MIC1) != 0U)) || \
      ((hdfsdm_channel->Instance == DFSDM1_Channel1) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    /* De-initialize DFSDM1_CKOUT, DFSDM1_DATIN2 pins */
    HAL_GPIO_DeInit(AUDIO_DFSDM1_CKOUT_GPIO_PORT, AUDIO_DFSDM1_CKOUT_GPIO_PIN);
    HAL_GPIO_DeInit(AUDIO_DFSDM1_DATIN2_GPIO_PORT, AUDIO_DFSDM1_DATIN2_GPIO_PIN);

    /* Disable DFSDM1 */
    AUDIO_DFSDM1_CLK_DISABLE();
  }
}

/**
  * @brief  Initialize DFSDM filter MSP.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
static void DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if(hdfsdm_filter->Instance == DFSDM1_Filter0)
  {
    /* Enable the DMA clock */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Configure the hDmaDfsdm[0] handle parameters */
    hDmaDfsdm[0].Init.Request             = DMA_REQUEST_0;
    hDmaDfsdm[0].Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDmaDfsdm[0].Init.PeriphInc           = DMA_PINC_DISABLE;
    hDmaDfsdm[0].Init.MemInc              = DMA_MINC_ENABLE;
    hDmaDfsdm[0].Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hDmaDfsdm[0].Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hDmaDfsdm[0].Init.Mode                = DMA_CIRCULAR;
    hDmaDfsdm[0].Init.Priority            = DMA_PRIORITY_HIGH;
    hDmaDfsdm[0].Instance                 = DMA1_Channel4;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hdfsdm_filter, hdmaReg, hDmaDfsdm[0]);

    /* Deinitialize the DMA channel for new transfer */
    if (HAL_DMA_DeInit(&hDmaDfsdm[0]) != HAL_OK)
    {
      /* Nothing to do */
    }

    /* Configure the DMA Channel */
    if (HAL_DMA_Init(&hDmaDfsdm[0]) != HAL_OK)
    {
      /* Nothing to do */
    }

    /* DMA IRQ Channel configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, BSP_AUDIO_IN_IT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  }
  else /* DFSDM1_Filter1 */
  {
    if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)
    {
      /* Enable the DMA clock needed if only MIC2 is used */
      __HAL_RCC_DMA1_CLK_ENABLE();
    }

    /* Configure the hDmaDfsdm[1] handle parameters */
    hDmaDfsdm[1].Init.Request             = DMA_REQUEST_0;
    hDmaDfsdm[1].Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDmaDfsdm[1].Init.PeriphInc           = DMA_PINC_DISABLE;
    hDmaDfsdm[1].Init.MemInc              = DMA_MINC_ENABLE;
    hDmaDfsdm[1].Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hDmaDfsdm[1].Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hDmaDfsdm[1].Init.Mode                = DMA_CIRCULAR;
    hDmaDfsdm[1].Init.Priority            = DMA_PRIORITY_HIGH;
    hDmaDfsdm[1].Instance                 = DMA1_Channel5;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hdfsdm_filter, hdmaReg, hDmaDfsdm[1]);

    /* Deinitialize the DMA channel for new transfer */
    if (HAL_DMA_DeInit(&hDmaDfsdm[1]) != HAL_OK)
    {
      /* Nothing to do */
    }

    /* Configure the DMA Channel */
    if (HAL_DMA_Init(&hDmaDfsdm[1]) != HAL_OK)
    {
      /* Nothing to do */
    }

    /* DMA IRQ Channel configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, BSP_AUDIO_IN_IT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  }
}

/**
  * @brief  DeInitialize DFSDM filter MSP.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
static void DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if(hdfsdm_filter->Instance == DFSDM1_Filter0)
  {
    /* Disable DMA  Channel IRQ */
    HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);

    /* De-initialize the DMA Channel */
    if (HAL_DMA_DeInit(&hDmaDfsdm[0]) != HAL_OK)
    {
      /* Nothing to do */
    }
  }
  else /* DFSDM1_Filter1 */
  {
    /* Disable DMA  Channel IRQ */
    HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);

    /* De-initialize the DMA Channel */
    if (HAL_DMA_DeInit(&hDmaDfsdm[1]) != HAL_OK)
    {
      /* Nothing to do */
    }
  }
}

#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
/**
  * @brief  DFSDM filter regular conversion complete callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
static void DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t     index;
  uint32_t     recbufsize = (Audio_In_Ctx[0].Size / (2U * Audio_In_Ctx[0].ChannelsNbr));
  __IO int32_t tmp;

  if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)
  {
    for (index = (recbufsize / 2U); index < recbufsize; index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[4U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      tmp = Audio_DigMic2RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[(4U * index) + 2U] = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 3U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)
  {
    for (index = (recbufsize / 2U); index < recbufsize; index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else
  {
    if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)
    {
      for (index = (recbufsize / 2U); index < recbufsize; index++)
      {
        tmp = Audio_DigMic2RecBuff[index] / 256;
        tmp = SaturaLH(tmp, -32768, 32767);
        Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
        Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      }
    }
  }

  /* Invoke 'TransferCompete' callback function */
  if(hdfsdm_filter == &haudio_in_dfsdm_filter[0])
  {
    Audio_DmaDigMic1RecBuffCplt = 1;
  }
  else
  {
    Audio_DmaDigMic2RecBuffCplt = 1;
  }

  if (((Audio_DmaDigMic1RecBuffCplt != 0U) && (Audio_DmaDigMic2RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)) ||
      ((Audio_DmaDigMic1RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)) ||
      ((Audio_DmaDigMic2RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    BSP_AUDIO_IN_TransferComplete_CallBack(0);
    Audio_DmaDigMic1RecBuffCplt = 0;
    Audio_DmaDigMic2RecBuffCplt = 0;
  }
}

/**
  * @brief  DFSDM filter regular conversion half complete callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
static void DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t     index;
  uint32_t     recbufsize = (Audio_In_Ctx[0].Size / (2U * Audio_In_Ctx[0].ChannelsNbr));
  __IO int32_t tmp;

  if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)
  {
    for (index = 0; index < (recbufsize / 2U); index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[4U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      tmp = Audio_DigMic2RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[(4U * index) + 2U] = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 3U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)
  {
    for (index = 0; index < (recbufsize / 2U); index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else
  {
    if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)
    {
      for (index = 0; index < (recbufsize / 2U); index++)
      {
        tmp = Audio_DigMic2RecBuff[index] / 256;
        tmp = SaturaLH(tmp, -32768, 32767);
        Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
        Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      }
    }
  }

  /* Invoke the 'HalfTransfer' callback function */
  if(hdfsdm_filter == &haudio_in_dfsdm_filter[0])
  {
    Audio_DmaDigMic1RecHalfBuffCplt = 1;
  }
  else
  {
    Audio_DmaDigMic2RecHalfBuffCplt = 1;
  }

  if (((Audio_DmaDigMic1RecHalfBuffCplt != 0U) && (Audio_DmaDigMic2RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)) ||
      ((Audio_DmaDigMic1RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)) ||
      ((Audio_DmaDigMic2RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    BSP_AUDIO_IN_HalfTransfer_CallBack(0);
    Audio_DmaDigMic1RecHalfBuffCplt = 0;
    Audio_DmaDigMic2RecHalfBuffCplt = 0;
  }
}

/**
  * @brief  DFSDM filter error callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
static void DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  UNUSED(hdfsdm_filter);

  BSP_AUDIO_IN_Error_CallBack(0);
}
#else /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
/**
  * @brief  DFSDM filter regular conversion complete callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
__weak void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t     index;
  uint32_t     recbufsize = (Audio_In_Ctx[0].Size / (2U * Audio_In_Ctx[0].ChannelsNbr));
  __IO int32_t tmp;

  if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)
  {
    for (index = (recbufsize / 2U); index < recbufsize; index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[4U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      tmp = Audio_DigMic2RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[(4U * index) + 2U] = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 3U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)
  {
    for (index = (recbufsize / 2U); index < recbufsize; index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else
  {
    if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)
    {
      for (index = (recbufsize / 2U); index < recbufsize; index++)
      {
        tmp = Audio_DigMic2RecBuff[index] / 256;
        tmp = SaturaLH(tmp, -32768, 32767);
        Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
        Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      }
    }
  }

  /* Invoke 'TransferCompete' callback function */
  if(hdfsdm_filter == &haudio_in_dfsdm_filter[0])
  {
    Audio_DmaDigMic1RecBuffCplt = 1;
  }
  else
  {
    Audio_DmaDigMic2RecBuffCplt = 1;
  }

  if (((Audio_DmaDigMic1RecBuffCplt != 0U) && (Audio_DmaDigMic2RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)) ||
      ((Audio_DmaDigMic1RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)) ||
      ((Audio_DmaDigMic2RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    BSP_AUDIO_IN_TransferComplete_CallBack(0);
    Audio_DmaDigMic1RecBuffCplt = 0;
    Audio_DmaDigMic2RecBuffCplt = 0;
  }
}

/**
  * @brief  DFSDM filter regular conversion half complete callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
__weak void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t     index;
  uint32_t     recbufsize = (Audio_In_Ctx[0].Size / (2U * Audio_In_Ctx[0].ChannelsNbr));
  __IO int32_t tmp;

  if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)
  {
    for (index = 0; index < (recbufsize / 2U); index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[4U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      tmp = Audio_DigMic2RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[(4U * index) + 2U] = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 3U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)
  {
    for (index = 0; index < (recbufsize / 2U); index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else
  {
    if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)
    {
      for (index = 0; index < (recbufsize / 2U); index++)
      {
        tmp = Audio_DigMic2RecBuff[index] / 256;
        tmp = SaturaLH(tmp, -32768, 32767);
        Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
        Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      }
    }
  }

  /* Invoke the 'HalfTransfer' callback function */
  if(hdfsdm_filter == &haudio_in_dfsdm_filter[0])
  {
    Audio_DmaDigMic1RecHalfBuffCplt = 1;
  }
  else
  {
    Audio_DmaDigMic2RecHalfBuffCplt = 1;
  }

  if (((Audio_DmaDigMic1RecHalfBuffCplt != 0U) && (Audio_DmaDigMic2RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)) ||
      ((Audio_DmaDigMic1RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)) ||
      ((Audio_DmaDigMic2RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    BSP_AUDIO_IN_HalfTransfer_CallBack(0);
    Audio_DmaDigMic1RecHalfBuffCplt = 0;
    Audio_DmaDigMic2RecHalfBuffCplt = 0;
  }
}

/**
  * @brief  DFSDM filter error callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
void HAL_DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  UNUSED(hdfsdm_filter);

  BSP_AUDIO_IN_Error_CallBack(0);
}
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_Private_Functions STM32L475E_IOT01 AUDIO_OUT Private Functions
  * @{
  */


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
HAL_StatusTypeDef MX_TIMx_Init(TIM_HandleTypeDef htim)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};


  htim.Instance = AUDIO_DAC_TIM_SOURCE;  // voir ...audio.conf
  htim.Init.Prescaler = 0;
  htim.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim.Init.Period = 1666;
  htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim) != HAL_OK)
  {
    return HAL_ERROR;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig) != HAL_OK)
  {
	    return HAL_ERROR;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
  {
	    return HAL_ERROR;
  }
  return HAL_OK;
}




/**
  * @brief  DAC1 clock Config.
  * @param  hsai SAI handle.
  * @param  SampleRate Audio sample rate used to play the audio stream.
  * @note   The DAC1 configuration utilise le TIM2 pour déclencher les conversions.
  * 		Utilise TIM2_config.
  * @retval HAL status.
  */

HAL_StatusTypeDef MX_DAC1_ClockConfig(DAC_HandleTypeDef *hdac, uint32_t SampleRate)
{

	  /* Prevent unused argument(s) compilation warning */
	  UNUSED(hdac);

	  /* Set the TIMx configuration according to the audio frequency */
	  if((SampleRate == AUDIO_FREQUENCY_11K) || (SampleRate == AUDIO_FREQUENCY_22K) || (SampleRate == AUDIO_FREQUENCY_44K) ||
	     (SampleRate == AUDIO_FREQUENCY_88K) || (SampleRate == AUDIO_FREQUENCY_176K))
	  {
		  return HAL_ERROR;
	  }
	  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_32K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K or AUDIO_FREQUENCY_192K */
	  {
	    // TIMx clock config
		  if (MX_TIMx_Init(htim) != HAL_OK) return HAL_ERROR ;
	  }

	  return HAL_OK;
};

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
HAL_StatusTypeDef MX_DAC1_Init(DAC_HandleTypeDef* hdac)
{


  DAC_ChannelConfTypeDef sConfig = {0};

  /** DAC Initialization
  */
  hdac->Instance = DAC1;
  if (HAL_DAC_Init(hdac) != HAL_OK)
  {
	  return HAL_ERROR;
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
	  return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_Transmit_DMA(DAC_HandleTypeDef* hdac, uint16_t* pData, uint16_t NbrOfDMAData)
{
	//TODO
	return HAL_OK;
};

HAL_StatusTypeDef HAL_DAC_DMAPause(DAC_HandleTypeDef* hdac)
{
	//TODO
	return HAL_OK;

};

HAL_StatusTypeDef HAL_DAC_DMAResume(DAC_HandleTypeDef* hdac)
{
	//TODO
	return HAL_OK;

};

HAL_StatusTypeDef HAL_DAC_DMAStop(DAC_HandleTypeDef* hdac)
{
	//TODO
	return HAL_OK;

};
HAL_StatusTypeDef HAL_DAC_Mute(DAC_HandleTypeDef* hdac)
{
	//TODO
	return HAL_OK;

};
HAL_StatusTypeDef HAL_DAC_UnMute(DAC_HandleTypeDef* hdac)
{
	//TODO
	return HAL_OK;

};


/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
__weak void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
