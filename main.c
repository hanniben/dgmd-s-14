/**
  ******************************************************************************
  * @file    main.c
  * @author  Central LAB
  * @version V3.1.0
  * @date    14-July-2017
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/**
 * @mainpage FP-SNS-ALLMEMS1 Bluetooth Low Energy and Sensors Software
 *
 * @image html st_logo.png
 *
 * <b>Introduction</b>
 *
 * This firmware package includes Components Device Drivers, Board Support Package
 * and example application for the following STMicroelectronics elements:
 * - X-NUCLEO-IDB04A1/X-NUCLEO-IDB05A1 Bluetooth Low energy expansion boards
 * - X-NUCLEO-IKS01A1 Expansion board for four MEMS sensor devices:
 *       HTS221, LPS25H, LSM6DS0, LSM6DS3, LIS3MDL
 * - X-NUCLEO-IKS01A2 Expansion board for four MEMS sensor devices:
 *       HTS221, LPS22HB, LSM6DSL, LSM303AGR
 * - X-NUCLEO-CCA02M1 Digital MEMS microphones expansion board
 * - NUCLEO-F446RE NUCLEO-F401RE NUCLEO-L476RG Nucleo boards
 * - STEVAL-STLKT01V1 (SensorTile) evaluation board that contains the following components:
 *     - MEMS sensor devices: HTS221, LPS22HB, LSM303, LSM6DSM
 *     - digital microphone: MP34DT04
 *     - Gas Gauge IC with alarm output: STC3115
 * - STEVAL-BCNKT01V1 (BlueCoin) evaluation board that contains the following components:
 *     - MEMS sensor devices: LPS22HB, LSM303AGR, LSM6DSM
 *     - digital microphone: 4 x MP34DT04-C1
 * - The MotionFX (iNEMOEngine PRO) suite uses advanced algorithms to integrate outputs
 *   from multiple MEMS sensors in a smartway, independent of environmental conditions,
 *   to reach optimal performance. Real-time motion-sensor data fusion is set to significantly
 *   improve the user experience, increasing accuracy, resolution, stability and response time.
 * - MotionAR (iNEMOEngine PRO) software provides real-time activity recognition data using MEMS accelerometer sensor
 * - MotionCP (iNEMOEngine PRO) software provides carry Position recognition data using MEMS accelerometer sensor
 *   (feature not available on NUCLEO-F446RE, NUCLEO-F401RE and STEVAL-BCNKT01V1)
 * - MotionGR (iNEMOEngine PRO) software provides carry Gesture recognition data using MEMS accelerometer sensor 
 *   (feature not available on NUCLEO-F446RE, NUCLEO-F401RE and STEVAL-BCNKT01V1)
 * - AcousticSL software provides real-time audio source localization using PCM signal audio
 *   (feature not available on the STEVAL-STLKT01V1)
 * - AcousticBF software provides real-time beam forming software, using the audio signals acquired from two digital 
 *   MEMS microphones, it creates a virtual directional microphone pointing to a fixed direction in space 
 *   (feature not available on STEVAL-STLKT01V1 and NUCLEO-L476RG).
 * - BlueVoiceADPCM software enables real-time half-duplex voice-over-Bluetooth low energy communication profile.
 *   It includes one characteristic for audio transmission and one for synchronization and it is responsible for audio encoding and periodical data
 *   transmission on Server side and for decoding of received voice data on Client side
 * - USB device library (for only STEVAL-STLCS01V1) provides support of multi packet transfer to allow
 *   sending big amount of data without split them into max packet size transfers.
 *
 * @attention
 * <b>Important Hardware Additional Information</b><br>
 * <br>\image html X-NUCLEO-IKS01A1_HW_Info.jpg "Figure 1: X-NUCLEO-IKS01A1 expansion board"
 * <br>Before to connect X-NUCLEO-IKS01A1 with X-NUCLEO-CCA02M1 expansion board through the Arduino UNO R3 extension connector,
 * remove the 0-ohm resistors SB25, SB26 and SB27 onto X-NUCLEO-IKS01A1 board, as the above figure 1 shows.<br>
 * <br>\image html X-NUCLEO-IKS01A2_HW_Info.jpg "Figure 2: X-NUCLEO-IKS01A2 expansion board"
 * <br>Before to connect X-NUCLEO-IKS01A2 with X-NUCLEO-CCA02M1 expansion board through the Arduino UNO R3 extension connector,
 * on to X-NUCLEO-IKS01A2 board remove these 0-ohm resistor:
 * - For F4xx STM32 Nucleo motherboard remove SB25, SB26 and SB27
 * - For L4 STM32 Nucleo motherboard remove SB25 if additional microphones are plugged on to X-NUCLEO-CCA02M1 board.<br>
 * .
 * <br>\image html X-NUCLEO-CCA02M1_HW_Info.jpg "Figure 3: X-NUCLEO-CCA02M1 expansion board"
 * <br>For only L4 STM32 Nucleo motherboard, before to connect the board X-NUCLEO-CCA02M1 with the STM32 L4 Nucleo motherboard through the Morpho connector layout,
 * on to X-NUCLEO-CCA02M1 board:
 * - close the solder bridges SB12, SB16 and open the solder bridges SB7, SB15 and SB17
 * - if additional microphones are plugged, close the solder bridge SB17.<br>
 *
 * <b>Example Application</b>
 *
 * The Example application initizializes all the Components and Library creating 3 Custom Bluetooth services:
 * - The first service exposes all the HW and SW characteristics:
 *  - HW characteristics:
 *      - related to MEMS sensor devices: Temperature, Humidity, Pressure, Magnetometer, Gyroscope and Accelleromenter 
 *        and Microphones Signal Noise dB level.
 *      - battery alarm output (for only SensorTile) 
 *  - SW characteristics: the quaternions generated by the MotionFX library in short precision, the activity
 *    recognized using the MotionAR algorithm, the carry position recognized using the MotionCP algorithm (excluding STM32 Nucleo F4xx),
 *    the Gesture recognized using the MotionGR algorithm(excluding STM32 Nucleo F4xx), the audio source localization using the AcousticSL software (excluding STEVAL-STLKT01V1)
 *    that provides real-time audio source localization, AcousticBF software (Excluding STEVAL-STLKT01V1 and STM32 Nucleo L476) provides real-time beam forming
 *    using the audio signals acquired from two digital MEMS microphones.
 *    It uses BlueVoiceADPCM software for real-time half-duplex voice-over-Bluetooth low energy communication profile.
 * - The second Service exposes the console services where we have stdin/stdout and stderr capabilities
 * - The last Service is used for configuration purpose
 *
 * For NUCLEO boards the example application allows the user to control the initialization phase via UART.
 * Launch a terminal application and set the UART port to 460800 bps, 8 bit, No Parity, 1 stop bit.
 * For having the same UART functionality on SensorTile board, is necessary to recompile the code uncomment the line 87
 *	//#define ALLMEMS1_ENABLE_PRINTF
 * on file:
 *	Projects\Multi\Applications\ALLMEMS1\Inc\ALLMEMS1_config.h file
 * This enables the UART that starts with a delay of 10 Seconds for allowing the time to open the UART for looking
 * the initialization phase.
 *
 * This example must be used with the related BlueMS Android/iOS application available on Play/itune store (Version 3.5.0 or higher),
 * in order to read the sent information by Bluetooth Low Energy protocol
 *
 *                              -----------------------
 *                              | VERY IMPORTANT (1): |
 *                              -----------------------
 * 1) This example support the Firmware-Over-The-Air (FOTA) update using the BlueMS Android/iOS application (Version 3.0.0 and above)
 * The FOTA does not work when using X-NUCLEO-IDB04A1
 *
 * 2) This example must run starting at address 0x08004000 in memory and works ONLY if the BootLoader 
 * is saved at the beginning of the FLASH (address 0x08000000)
 *
 * 3) If the user presses the blue user button on Nucleo board, 3 times on less that 2 seconds,
 * he forces a new Calibration for MotionFX Library
 * The calibration value could be stored on FLASH memory or in RAM for avoiding to do the calibration at each board reset
 *
 * 4) For each IDE (IAR/µVision/System Workbench), and for each platform (NUCLEO-F446RE/NUCLEO-F401RE/NUCLEO-L476RG/SensorTile),
 * there are some scripts *.bat and *.sh that makes the following operations:
 * - Full Flash Erase
 * - Load the BootLoader on the rigth flash region
 * - Load the Program (after the compilation) on the rigth flash region (This could be used for a FOTA)
 * - Dump back one single binary that contain BootLoader+Program that could be
 *   flashed at the flash beginning (address 0x08000000) (This COULD BE NOT used for FOTA)
 * - Reset the board
 * .
 *
 *                              -----------------------
 *                              | VERY IMPORTANT (2): |
 *                              -----------------------
 * It's necessary to choose the right Target configuration during the compilation.
 * If the code is compiled for IKS01A1 could not run if it's attached the X-NUCLEO-IKS01A2 and vice versa
 *
 *
 *                                   ----------
 *                                   | ISSUE: |
 *                                   ----------
 * FOTA is not available on NUCLEO-F401RE and NUCLEO-F446RE using binary generated by System Workbench for STM32
 * (due to flash size constraints).
 *
 *                               --------------------
 *                               | KNOWN LIMITATION |
 *                               --------------------
 * - Even if FP-SNS-ALLMEMS1 send 100quaternions/second with Bluetooth, the mobile devices could render only 60frames/second
 * - FOTA does not work when using X-NUCLEO-IDB04A1
 * - For NUCLEO-F446RE/NUCLEO-F401RE board, there is an hardware conflict between the boards X-NUCLEO-IKS01A2 and the X-NUCLEO-CCA02M1.
 *   The hardware features of the LSM6DSL are disabled.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>

//#include "stm32F4xx_hal.h"

#include <limits.h>
#include "TargetFeatures.h"
#include "main.h"
#include "OTA.h"
#include "MetaDataManager.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "HWAdvanceFeatures.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define BLUEMSYS_N_BUTTON_PRESS 3
#define BLUEMSYS_CHECK_CALIBRATION ((uint32_t)0x12345678)

/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;
extern int connected;

/* Code for MotionAR integration - Start Section */
extern MAR_output_t ActivityCode;
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
extern MCP_output_t CarryPositionCode;
#endif /* ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONGR
extern MGR_output_t GestureRecognitionCode;
#endif /* ALLMEMS1_MOTIONGR */

#ifdef STM32_SENSORTILE
  #ifdef ALLMEMS1_ENABLE_PRINTF
    extern TIM_HandleTypeDef  TimHandle;
    extern void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
  #endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/* Exported Variables -------------------------------------------------------------*/

float sensitivity;
/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

MFX_MagCal_output_t magOffset;
SensorAxes_t MAG_Offset; 

uint32_t ConnectionBleStatus  =0;

uint32_t ForceReCalibration    =0;
uint32_t FirstConnectionConfig =0;

uint8_t BufferToWrite[256];
int32_t BytesToWrite;

TIM_HandleTypeDef    TimCCHandle;
TIM_HandleTypeDef    TimEnvHandle;
TIM_HandleTypeDef    TimAudioDataHandle;

uint8_t bdaddr[6];

uint32_t uhCCR4_Val = DEFAULT_uhCCR4_Val;

uint32_t CalibrationData[30];

int32_t ReadPress;
int32_t PrevPress=0;
int32_t HighPress=0;
int32_t LowPress=0;
int16_t SqueezeID=0;
int16_t PinchID=0;
int16_t ExType=0;

/* Table with All the known Meta Data */
MDM_knownGMD_t known_MetaData[]={
  {GMD_CALIBRATION,(sizeof(CalibrationData))},
  {GMD_END    ,0}/* THIS MUST BE THE LAST ONE */
};

static CRC_HandleTypeDef hcrc;

extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];

#ifdef USE_STM32F4XX_NUCLEO
extern uint16_t PDM_Buffer[];
#endif /* USE_STM32F4XX_NUCLEO */
extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];

#ifdef USE_STM32F4XX_NUCLEO
extern uint16_t PDM_Buffer[];
#endif /* USE_STM32F4XX_NUCLEO */

/* Private variables ---------------------------------------------------------*/
static volatile int ButtonPressed        =0;
static volatile int MEMSInterrupt        =0;
static volatile uint32_t HCI_ProcessEvent=0;
static volatile uint32_t SendEnv         =0;
static volatile uint32_t SendAudioLevel  =0;
static volatile uint32_t SendAccGyroMag  =0;

#ifdef STM32_SENSORTILE
static volatile uint32_t SendBatteryInfo =0;
#endif /* STM32_SENSORTILE */

/* Code for MotionFX integration - Start Section */
static volatile uint32_t Quaternion      =0;
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
static volatile uint32_t UpdateMotionAR  =0;
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
static volatile uint32_t UpdateMotionCP  =0;
#endif /* ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONGR
static volatile uint32_t UpdateMotionGR  =0;
#endif /* ALLMEMS1_MOTIONGR */

#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
static volatile uint32_t SendAudioSourceLocalization=0;

extern volatile int32_t SourceLocationToSend;
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */

unsigned char isCal = 0;
static uint32_t mag_time_stamp = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);

static unsigned char ResetCalibrationInMemory(void);

static void MX_CRC_Init(void);

static void InitTimers(void);
static void SendEnvironmentalData(void);
static void MEMSCallback(void);
static void MagCalibTest(void);
static void ReCalibration(void);
static void ButtonCallback(void);
static void SendMotionData(void);
static void SendAudioLevelData(void);

void PressureSensing(int32_t ReadPress, int32_t *PrevPress, int32_t *LowPress, int32_t *HighPress, int16_t *SqueezeID, int16_t *PinchID, int16_t *ExType);


//void SW_BV_send_Callback(void);

#ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING
extern volatile uint8_t BF_toggleFlag;
#endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */

#ifdef STM32_SENSORTILE
static void SendBatteryInfoData(void);
#endif /* STM32_SENSORTILE */

void AudioProcess_DB_Noise(void);

/* Code for MotionFX integration - Start Section */
static void ComputeQuaternions(void);
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
static void ComputeMotionAR(void);
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
static void ComputeMotionCP(void);
#endif /* ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONGR
static void ComputeMotionGR(void);
#endif /* ALLMEMS1_MOTIONGR */

#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
static void SendAudioSourceLocalizationData(void);
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */

uint32_t t_coin=0;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();
      
#ifdef STM32_NUCLEO
  InitTargetPlatform(TARGET_NUCLEO);
#elif STM32_SENSORTILE
  InitTargetPlatform(TARGET_SENSORTILE);
#elif STM32_BLUECOIN
  InitTargetPlatform(TARGET_BLUECOIN);
  t_coin=HAL_GetTick();
  BSP_LED_On(LED2);
  HAL_Delay(800);
  BSP_LED_Off(LED2);

  /* Check if the system has resumed from IWDG reset */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    /* Clear reset flags */
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }
#endif /* STM32_NUCLEO */
  
  /* For enabling CRC clock for using motion libraries (for checking if STM32 microprocessor is used)*/
  MX_CRC_Init();
  
  /* Check the MetaDataManager */
 InitMetaDataManager((void *)&known_MetaData,MDM_DATA_TYPE_GMD,NULL); 
  
  ALLMEMS1_PRINTF("\n\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"

#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (openstm32)\r\n"
#endif
         "\tSend Every %4dmS %d Short precision Quaternions\r\n"
         "\tSend Every %4dmS Temperature/Humidity/Pressure\r\n"
         "\tSend Every %4dmS Acc/Gyro/Magneto\r\n"
         "\tSend Every %4dmS dB noise\r\n\n",
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__,
         QUAT_UPDATE_MUL_10MS*10,SEND_N_QUATERNIONS,
         ENV_UPDATE_MUL_100MS * 100,
         DEFAULT_uhCCR4_Val/10,
         MICS_DB_UPDATE_MUL_10MS * 10);

#ifdef ALLMEMS1_DEBUG_CONNECTION
  ALLMEMS1_PRINTF("Debug Connection         Enabled\r\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */

#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
  ALLMEMS1_PRINTF("Debug Notify Trasmission Enabled\r\n\n");
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */

  /* Initialize the BlueNRG */
  Init_BlueNRG_Stack();

  /* Initialize the BlueNRG Custom services */
  Init_BlueNRG_Custom_Services();  

  if(TargetBoardFeatures.HWAdvanceFeatures) {
    InitHWFeatures();
  }
  
  /* Check the BootLoader Compliance */
  ALLMEMS1_PRINTF("\r\n");
  if(CheckBootLoaderCompliance()) {
    ALLMEMS1_PRINTF("BootLoader Compliant with FOTA procedure\r\n\n");
  } else {
    ALLMEMS1_PRINTF("ERROR: BootLoader NOT Compliant with FOTA procedure\r\n\n");
  }

  /* Set Accelerometer Full Scale to 2G */
  Set2GAccelerometerFullScale();

  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);

  /* initialize timers */
  InitTimers();

  /* Infinite loop */
  while (1){
    /* Led Blinking when there is not a client connected */
    if(!connected)
    {
      if(!TargetBoardFeatures.LedStatus) {
        if(!(HAL_GetTick()&0x3FF)) {
          LedOnTargetPlatform();
        }
      } else {
        if(!(HAL_GetTick()&0x3F)) {
          LedOffTargetPlatform();
        }
      }
    }

    if(set_connectable){     
      /* Code for MotionFX integration - Start Section */
      /* Initialize MotionFX library */ 
      if(TargetBoardFeatures.MotionFXIsInitalized==0)
      {
        MotionFX_manager_init();
        MotionFX_manager_start_9X();
        /* Enable magnetometer calibration */
        MagCalibTest();
      }
      /* Code for MotionFX integration - End Section */
      
      /* Code for MotionAR integration - Start Section */ 
      /* Initialize MotionAR Library */ 
      if(TargetBoardFeatures.MotionARIsInitalized==0)
        MotionAR_manager_init();
      /* Code for MotionAR integration - End Section */
      
      #ifdef ALLMEMS1_MOTIONCP
      /* Initialize MotionCP Library */ 
      if(TargetBoardFeatures.MotionCPIsInitalized==0)
        MotionCP_manager_init();
      #endif /* ALLMEMS1_MOTIONCP */

      #ifdef ALLMEMS1_MOTIONGR
      /* Initialize MotionGR Library */
      if(TargetBoardFeatures.MotionGRIsInitalized==0)
        MotionGR_manager_init();
      #endif /* ALLMEMS1_MOTIONGR */
      
#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
      /* Initialize AcousticSL Library */
      if(TargetBoardFeatures.AcousticSLIsInitalized==0)
        AcousticSL_Manager_init();
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */
      
#ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING
      /* Initialize AcousticBF Library */
        if(TargetBoardFeatures.AcousticBFIsInitalized==0){
          AcousticBF_Manager_init();
        }
#endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */
      
      /* Code for BlueVoice integration - Start Section */
      /* Initialize BlueVoice Library */
        if(TargetBoardFeatures.AudioBVIsInitalized==0)
          AudioBV_Manager_init();
      /* Code for BlueVoice integration - End Section */

      if(NecessityToSaveMetaDataManager) {
        uint32_t Success = EraseMetaDataManager();
        if(Success) {
          SaveMetaDataManager();
        }
      }

      /* Now update the BLE advertize data and make the Board connectable */
      setConnectable();
      set_connectable = FALSE;
      
#ifdef USE_STM32F4XX_NUCLEO
        //BSP_AUDIO_IN_Record(PDM_Buffer, 0);
#endif /* USE_STM32F4XX_NUCLEO */

    }
    
    /* Handle Interrupt from MEMS */
    if(MEMSInterrupt) {
      MEMSCallback();
      MEMSInterrupt=0;
    }

    /* Handle user button */
    if(ButtonPressed) {
      ButtonCallback();
      ButtonPressed=0;       
    }

    /* Handle Re-Calibration */
    if(ForceReCalibration) {
      ForceReCalibration=0;
      ReCalibration();
    }

    /* handle BLE event */
    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      HCI_Process();
    }

    /* Environmental Data */
    if(SendEnv) {
      SendEnv=0;
      SendEnvironmentalData();
    }
    
    /* Mic Data */
    if (SendAudioLevel) {
      SendAudioLevel = 0;
      SendAudioLevelData();
    }

#ifdef STM32_SENSORTILE    
    /* Battery Info Data */
    if(SendBatteryInfo) {
      SendBatteryInfo=0;
      SendBatteryInfoData();
    }
#endif /* STM32_SENSORTILE */

    /* Motion Data */
    if(SendAccGyroMag) {
      SendAccGyroMag=0;
      SendMotionData();
    }

    /* Code for MotionFX integration - Start Section */
    if(Quaternion) {
      Quaternion=0;
      ComputeQuaternions();
    }
    /* Code for MotionFX integration - End Section */

    /* Code for MotionAR integration - Start Section */
    if(UpdateMotionAR) {
      UpdateMotionAR=0;
      ComputeMotionAR();
    }
    /* Code for MotionAR integration - End Section */
    
    #ifdef ALLMEMS1_MOTIONCP
    if(UpdateMotionCP) {
      UpdateMotionCP=0;
      ComputeMotionCP();
    }
    #endif /* ALLMEMS1_MOTIONCP */

    #ifdef ALLMEMS1_MOTIONGR
    if(UpdateMotionGR) {
      UpdateMotionGR=0;
      ComputeMotionGR();
    }
    #endif /* ALLMEMS1_MOTIONGR */
    
#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
    /* Audio Source Localization Data */
    if (SendAudioSourceLocalization)
    {
      SendAudioSourceLocalization = 0;
      SendAudioSourceLocalizationData();
    }
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */ 

    /* Wait for Event */
    __WFI();
  }
}

/**
  * @brief  This function sets the ACC FS to 2g
  * @param  None
  * @retval None
  */
void Set2GAccelerometerFullScale(void)
{
  /* Set Full Scale to +/-2g */
  BSP_ACCELERO_Set_FS_Value(TargetBoardFeatures.HandleAccSensor,2.0f);
  
  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity* ((float) FROM_MG_TO_G);
}

/**
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
void Set4GAccelerometerFullScale(void)
{
  
  /* Set Full Scale to +/-4g */
  BSP_ACCELERO_Set_FS_Value(TargetBoardFeatures.HandleAccSensor,4.0f);

  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity* ((float) FROM_MG_TO_G);
}

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  
  /* Code for MotionFX and MotionGR integration - Start Section */
  /* TIM1_CH1 toggling with frequency = 100Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));

    /* Code for MotionFX integration - Start Section */
    if ((W2ST_CHECK_CONNECTION(W2ST_CONNECT_QUAT)) | (W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC))) {
      Quaternion=1;
    }
    /* Code for MotionFX integration - End Section */

    #ifdef ALLMEMS1_MOTIONGR
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_GR)) {
      UpdateMotionGR=1;
    }
    #endif /* ALLMEMS1_MOTIONGR */
  }
  /* Code for MotionFX and MotionGR integration - End Section */

  #ifdef ALLMEMS1_MOTIONCP
  /* TIM1_CH2 toggling with frequency = 50Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CP)) {
      UpdateMotionCP=1;
    }
  }
  #endif /* ALLMEMS1_MOTIONCP */

  /* Code for MotionAR integration - Start Section */
  /* TIM1_CH3 toggling with frequency = 16Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + DEFAULT_uhCCR3_Val));
    
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR)) {
      UpdateMotionAR=1;
    }
  }
  /* Code for MotionAR integration - End Section */

  /* TIM1_CH4 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    SendAccGyroMag=1;
  }
}


/**
  * @brief  Period elapsed callback in non blocking mode for Environmental timer
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&TimEnvHandle)) {
    /* Environmental */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV))
      SendEnv=1;

#ifdef STM32_SENSORTILE    
    /* Battery Info */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT))
      SendBatteryInfo= 1;
#endif /* STM32_SENSORTILE */
    
#ifdef STM32_SENSORTILE
#ifdef ALLMEMS1_ENABLE_PRINTF
    } else if(htim == (&TimHandle)) {
      CDC_TIM_PeriodElapsedCallback(htim);
#endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */
  } else if(htim == (&TimAudioDataHandle)) {
    /* Mic Data */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
      SendAudioLevel=1;
    
#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION    
    /* Audio Source Localization Data */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SL))
      SendAudioSourceLocalization= 1;
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */
  }
}

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  /* Only if connected */
  if(connected) {
    static uint32_t HowManyButtonPress=0;
    static uint32_t tickstart=0;
    uint32_t tickstop;

    if(!tickstart)
      tickstart = HAL_GetTick();

    tickstop = HAL_GetTick();

    if((tickstop-tickstart)>2000) {
      HowManyButtonPress=0;
      tickstart=tickstop;
    }

    if(TargetBoardFeatures.MotionFXIsInitalized)
    {
      if((HowManyButtonPress+1)==BLUEMSYS_N_BUTTON_PRESS)
      {
        ForceReCalibration=1;
        HowManyButtonPress=0;
      }
      else
      {
        HowManyButtonPress++;
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
        {
           BytesToWrite = sprintf((char *)BufferToWrite, "%ld in %ldmS Reset Calib\r\n",3-HowManyButtonPress,2000-(tickstop-tickstart));
           Term_Update(BufferToWrite,BytesToWrite);
        }
        else
        {
          ALLMEMS1_PRINTF("%ld in %ldmS Reset Calib\r\n",3-HowManyButtonPress,2000-(tickstop-tickstart));
        }
      }
    }
    else
    {
      ALLMEMS1_PRINTF("UserButton Pressed\r\n");
    }
  }
}

/**
  * @brief  Reset the magneto calibration 
  * @param  None
  * @retval None
  */
static void ReCalibration(void)
{
  /* Only if connected */
  if(connected) {
    /* Reset the Compass Calibration */
    isCal=0;
    MFX_MagCal_output_t mag_cal_test;

    /* Notifications of Compass Calibration */
    Config_Notify(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);
    Config_Notify(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);

    /* Reset the Calibration */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite, "\nForce ReCalibration\n\r");
       Term_Update(BufferToWrite,BytesToWrite);
    } else
      ALLMEMS1_PRINTF("\nForce ReCalibration\n\r");
    {
       ResetCalibrationInMemory();
    }

    /* Enable magnetometer calibration */
    MotionFX_manager_MagCal_start(SAMPLE_PERIOD);
    MotionFX_MagCal_getParams(&mag_cal_test);

    /* Switch off the LED */    
    LedOffTargetPlatform();
  }
}

/**
  * @brief  Send Notification where there is a interrupt from MEMS
  * @param  None
  * @retval None
  */
static void MEMSCallback(void)
{
  uint8_t stat = 0;

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) {
    /* Check if the interrupt is due to Free Fall */
    BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_FREE_FALL);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) {
    /* Check if the interrupt is due to Double Tap */
    BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_DOUBLE_TAP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) {
    /* Check if the interrupt is due to Single Tap */
    BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_SINGLE_TAP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    /* Check if the interrupt is due to Wake Up */
    BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_WAKE_UP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) {
    /* Check if the interrupt is due to Tilt */
    BSP_ACCELERO_Get_Tilt_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_TILT);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) {
    /* Check if the interrupt is due to 6D Orientation */
    BSP_ACCELERO_Get_6D_Orientation_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEventType Orientation = GetHWOrientation6D();
      AccEvent_Notify(Orientation);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
    /* Check if the interrupt is due to Pedometer */
    BSP_ACCELERO_Get_Pedometer_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      uint16_t StepCount = GetStepHWPedometer();
      AccEvent_Notify(StepCount);
    }
  }
}

/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{
  SensorAxes_t ACC_Value;
  SensorAxes_t GYR_Value;
  SensorAxes_t MAG_Value;

  /* Read the Acc values */
  BSP_ACCELERO_Get_Axes(TargetBoardFeatures.HandleAccSensor,&ACC_Value);

  /* Read the Magneto values */
  BSP_MAGNETO_Get_Axes(TargetBoardFeatures.HandleMagSensor,&MAG_Value);

  /* Read the Gyro values */
  BSP_GYRO_Get_Axes(TargetBoardFeatures.HandleGyroSensor,&GYR_Value);
  
  #ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION

    int16_t acc_x_to_send, acc_y_to_send, acc_z_to_send;
    int16_t gyr_x_to_send = 0, gyr_y_to_send = 0, gyr_z_to_send = 0;
    int16_t mag_x_to_send, mag_y_to_send, mag_z_to_send;

    acc_x_to_send = ACC_Value.AXIS_X;
    acc_y_to_send = ACC_Value.AXIS_Y;
    acc_z_to_send = ACC_Value.AXIS_Z;
    ALLMEMS1_PRINTF("ACC_X=%d ",acc_x_to_send);
    ALLMEMS1_PRINTF("ACC_Y=%d ",acc_y_to_send);
    ALLMEMS1_PRINTF("ACC_Z=%d ",acc_z_to_send);

    gyr_x_to_send = GYR_Value.AXIS_X/100;
    gyr_y_to_send = GYR_Value.AXIS_Y/100;
    gyr_z_to_send = GYR_Value.AXIS_Z/100;
    ALLMEMS1_PRINTF("GYR_X=%d ",gyr_x_to_send);
    ALLMEMS1_PRINTF("GYR_Y=%d ",gyr_y_to_send);
    ALLMEMS1_PRINTF("GYR_Z=%d ",gyr_z_to_send);

    mag_x_to_send = MAG_Value.AXIS_X;
    mag_y_to_send = MAG_Value.AXIS_Y;
    mag_z_to_send = MAG_Value.AXIS_Z;
    ALLMEMS1_PRINTF("MAG_X=%d ",mag_x_to_send);
    ALLMEMS1_PRINTF("MAG_Y=%d ",mag_y_to_send);
    ALLMEMS1_PRINTF("MAG_Z=%d ",mag_z_to_send);

    ALLMEMS1_PRINTF("\r\n");
  #endif /* ALLMEMS1_DEBUG_NOTIFY_TRANSMISSION */

  AccGyroMag_Update(&ACC_Value,&GYR_Value,&MAG_Value);
}

/* Code for MotionFX integration - Star Section */
/* @brief  MotionFX Working function
 * @param  None
 * @retval None
 */
static void ComputeQuaternions(void)
{
  static SensorAxes_t quat_axes[SEND_N_QUATERNIONS];
  
  static int32_t calibIndex =0;
  static int32_t CounterFX  =0;
  static int32_t CounterEC  =0;
  
  SensorAxesRaw_t ACC_Value_Raw;
  SensorAxes_t GYR_Value;
  SensorAxes_t MAG_Value;

  MFX_MagCal_input_t mag_data_in;

  /* Increment the Counter */
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC)) {
    CounterEC++;
  } else {
    CounterFX++;
  }

  /* Read the Acc RAW values */
  BSP_ACCELERO_Get_AxesRaw(TargetBoardFeatures.HandleAccSensor,&ACC_Value_Raw);

  /* Read the Magneto values */
  BSP_MAGNETO_Get_Axes(TargetBoardFeatures.HandleMagSensor,&MAG_Value);

  /* Read the Gyro values */
  BSP_GYRO_Get_Axes(TargetBoardFeatures.HandleGyroSensor,&GYR_Value);
      
  /* Check if is calibrated */
  if(isCal!=0x01){
    /* Run Compass Calibration @ 25Hz */
    calibIndex++;
    if (calibIndex == 4){
      calibIndex = 0;

      mag_data_in.mag[0]= MAG_Value.AXIS_X * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[1]= MAG_Value.AXIS_Y * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[2]= MAG_Value.AXIS_Z * FROM_MGAUSS_TO_UT50;
      mag_data_in.time_stamp = mag_time_stamp;
      mag_time_stamp += SAMPLE_PERIOD;
      MotionFX_manager_MagCal_run(&mag_data_in, &magOffset);
      
      /* Control the calibration status */
      if( (magOffset.cal_quality == MFX_MAGCALOK) ||
          (magOffset.cal_quality == MFX_MAGCALGOOD) )
      {
        isCal= 1;
        
        MAG_Offset.AXIS_X= (int32_t)(magOffset.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.AXIS_Y= (int32_t)(magOffset.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.AXIS_Z= (int32_t)(magOffset.hi_bias[2] * FROM_UT50_TO_MGAUSS);
        
        /* Disable magnetometer calibration */
        MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);
      }

      if(isCal == 0x01){
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite, "Compass Calibrated\n\r");
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          ALLMEMS1_PRINTF("Compass Calibrated\n\r");
        }

        /* Switch on the Led */
        LedOnTargetPlatform();

        /* Notifications of Compass Calibration */
        Config_Notify(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);
        Config_Notify(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);
      }
    }
  }else {
    calibIndex=0;
  }

  MotionFX_manager_run(ACC_Value_Raw,GYR_Value,MAG_Value);

  /* Read the quaternions */
  MFX_output_t *MotionFX_Engine_Out = MotionFX_manager_getDataOUT();

  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC)) {
    /* E-Compass Updated every 0.1 Seconds*/
    if(CounterEC==10) {
      uint16_t Angle = (uint16_t)trunc(100*MotionFX_Engine_Out->heading_9X);
      CounterEC=0;
      ECompass_Update(Angle);
    }
  } else {
    int32_t QuaternionNumber = (CounterFX>SEND_N_QUATERNIONS) ? (SEND_N_QUATERNIONS-1) : (CounterFX-1);

    /* Scaling quaternions data by a factor of 10000
      (Scale factor to handle float during data transfer BT) */

    /* Save the quaternions values */
    if(MotionFX_Engine_Out->quaternion_9X[3] < 0){
      quat_axes[QuaternionNumber].AXIS_X = (int32_t)(MotionFX_Engine_Out->quaternion_9X[0] * (-10000));
      quat_axes[QuaternionNumber].AXIS_Y = (int32_t)(MotionFX_Engine_Out->quaternion_9X[1] * (-10000));
      quat_axes[QuaternionNumber].AXIS_Z = (int32_t)(MotionFX_Engine_Out->quaternion_9X[2] * (-10000));
    } else {
      quat_axes[QuaternionNumber].AXIS_X = (int32_t)(MotionFX_Engine_Out->quaternion_9X[0] * 10000);
      quat_axes[QuaternionNumber].AXIS_Y = (int32_t)(MotionFX_Engine_Out->quaternion_9X[1] * 10000);
      quat_axes[QuaternionNumber].AXIS_Z = (int32_t)(MotionFX_Engine_Out->quaternion_9X[2] * 10000);
    }
      
    /* Every QUAT_UPDATE_MUL_10MS*10 mSeconds Send Quaternions informations via bluetooth */
    if(CounterFX==QUAT_UPDATE_MUL_10MS){
      Quat_Update(quat_axes);
      CounterFX=0;
    }
  }
}
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
/**
  * @brief  MotionAR Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionAR(void)
{
  static MAR_output_t ActivityCodeStored = MAR_NOACTIVITY;
  SensorAxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  BSP_ACCELERO_Get_AxesRaw(TargetBoardFeatures.HandleAccSensor,&ACC_Value_Raw);

  MotionAR_manager_run(ACC_Value_Raw);

  if(ActivityCodeStored!=ActivityCode){
    ActivityCodeStored = ActivityCode;

    ActivityRec_Update(ActivityCode);

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: AR=%d\r\n",ActivityCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: AR=%d\r\n",ActivityCode);
    }
  }
}
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
/**
  * @brief  MotionCP Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionCP(void)
{  
  static MCP_output_t CarryPositionCodeStored = MCP_UNKNOWN;
  SensorAxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  BSP_ACCELERO_Get_AxesRaw(TargetBoardFeatures.HandleAccSensor,&ACC_Value_Raw);
  MotionCP_manager_run(ACC_Value_Raw);

  if(CarryPositionCodeStored!=CarryPositionCode){
    CarryPositionCodeStored = CarryPositionCode;
    CarryPosRec_Update(CarryPositionCode);

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: CP=%d\r\n",CarryPositionCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: CP=%d\r\n",CarryPositionCode);
    }
  }
}
#endif /* ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONGR
/**
  * @brief  MotionGR Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionGR(void)
{
  static MGR_output_t GestureRecognitionCodeStored = MGR_NOGESTURE;
  SensorAxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  BSP_ACCELERO_Get_AxesRaw(TargetBoardFeatures.HandleAccSensor,&ACC_Value_Raw);
  MotionGR_manager_run(ACC_Value_Raw);

  if(GestureRecognitionCodeStored!=GestureRecognitionCode){
    GestureRecognitionCodeStored = GestureRecognitionCode;
    GestureRec_Update(GestureRecognitionCode);

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: GR=%d\r\n",GestureRecognitionCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: GR=%d\r\n",GestureRecognitionCode);
    }
  }
}
#endif /* ALLMEMS1_MOTIONGR */

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess(void)
{
  /*for L4 PDM to PCM conversion is performed in hardware by DFSDM peripheral*/
  
#if defined(USE_STM32F4XX_NUCLEO) || defined(STM32_BLUECOIN)
    
    #ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING
    if(BF_toggleFlag==FALSE)
    #endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */
      {
        BSP_AUDIO_IN_PDMToPCM(PDM_Buffer, PCM_Buffer);
      }
    
#endif /* USE_STM32F4XX_NUCLEO */
  
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
  {
    AudioProcess_DB_Noise();
  }
  
#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SL))
    AudioProcess_SL();
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */
  
  /* Code for BlueVoice integration - Start Section */
  if(((W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_AUDIO))!=0) & ((W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_SYNC))!=0))
  {
    AudioProcess_BV();
  }
  /* Code for BlueVoice integration - End Section */
 
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess_DB_Noise(void)
{
  int32_t i;
  int32_t NumberMic;
  
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL)) {
    for(i = 0; i < 16; i++){
      for(NumberMic=0;NumberMic<AUDIO_CHANNELS;NumberMic++) {
        RMS_Ch[NumberMic] += (float)((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic] * ((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic]));
      }
    }
  }
}

/**
  * @brief  Send Audio Level Data (Ch1) to BLE
  * @param  None
  * @retval None
  */
static void SendAudioLevelData(void)
{
  int32_t NumberMic;
  uint16_t DBNOISE_Value_Ch[AUDIO_CHANNELS];
  
  for(NumberMic=0;NumberMic<(AUDIO_CHANNELS);NumberMic++) {
    DBNOISE_Value_Ch[NumberMic] = 0;

    RMS_Ch[NumberMic] /= (16.0f*MICS_DB_UPDATE_MUL_10MS*10);

    DBNOISE_Value_Ch[NumberMic] = (uint16_t)((120.0f - 20 * log10f(32768 * (1 + 0.25f * (AUDIO_VOLUME_VALUE /*AudioInVolume*/ - 4))) + 10.0f * log10f(RMS_Ch[NumberMic])) * 0.3f + DBNOISE_Value_Old_Ch[NumberMic] * 0.7f);
    DBNOISE_Value_Old_Ch[NumberMic] = DBNOISE_Value_Ch[NumberMic];
    RMS_Ch[NumberMic] = 0.0f;
  }
  
  AudioLevel_Update(DBNOISE_Value_Ch);
}

#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION

/**
  * @brief  Send Audio Source Localization Data to BLE
  * @param  None
  * @retval None
  */
void SendAudioSourceLocalizationData(void)
{
  AudioSourceLocalization_Update(SourceLocationToSend);
 
#ifdef STM32_BLUECOIN
  if(SourceLocationToSend < 23)
    {
      BSP_LED_On(LED1);
      BSP_LED_Off(LED2);
      BSP_LED_Off(LED3);
      BSP_LED_Off(LED4);
      BSP_LED_Off(LED5);
      BSP_LED_Off(LED6);
      BSP_LED_Off(LED7);
      BSP_LED_Off(LED8);
    }
    else if(SourceLocationToSend < 23 + 45)
    {
      BSP_LED_Off(LED1);
      BSP_LED_On(LED2);
      BSP_LED_Off(LED3);
      BSP_LED_Off(LED4);
      BSP_LED_Off(LED5);
      BSP_LED_Off(LED6);
      BSP_LED_Off(LED7);
      BSP_LED_Off(LED8);
    }
    else if(SourceLocationToSend < 23 + 45*2)
    {
      BSP_LED_Off(LED1);
      BSP_LED_Off(LED2);
      BSP_LED_On(LED3);
      BSP_LED_Off(LED4);
      BSP_LED_Off(LED5);
      BSP_LED_Off(LED6);
      BSP_LED_Off(LED7);
      BSP_LED_Off(LED8);  
    }
    else if(SourceLocationToSend < 23 + 45*3)
    {
      BSP_LED_Off(LED1);
      BSP_LED_Off(LED2);
      BSP_LED_Off(LED3);
      BSP_LED_On(LED4);
      BSP_LED_Off(LED5);
      BSP_LED_Off(LED6);
      BSP_LED_Off(LED7);
      BSP_LED_Off(LED8);
    }
    else if(SourceLocationToSend < 23 + 45*4)
    {
      BSP_LED_Off(LED1);
      BSP_LED_Off(LED2);
      BSP_LED_Off(LED3);
      BSP_LED_Off(LED4);
      BSP_LED_On(LED5);
      BSP_LED_Off(LED6);
      BSP_LED_Off(LED7);
      BSP_LED_Off(LED8);
    }
    else if(SourceLocationToSend < 23 + 45*5)
    {
      BSP_LED_Off(LED1);
      BSP_LED_Off(LED2);
      BSP_LED_Off(LED3);
      BSP_LED_Off(LED4);
      BSP_LED_Off(LED5);
      BSP_LED_On(LED6);
      BSP_LED_Off(LED7);
      BSP_LED_Off(LED8);
    }
    else if(SourceLocationToSend < 23 + 45*6)
    {
      BSP_LED_Off(LED1);
      BSP_LED_Off(LED2);
      BSP_LED_Off(LED3);
      BSP_LED_Off(LED4);
      BSP_LED_Off(LED5);
      BSP_LED_Off(LED6);
      BSP_LED_On(LED7);
      BSP_LED_Off(LED8);    
    }
    else if(SourceLocationToSend < 23 + 45*7)
    {
      BSP_LED_Off(LED1);
      BSP_LED_Off(LED2);
      BSP_LED_Off(LED3);
      BSP_LED_Off(LED4);
      BSP_LED_Off(LED5);
      BSP_LED_Off(LED6);
      BSP_LED_Off(LED7);
      BSP_LED_On(LED8);
    }
    else if(SourceLocationToSend < 360)
    {
      BSP_LED_On(LED1);
      BSP_LED_Off(LED2);
      BSP_LED_Off(LED3);
      BSP_LED_Off(LED4);
      BSP_LED_Off(LED5);
      BSP_LED_Off(LED6);
      BSP_LED_Off(LED7);
      BSP_LED_Off(LED8);
    }
#endif /* STM32_BLUECOIN */
}



#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */

/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  AudioProcess();
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  AudioProcess();
}

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
  uint8_t Status;

#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite,"Sending: ");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    //ALLMEMS1_PRINTF("Sending: ");
  }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */

  /* Notifications of Compass Calibration status*/
  if(FirstConnectionConfig) {
    Config_Notify(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);
    Config_Notify(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Cal=%d ",isCal);
       Term_Update(BufferToWrite,BytesToWrite);
     } else {
      //ALLMEMS1_PRINTF("Cal=%d ",isCal);
     }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
    FirstConnectionConfig=0;
    
    /* Switch on/off the LED according to calibration */
    if(isCal){
      LedOnTargetPlatform();
    } else {
      LedOffTargetPlatform();
    }
  }

  /* Pressure,Humidity, and Temperatures*/
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV)) {
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;

    if(TargetBoardFeatures.HandlePressSensor) {
      if(BSP_PRESSURE_IsInitialized(TargetBoardFeatures.HandlePressSensor,&Status)==COMPONENT_OK) {
        BSP_PRESSURE_Get_Press(TargetBoardFeatures.HandlePressSensor,(float *)&SensorValue);
        MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
        PressToSend=intPart*100+decPart;
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite,"Press=%ld ",PressToSend);
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          //ALLMEMS1_PRINTF("Press=%ld ",PressToSend);
        }
        PressureSensing(PressToSend, &PrevPress, &LowPress, &HighPress, &SqueezeID, &PinchID, &ExType);

#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
      }
    }

#ifndef STM32_BLUECOIN
    if(TargetBoardFeatures.HandleHumSensor) {
      if(BSP_HUMIDITY_IsInitialized(TargetBoardFeatures.HandleHumSensor,&Status)==COMPONENT_OK){
        BSP_HUMIDITY_Get_Hum(TargetBoardFeatures.HandleHumSensor,(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        HumToSend = intPart*10+decPart;
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite,"Hum=%d ",HumToSend);
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          ALLMEMS1_PRINTF("Hum=%d ",HumToSend);
        }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
      }
    }
#endif /* STM32_BLUECOIN */

    if(TargetBoardFeatures.NumTempSensors==2) {
      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart; 
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite,"Temp=%d ",Temp1ToSend);
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          //ALLMEMS1_PRINTF("Temp=%d ",Temp1ToSend);
        }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
      }

      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[1],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[1],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp2ToSend = intPart*10+decPart;
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite,"Temp2=%d ",Temp2ToSend);
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          ALLMEMS1_PRINTF("Temp2=%d ",Temp2ToSend);
        }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
      }
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart;
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite,"Temp1=%d ",Temp1ToSend);
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          //ALLMEMS1_PRINTF("Temp1=%d ",Temp1ToSend);
        }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
      }
    }
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  }

#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite,"\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    //ALLMEMS1_PRINTF("\r\n");
  }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
}

void PressureSensing(int32_t PressRead, int32_t *PrevPress, int32_t *LowPress, int32_t *HighPress, int16_t *SqueezeID, int16_t *PinchID, int16_t *ExType)
{
	int32_t zscore=0;

	// Reset exercises when finished
	if ((*SqueezeID >= 10) && (*PinchID >= 10)) {
		*SqueezeID = 0;
		*PinchID = 0;
	}

	// Check if squeeze exercises are done, start pinch
	if (*SqueezeID < 10) {
		*ExType = 0;
		ALLMEMS1_PRINTF("Squeeze test #%d\r\n",*SqueezeID + 1);
	}
	else {
		*ExType = 1;
		ALLMEMS1_PRINTF("Pinch test #%d\r\n",*PinchID + 1);
	}

	// Initialize Low Pressure reading if none exists
	if(*LowPress == 0) {
		*LowPress=PressRead;
	}

	// Check if pressure is rising, update high pressure reading
	if(PressRead > *PrevPress + 50) {
	     *HighPress=PressRead;
	}

	// If pressure is falling, update low pressure reading
	if(PressRead < *PrevPress - 50) {

		 // If high pressure has been previously recorded, compute difference and z-score
	     if (*HighPress != 0) {
	        ALLMEMS1_PRINTF("Press Detected! Change in Pressure=%ld ",*HighPress - *LowPress);
	        if (*ExType == 0) {
	        	zscore = (*HighPress - *LowPress - 5057)/192;
	        	ALLMEMS1_PRINTF("Looking for Squeeze... ");
	        	*SqueezeID = *SqueezeID + 1;
	        }
	        else {
	        	zscore = (*HighPress - *LowPress - 3149)/97;
	        	ALLMEMS1_PRINTF("Looking for Pinch... ");
	        	*PinchID = *PinchID + 1;
	        }
	        ALLMEMS1_PRINTF("Z-Score=%ld\r\n",zscore);
	        *HighPress=0;

	        if ((zscore >= -3) && (zscore <= 3)) {
	        	ALLMEMS1_PRINTF("Perfect!\r\n");
	        }
	        else if ((zscore < -3) && (zscore >= -10)) {
	        	ALLMEMS1_PRINTF("Good, Pressure Low\r\n");
	        }
	        else if ((zscore > 3) && (zscore <= 10)) {
	        	ALLMEMS1_PRINTF("Good, Pressure High\r\n");
	        }
	        else if (zscore < -10) {
	        	ALLMEMS1_PRINTF("Pressure Too Low\r\n");
	        }
	        else if (zscore > 10) {
	        	ALLMEMS1_PRINTF("Pressure Too High\r\n");
	        }
	     }
	     *LowPress=PressRead;
	}

	*PrevPress = PressRead;
}

#ifdef STM32_SENSORTILE
/**
  * @brief  Send Battery Info Data (Voltage/Current/Soc) to BLE
  * @param  None
  * @retval None
  */
static void SendBatteryInfoData(void)
{
  uint32_t voltage, soc;
  int32_t current;
  uint8_t v_mode;
  
  /* Update Gas Gouge Status */
  BSP_GG_Task(TargetBoardFeatures.HandleGGComponent,&v_mode);

  /* Read the Gas Gouge Status */
  BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
  BSP_GG_GetCurrent(TargetBoardFeatures.HandleGGComponent, &current);
  BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);
  
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite,"Battery Report: \r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    ALLMEMS1_PRINTF("Battery Report: \r\n");
  }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */

  /* Battery Informations */
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT)) {
    GG_Update(soc, voltage, current);
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Charge= %ld%% Voltage=%ld mV Current= %ld mA", soc, voltage, current);
       Term_Update(BufferToWrite,BytesToWrite);
     } else {
       ALLMEMS1_PRINTF("Charge= %ld%% Voltage=%ld mV Current= %ld mA", soc, voltage, current);
     }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
  }

#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite,"\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    //ALLMEMS1_PRINTF("\r\n");
  }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
}
#endif /* STM32_SENSORTILE */

/**
  * @brief  CRC init function.
  * @param  None
  * @retval None
  */
static void MX_CRC_Init(void)
{
  hcrc.Instance = CRC;

  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief  Function for initializing timers for sending the information to BLE:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to have TIM4 counter clock equal to 2 KHz */
#if defined(STM32F401xE) || defined (STM32L476xx)
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2000) - 1);
#elif STM32F446xx
  uwPrescalerValue = (uint32_t) (((SystemCoreClock / 2) / 2000) - 1);
#endif /* defined(STM32F401xE) || defined (STM32L476xx) */
  
  /* Set TIM4 instance ( Environmental ) */
  TimEnvHandle.Instance = TIM4;
  /* Initialize TIM4 peripheral */
  TimEnvHandle.Init.Period = ENV_UPDATE_MUL_100MS*200 - 1;
  TimEnvHandle.Init.Prescaler = uwPrescalerValue;
  TimEnvHandle.Init.ClockDivision = 0;
  TimEnvHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimEnvHandle) != HAL_OK) {
    /* Initialization Error */
  }

  /* Compute the prescaler value to have TIM1 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1); 
  
  /* Set TIM1 instance ( Motion ) */
  TimCCHandle.Instance = TIM1;  
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
 /* Configure the Output Compare channels */
 /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Code for MotionFX and MotionGR integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel1 */
  sConfig.Pulse = DEFAULT_uhCCR1_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionFX and MotionGR integration - End Section */
  
  #ifdef ALLMEMS1_MOTIONCP
  /* Output Compare Toggle Mode configuration: Channel2 */
  sConfig.Pulse = DEFAULT_uhCCR2_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  #endif /* ALLMEMS1_MOTIONCP */
  
  /* Code for MotionAR integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel3 */
  sConfig.Pulse = DEFAULT_uhCCR3_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionAR integration - End Section */
  
  /* Output Compare Toggle Mode configuration: Channel4 */
  sConfig.Pulse = DEFAULT_uhCCR4_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
    /* Compute the prescaler value to have TIM5 counter clock equal to 10 KHz */
#if defined(STM32F401xE) || defined (STM32L476xx)
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);
#elif STM32F446xx
  uwPrescalerValue = (uint32_t) (((SystemCoreClock / 2) / 10000) - 1);
#endif /* defined(STM32F401xE) || defined (STM32L476xx) */
  
  /* Set TIM5 instance ( Mic ) */
  TimAudioDataHandle.Instance = TIM5;
  TimAudioDataHandle.Init.Period = MICS_DB_UPDATE_MUL_10MS*100 - 1;
  TimAudioDataHandle.Init.Prescaler = uwPrescalerValue;
  TimAudioDataHandle.Init.ClockDivision = 0;
  TimAudioDataHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimAudioDataHandle) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }
}

/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Stack(void)
{
  const char BoardName[8] = {NAME_BLUEMS,0};
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  hwVersion;
  uint16_t fwVersion;

#ifdef MAC_BLUEMS
  {
    uint8_t tmp_bdaddr[6]= {MAC_BLUEMS};
    int32_t i;
    for(i=0;i<6;i++)
      bdaddr[i] = tmp_bdaddr[i];
  }
#endif /* MAC_BLUEMS */
  
#ifndef STM32_NUCLEO
  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();
#endif /* STM32_NUCLEO */

  /* Initialize the BlueNRG HCI */
  HCI_Init();
    
  /* Reset BlueNRG hardware */
  BlueNRG_RST();

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  if (hwVersion > 0x30) {
    /* X-NUCLEO-IDB05A1 expansion board is used */
    TargetBoardFeatures.bnrg_expansion_board = IDB05A1;
  } else {
    /* X-NUCLEO-IDB0041 expansion board is used */
    TargetBoardFeatures.bnrg_expansion_board = IDB04A1;
  }
  
  /* 
   * Reset BlueNRG again otherwise it will fail.
   */
  BlueNRG_RST();

#ifndef MAC_BLUEMS
  #ifdef MAC_STM32UID_BLUEMS
  /* Create a Unique BLE MAC Related to STM32 UID */
  {
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
#ifdef STM32_NUCLEO
    /* if IDB05A1 = Number between 100->199
     * if IDB04A1 = Number between 0->99
     * where Y == (ALLMEMS1_VERSION_MAJOR + ALLMEMS1_VERSION_MINOR)&0xF */
    bdaddr[4] = (hwVersion > 0x30) ?
         ((((ALLMEMS1_VERSION_MAJOR-48)*10) + (ALLMEMS1_VERSION_MINOR-48)+100)&0xFF) :
         ((((ALLMEMS1_VERSION_MAJOR-48)*10) + (ALLMEMS1_VERSION_MINOR-48)    )&0xFF) ;
#else /* STM32_NUCLEO */
    bdaddr[4] = (((ALLMEMS1_VERSION_MAJOR-48)*10) + (ALLMEMS1_VERSION_MINOR-48)+100)&0xFF;
#endif  /* STM32_NUCLEO */
    bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  }
  #else /* MAC_STM32UID_BLUEMS */
  {
    /* we will let the BLE chip to use its Random MAC address */
    uint8_t data_len_out;
    ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

    if(ret){
      ALLMEMS1_PRINTF("\r\nReading  Random BD_ADDR failed\r\n");
      goto fail;
    }
  }
  #endif /* MAC_STM32UID_BLUEMS */
#else /* MAC_BLUEMS */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);

  if(ret){
     ALLMEMS1_PRINTF("\r\nSetting Pubblic BD_ADDR failed\r\n");
     goto fail;
  }
#endif /* MAC_BLUEMS */

  ret = aci_gatt_init();    
  if(ret){
     ALLMEMS1_PRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }

  if (TargetBoardFeatures.bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

  if(ret != BLE_STATUS_SUCCESS){
     ALLMEMS1_PRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }

#ifndef  MAC_BLUEMS
  #ifdef MAC_STM32UID_BLUEMS
    ret = hci_le_set_random_address(bdaddr);

    if(ret){
       ALLMEMS1_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
       goto fail;
    }
  #endif /* MAC_STM32UID_BLUEMS */
#endif /* MAC_BLUEMS */

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret){
     ALLMEMS1_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) {
     ALLMEMS1_PRINTF("\r\nGAP setting Authentication failed\r\n");
     goto fail;
  }

  ALLMEMS1_PRINTF("SERVER: BLE Stack Initialized \r\n"
         "\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
         "\t\tBoardName= %s\r\n"
         "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
         (TargetBoardFeatures.bnrg_expansion_board==IDB05A1) ? "IDB05A1" : "IDB04A1",
         hwVersion,
         fwVersion>>8,
         (fwVersion>>4)&0xF,
         (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
         BoardName,
         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);

  return;

fail:
  return;
}

/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;
  
  ret = Add_HW_SW_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     ALLMEMS1_PRINTF("HW & SW Service W2ST added successfully\r\n");
  }
  else
  {
     ALLMEMS1_PRINTF("\r\nError while adding HW & SW Service W2ST\r\n");
  }

  ret = Add_ConsoleW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     ALLMEMS1_PRINTF("Console Service W2ST added successfully\r\n");
  }
  else
  {
     ALLMEMS1_PRINTF("\r\nError while adding Console Service W2ST\r\n");
  }

  ret = Add_ConfigW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     ALLMEMS1_PRINTF("Config  Service W2ST added successfully\r\n");
  }
  else
  {
     ALLMEMS1_PRINTF("\r\nError while adding Config Service W2ST\r\n");
  }
}

#ifdef USE_STM32F4XX_NUCLEO
#ifdef STM32_NUCLEO

#ifdef STM32F401xE
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow:
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is
  clocked below the maximum system frequency, to update the voltage scaling value
  regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    Error_Handler();
  }
  
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
    Error_Handler();
  }
}
#elif STM32F446xx
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
  clocked below the maximum system frequency, to update the voltage scaling value 
  regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  #ifdef USE_BLUECOIN   
  RCC_OscInitStruct.PLL.PLLM = 16;
#else
  RCC_OscInitStruct.PLL.PLLM = 8;
#endif
  RCC_OscInitStruct.PLL.PLLN = 336; //192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;//4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Activate the OverDrive to reach the 180 MHz Frequency */  
  //HAL_PWREx_EnableOverDrive();
  
  /*Select Main PLL output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLQ;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}
#endif /* STM32F401xE */

#elif STM32_BLUECOIN
/**
* @brief  System Clock Configuration
*         The system Clock is configured as follow : 
*            System Clock source            = PLL (HSE)
*            SYSCLK(Hz)                     = 168000000
*            HCLK(Hz)                       = 168000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 4
*            APB2 Prescaler                 = 2
*            HSE Frequency(Hz)              = 8000000
*            PLL_M                          = 8
*            PLL_N                          = 336
*            PLL_P                          = 2
*            PLL_Q                          = 7
*            VDD(V)                         = 3.3
*            Main regulator output voltage  = Scale1 mode
*            Flash Latency(WS)              = 5
*         The USB clock configuration from PLLSAI:
*            PLLSAIM                        = 8
*            PLLSAIN                        = 384
*            PLLSAIP                        = 8
* @param  None
* @retval None
*/
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
  clocked below the maximum system frequency, to update the voltage scaling value 
  regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  #ifdef USE_BLUECOIN   
  RCC_OscInitStruct.PLL.PLLM = 16;
#else
  RCC_OscInitStruct.PLL.PLLM = 8;
#endif
  RCC_OscInitStruct.PLL.PLLN = 336;//192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;//4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Activate the OverDrive to reach the 180 MHz Frequency */  
  //HAL_PWREx_EnableOverDrive();
  
  /*Select Main PLL output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLQ;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}
#endif /* STM32_NUCLEO */
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#ifdef STM32_NUCLEO
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;  /* Previous value = RCC_MSIRANGE_6; */
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6; /* Previous value = 1 */
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 4; /* Previous value = 2 */
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
}
#elif STM32_SENSORTILE
/**
* @brief  System Clock Configuration
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  
  /* Enable the LSE Oscilator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }
  
  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
  HAL_RCCEx_DisableLSECSS();
  
  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 6;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  RCC_OscInitStruct.PLL.PLLR            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }
  
  /* Enable MSI Auto-calibration through LSE */
  HAL_RCCEx_EnableMSIPLLMode();
  
  /* Select MSI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK){
    while(1);
  }
}
#endif /* STM32_NUCLEO */
#endif /* USE_STM32L4XX_NUCLEO */

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin){
#ifdef STM32_NUCLEO
    case SPI1_CMN_DEFAULT_IRQ_PIN:
#else
    case BNRG_SPI_EXTI_PIN:
#endif /* STM32_NUCLEO */   
      HCI_Isr();
      HCI_ProcessEvent=1;
    break;
#ifdef STM32_NUCLEO
  case KEY_BUTTON_PIN:
    ButtonPressed = 1;
    break;
#elif STM32_BLUECOIN
    case BUTTON_1_PIN:
      if(HAL_GetTick() - t_coin > 4000)
        BSP_ShutDown(); 
    break;
    case BUTTON_2_PIN:
      ButtonPressed = 1;
    break;
#endif /* STM32_NUCLEO */

#ifdef STM32_NUCLEO
  #ifdef IKS01A1
    case M_INT1_PIN:
  #elif IKS01A2
    case LSM6DSL_INT1_O_PIN:
  #endif /* IKS01A1 */
#elif STM32_SENSORTILE
  case LSM6DSM_INT2_PIN:
#elif STM32_BLUECOIN
  case GPIO_PIN_0:
  case GPIO_PIN_4:
#endif /* STM32_NUCLEO */
    MEMSInterrupt=1;
    break;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: ALLMEMS1_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif

/**
 * @brief  Test if calibration data are available
 * @param  None
 * @retval None
 */
static void MagCalibTest(void)
{
  MFX_MagCal_output_t mag_cal_test;
  
  /* Recall the calibration Credential saved */
  MotionFX_manager_MagCal_start(SAMPLE_PERIOD);
  MotionFX_MagCal_getParams(&mag_cal_test);
    
  if(CalibrationData[0]== BLUEMSYS_CHECK_CALIBRATION) {
#ifdef STM32_NUCLEO
    if(CalibrationData[1] == TargetBoardFeatures.mems_expansion_board) {
#endif /* STM32_NUCLEO */
      
      if( (mag_cal_test.cal_quality == MFX_MAGCALOK) ||
          (mag_cal_test.cal_quality == MFX_MAGCALGOOD) )
      {
        MAG_Offset.AXIS_X = (int32_t) (mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.AXIS_Y = (int32_t) (mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.AXIS_Z = (int32_t) (mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);

        isCal =1;
        
        ALLMEMS1_PRINTF("Magneto Calibration Read\r\n");
      }
      else
      {
        isCal =0;
        ALLMEMS1_PRINTF("Magneto Calibration quality is not good\r\n");
      }
#ifdef STM32_NUCLEO
    } else {
      ALLMEMS1_PRINTF("Magneto Calibration Not correct for Current %s board\r\n",TargetBoardFeatures.mems_expansion_board ? "IKS01A2" : "IKS01A1");
      ResetCalibrationInMemory();
      MotionFX_manager_MagCal_start(SAMPLE_PERIOD);
      MotionFX_MagCal_getParams(&mag_cal_test);      
      isCal=0;
    }
#endif /* STM32_NUCLEO */
  } else {
    ALLMEMS1_PRINTF("Magneto Calibration Not present\r\n");
    isCal=0;
  }
  
  if(!isCal)
  {
    MAG_Offset.AXIS_X = 0;
    MAG_Offset.AXIS_Y = 0;
    MAG_Offset.AXIS_Z = 0;
  }
}

/**
 * @brief  Check if there are a valid Calibration Values in Memory and read them
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
unsigned char ReCallCalibrationFromMemory(uint16_t dataSize, uint32_t *data)
{
  /* ReLoad the Calibration Values from RAM */
  unsigned char Success=0;

  int i;
  
  /* Recall the calibration Credential saved */
  MDM_ReCallGMD(GMD_CALIBRATION,(void *)&CalibrationData);
  
  for(i=0; i<dataSize; i++)
  {
    data[i]= CalibrationData[i+2];
  }

  return Success;
}

/**
 * @brief  Save the Magnetometer Calibration Values to Memory
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
unsigned char SaveCalibrationToMemory(uint16_t dataSize, uint32_t *data)
{
  unsigned char Success=1;
  
  int i;

  /* Reset Before The data in Memory */
  //Success = ResetCalibrationInMemory();

  if(Success) {
    /* Store in RAM */
    CalibrationData[0] = BLUEMSYS_CHECK_CALIBRATION;
    CalibrationData[1] = TargetBoardFeatures.mems_expansion_board;

    
    for(i=0; i<dataSize; i++)
    {
      CalibrationData[i+2]= data[i];
    }
 
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite, "Magneto Calibration will be saved in FLASH\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Magneto Calibration will be saved in FLASH\r\n");
    }

    MDM_SaveGMD(GMD_CALIBRATION,(void *)&CalibrationData);
    
    NecessityToSaveMetaDataManager=1;
  }

  return Success;
}

/**
 * @brief  Reset the Magnetometer Calibration Values in Memory
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
static unsigned char ResetCalibrationInMemory(void)
{
  /* Reset Calibration Values in RAM */
  unsigned char Success=1;
  int32_t Counter;

  for(Counter=0;Counter<29;Counter++)
    CalibrationData[Counter]=0x0;
    //CalibrationData[Counter]=0xFFFFFFFF;

  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite, "Magneto Calibration will be eresed in FLASH\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    ALLMEMS1_PRINTF("Magneto Calibration will be eresed in FLASH\r\n");
  }
  
  MDM_SaveGMD(GMD_CALIBRATION,(void *)&CalibrationData);

  NecessityToSaveMetaDataManager=1;
  return Success;
}

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
