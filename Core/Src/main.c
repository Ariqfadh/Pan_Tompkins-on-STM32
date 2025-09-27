
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "panTompkins.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define lpfBufferSize 13
#define hpfBufferSize 33
#define derivBufferSize 5
#define mwiWindowSize 38
#define REFRACTORY_PERIOD_SAMPLES 75
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int hpf_buffer[hpfBufferSize] = {0};
int hpfPtr = 0;
long hpf_y1 = 0;

int lpf_buffer[lpfBufferSize] = {0};
int lpfPtr = 0;
long lpf_y1 = 0;
long lpf_y2 = 0;

int deriv_buffer[derivBufferSize] = {0};
int derivPtr = 0;

int mwi_buffer[mwiWindowSize] = {0};
int mwiPtr = 0;
long mwiSum = 0;

//==================== Peak Detection Var ==========================

static int spki = 0;
static int npki = 0;
static int threshold1 = 0;
static int threshold2 = 0;

// Variabel untuk melacak RR-Interval
static int rrAvg = 0;
static int lastQrs = 0;
static int rrRecent[8] = {0};
static int rrRecentPtr = 0;

// State machine untuk mencari puncak
static int potentialPeakVal = 0;
static int potentialPeakTime = 0;
static int signalRising = 0;

// Waktu/sampel global
static long sample_count = 0;
static int refractory_period = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

//Init function
void panTompkins_init(void){
	memset(lpf_buffer ,0 , sizeof(lpf_buffer));
	memset(hpf_buffer ,0 , sizeof(hpf_buffer));
	memset(deriv_buffer ,0 , sizeof(deriv_buffer));
	memset(mwi_buffer ,0 , sizeof(mwi_buffer));
    memset(rrRecent, 0, sizeof(rrRecent));

	lpfPtr = 0; lpf_y1 = 0 ; lpf_y2 = 0;
	hpfPtr = 0; hpf_y1 = 0 ;
	derivPtr = 0;
	mwiPtr = 0; mwiSum =0 ;

    spki = 0;
    npki = 0;
    threshold1 = 0;
    threshold2 = 0;
    rrAvg = 0;
    lastQrs = 0;
    rrRecentPtr = 0;
    potentialPeakVal = 0;
    potentialPeakTime = 0;
    signalRising = 0;
    sample_count = 0;
    refractory_period = 0;

}

//@brief Filter Stage
static int lowPassFilter(int input){
	long y0;
	int half;


//	hitung y0
	int x6Ptr = (lpfPtr + lpfBufferSize - 6) % lpfBufferSize;
	int x12Ptr = (lpfPtr + lpfBufferSize - 12 ) % lpfBufferSize;
	y0 = 2 * lpf_y1 -lpf_y2 + input - 2 *lpf_buffer[x6Ptr] + lpf_buffer[x12Ptr];

//	history y1 dan y2
	lpf_y2 = lpf_y1;
	lpf_y1 = y0;

	lpf_buffer[lpfPtr] = input;
	lpfPtr = (lpfPtr + 1)% lpfBufferSize;
	half = y0/32;

	return (int)half;
}

static int highPassFilter(int input){
	long y0;

	int x16Ptr = (hpfPtr + hpfBufferSize - 16) % hpfBufferSize;
	int x17Ptr = (hpfPtr + hpfBufferSize - 17) % hpfBufferSize;
	int x32Ptr =(hpfPtr + hpfBufferSize - 32) % hpfBufferSize;

	// Hitung y0
	y0 = hpf_y1 - (input / 32) + hpf_buffer[x16Ptr] - hpf_buffer[x17Ptr] + (hpf_buffer[x32Ptr] / 32);
	hpf_y1 = y0;
	hpf_buffer[hpfPtr] = input;
	hpfPtr = (hpfPtr + 1) % hpfBufferSize;

	return (int)y0;
}

static int derivFilter(int input){
	long y0;

	int x1Ptr = (derivPtr + derivBufferSize -1) % derivBufferSize;
	int x3Ptr = (derivPtr + derivBufferSize -3) % derivBufferSize;
	int x4Ptr = (derivPtr + derivBufferSize -4) % derivBufferSize;

	y0 = 2 * input + deriv_buffer[x1Ptr] - deriv_buffer[x3Ptr] - 2 * deriv_buffer[x4Ptr];

	deriv_buffer[derivPtr] = input;
	derivPtr = (derivPtr + 1) % derivBufferSize;

	return (int)( y0 / 8 );
}

static int squaring(int input){
	return input * input;
}

static int movingWindowIntegration(int input){
	int oldInput = mwi_buffer[mwiPtr];
	mwiSum -= oldInput;
	mwiSum += input;

	mwi_buffer[mwiPtr] = input;
	mwiPtr = (mwiPtr + 1) % mwiWindowSize;

	return mwiSum / mwiWindowSize;

}

/**
 * @brief Fungsi utama untuk deteksi puncak dengan double threshold adaptif.
 * @param mwi_sample Sampel baru dari output Moving Window Integration.
 * @return 1 jika QRS terdeteksi, 0 jika tidak.
 */
static int peakDetection(int input){

    sample_count++;
    int qrs_detected = 0;

    // --- MANAJEMEN WAKTU & REFRAKTER ---
    if (refractory_period > 0) {
        refractory_period--;
        // Update NPK (Noise Peak) jika ada puncak tinggi saat refrakter
        if (input > npki) {
            npki = (input / 8) + (npki * 7 / 8);
        }
    } else {
        // --- 1. DETEKSI PUNCAK LOKAL ---
        if (input > potentialPeakVal) {
            // Sinyal masih naik, update potensi puncak
            potentialPeakVal = input;
            potentialPeakTime = sample_count;
            signalRising = 1;
        } else if (signalRising) {
            // Puncak lokal ditemukan (sinyal mulai turun)
            int currentPeakVal = potentialPeakVal;
            int currentPeakTime = potentialPeakTime;
            signalRising = 0;
            potentialPeakVal = input;

            // --- 2. ADAPTIVE THRESHOLDING & KLASIFIKASI ---
            if (currentPeakVal >= threshold1) {
                // Kandidat QRS (di atas Ambang Batas Sinyal)

                int rrInterval = currentPeakTime - lastQrs;

                // Pengecekan Diskriminasi T-Wave/RR Interval terlalu pendek
                if (rrInterval < 200 && rrInterval > REFRACTORY_PERIOD_SAMPLES && lastQrs != 0) {
                    // Terlalu cepat, diklasifikasikan sebagai Noise/T-Wave
                    npki = (currentPeakVal / 8) + (npki * 7 / 8); // Update NPK
                } else {
                    // QRS Terdeteksi!
                    qrs_detected = 1;

                    // Update SPK (Signal Peak)
                    spki = (currentPeakVal / 8) + (spki * 7 / 8);

                    // Update Rata-rata RR-Interval
                    // Logika update rrAvg...
                    int newRR = currentPeakTime - lastQrs;
                    if (lastQrs != 0) {
                        rrRecent[rrRecentPtr] = newRR;
                        rrRecentPtr = (rrRecentPtr + 1) % 8;
                        long sumRR = 0;
                        int countRR = 0;
                        for(int i = 0; i < 8; i++) {
                            if (rrRecent[i] > 0) {
                                sumRR += rrRecent[i];
                                countRR++;
                            }
                        }
                        if (countRR > 0) {
                            rrAvg = sumRR / countRR;
                        }
                    }
                    lastQrs = currentPeakTime;

                    // Aktifkan periode refrakter
                    refractory_period = REFRACTORY_PERIOD_SAMPLES;
                }

            } else if (currentPeakVal > threshold2) {
                // Puncak di antara THR1 dan THR2, diklasifikasikan sebagai Noise
                npki = (currentPeakVal / 8) + (npki * 7 / 8); // Update NPK
            }

            // --- 3. REKALKULASI AMBANG BATAS ---
            threshold1 = npki + ((spki - npki) / 4); // THR1 = NPK + 0.25 * (SPK - NPK)
            threshold2 = threshold1 / 2;             // THR2 = 0.5 * THR1

            // --- 4. SEARCHBACK (PENCARIAN QRS TERLEWAT) ---
            int rr_limit = (rrAvg * 5) / 3;
            if (lastQrs != 0 && (sample_count - lastQrs) > rr_limit) {
                // Jika terlalu lama tanpa deteksi, turunkan ambang batas (penyederhanaan searchback)
                if (threshold1 > 0) threshold1 = threshold1 / 2;
                if (threshold2 > 0) threshold2 = threshold2 / 2;
            }
        }
    }

    // --- INISIALISASI AWAL ---
    if (sample_count < 100 && input > spki) {
        // Set nilai awal SPK, NPK, dan Ambang batas
        spki = input;
        npki = input / 2;
        threshold1 = npki + ((spki - npki) / 4);
        threshold2 = threshold1 / 2;
    }

    return qrs_detected;
}
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void panTompkins_init(void);
extern int lowPassFilter(int input);
extern int highPassFilter(int input);
extern int derivFilter(int input);
extern int squaring(int input);
extern int movingWindowIntegration(int input);
extern int peakDetection(int input);

int ecg_data[] = {
	    100,102,105,120,300,500,800,600,400,200,100,90,85,80,75,
	    100,102,105,110,115,120,250,600,900,700,400,200,120,100
	};
int ecg_len = sizeof(ecg_data) / sizeof(int);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  panTompkins_init();

  printf("Initializing Pan-Tompkins...\n");

  for (int i = 0; i < ecg_len; i++) {
      int sample = ecg_data[i];

      // Tahapan filter
      int y_lpf  = lowPassFilter(sample);
      int y_hpf  = highPassFilter(y_lpf);
      int y_der  = derivFilter(y_hpf);
      int y_sqr  = squaring(y_der);
      int y_mwi  = movingWindowIntegration(y_sqr);

      // Deteksi QRS
      int detected = peakDetection(y_mwi);

      if (detected) {
          printf("QRS detected at sample %d (val=%d)\n", i, sample);
          // Misalnya toggle LED
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
      }

      HAL_Delay(2); // delay sesuai sampling rate (contoh 500Hz → 2ms)
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
