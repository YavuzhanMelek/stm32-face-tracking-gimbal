/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h" // USB üzerinden veri göndermek için

//!!!!!!!!!!BURASINI GORUNTU ISLEME  KISIMINI EKLEDIKTEN SONRA EKLIYCEM yuzmodel yerine ekleyecegim modelin ismi gelecek
#include "ai_platform.h"
#include "yuzmodel.h"
#include "yuzmodel_data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t size_buffer[4]; // Boyut bilgisini (4 bayt) tutacak tampon
  // 96*96*1 // Veri için dinamik tampon (burada volatile'a ihtiyac yok cunku
							  	  	//interrupt icinde image buffer degismiyor)
static uint8_t image_buffer[9216];

uint8_t sync_byte = 0;
uint32_t loop_counter = 0;


//!!!!!!!!!!BURASINI GORUNTU ISLEME KISIMINI EKLEDIKTEN SONRA EKLIYCEM
//GORUNTU ISLEME DOSYASINA KOYDUGUN ISMI DEGOSTIRMEYI UNUTMA MESELA BURDA YUZMODE
ai_handle yuzmodel= AI_HANDLE_NULL;
ai_network_report report;
ai_error err;

//////BURADA OZELLIKLE TANITTIGIMIZ MODELE NE AD VERMISSEK ONUN ADINI YAZIYORUZ MESELA BENIMKI "YUZMODEL"
//ram sigmadigindan dolayi modelimizi quantize ettik ondan dolayi
//verecegimiz girdi float degil int_8 olacak yani ai da ai_i8 verecegiz

static ai_u8 activations[AI_YUZMODEL_DATA_ACTIVATIONS_SIZE];
static ai_i8 in_data[AI_YUZMODEL_IN_1_SIZE_BYTES];
//OUTPUT KISMINI SIMDI 2 BYTE E CIKRAMA SEBEBIMIZ IKI CIKTININ OLMASI
// --- YENİ HALİ ---
// Model artik int8 ürettiği için tamponu da ai_i8 (yani int8_t) yapıyoruz.
static ai_i8 out_data_int8[AI_YUZMODEL_OUT_1_SIZE_BYTES + AI_YUZMODEL_OUT_2_SIZE_BYTES];

// Sadece tanımla, içini main'de dolduracağız
ai_buffer ai_input[AI_YUZMODEL_IN_NUM];
ai_buffer ai_output[AI_YUZMODEL_OUT_NUM];


// Sadece tanımla, içini main'de dolduracağız

// ── Servo Ayarları ──────────────────────────────────────────
// TIM2: Prescaler=83, Period=19999 → 50Hz PWM, 1 tick = 1μs
#define SERVO_MIN_US   500    // 0 derece
#define SERVO_MAX_US   2500   // 180 derece
#define SERVO_MID_US   1500   // 90 derece (merkez)

// Yumuşatma katsayısı (0.0-1.0). Küçüldükçe hareket daha yavaş/yumuşak olur.
#define SMOOTH_ALPHA   0.15f

// Görüntü içinde hedef: merkez (0.5, 0.5)
// cx/cy modelden 0.0-1.0 arası geliyor
#define TARGET_CX      0.5f
#define TARGET_CY      0.5f

// Hata çok küçükse servoyu hareket ettirme (titreme engeli)
#define DEADZONE       0.03f

// Mevcut servo pozisyonları (μs cinsinden, float ile yumuşak güncellenir)
static float servo_x_us = SERVO_MID_US;
static float servo_y_us = 300.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

// ── Yardımcı: float clamp ────────────────────────────────────
static inline float clamp_f(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ── Normalize koordinatı servo μs değerine çevir ────────────
// norm: 0.0 (sol/üst) → 1.0 (sağ/alt)
// invert: ekseni ters çevirmek gerekirse 1 yap
static float coord_to_us(float norm, uint8_t invert) {
    if (invert) norm = 1.0f - norm;
    float us = SERVO_MIN_US + norm * (SERVO_MAX_US - SERVO_MIN_US);
    return clamp_f(us, SERVO_MIN_US, SERVO_MAX_US);
}

// ── Servo PWM güncelle ───────────────────────────────────────
static void servo_set(float x_us, float y_us) {
    uint32_t x = (uint32_t)clamp_f(x_us, SERVO_MIN_US, SERVO_MAX_US);
    uint32_t y = (uint32_t)clamp_f(y_us, 0.0f, 700.0f);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, x);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, y);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */


//  ai_yuzmodel_get_report(yuzmodel, &report); fonksiyonu, STM32 içindeki yapay zeka modelinin "kimlik kartını"
//  okumaya yarar. Bu komut çalıştırıldığında report yapısının içine şu kritik bilgiler dolar:
//
//  Modelin Yapısı: Giriş ve çıkış buffer'larının tam boyutları ve kaç katmandan oluştuğu.
//
//  Quantization (Niceleme) Parametreleri: Bir önceki mesajda bahsettiğimiz scale ve zero_point değerleri bu fonksiyon sayesinde elde edilir.
//
//  Bellek Kullanımı: Modelin çalışma anında ne kadar RAM (activations) harcadığı.

//!!!!!!!! BU KISIM GORUNTU ISLEME EKLENDIGINDE EKLENECEK EKLEDIGINDE TUM
//  ISIMLERI KOYDUGUN GORUNTU ISLEMENIN ISMINE DONUSTURDUGUNDEN EMIN OL
  // 1. Modeli Oluştur
  // Activation buffer'ını doğrudan map'e yerleştir
  // 1. Model Parametre Yapısını Tanımla ve İçini Doldur
  // 2. Modeli Bu Parametrelerle Oluştur
  err = ai_yuzmodel_create(&yuzmodel, AI_YUZMODEL_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
      while(1) {}
  }

  ai_network_params params;
  if (!ai_yuzmodel_data_params_get(&params)) {
      while(1) {}
  }

  // Bu versiyonda activation bağlama yöntemi:
  AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, 0, activations);

    // 3. Modeli Başlat (Artık yukarısı ayarlandığı için doğrudan params veriyoruz)
    if (!ai_yuzmodel_init(yuzmodel, &params)) {
        while(1) {}
    }

    // 4. Model Kimlik Kartını Oku
    ai_yuzmodel_get_report(yuzmodel, &report);

    // Giriş/Çıkış katman eşlemeleri (Buralar aynı kalıyor)
    ai_buffer* tmp_in  = ai_yuzmodel_inputs_get(yuzmodel, NULL);
    ai_buffer* tmp_out = ai_yuzmodel_outputs_get(yuzmodel, NULL);

    ai_input[0]  = *tmp_in;
    ai_input[0].data  = (ai_handle)in_data;

    ai_output[0] = tmp_out[0];
    ai_output[1] = tmp_out[1];

    // --- YENİ HALİ ---
    ai_output[0].data = (ai_handle)out_data_int8;
    ai_output[1].data = (ai_handle)(out_data_int8 + AI_YUZMODEL_OUT_1_SIZE_BYTES);


    // ── Servo PWM Başlat (merkez konumda) ────────────────────
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // X ekseni
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // Y ekseni
    servo_set(servo_x_us, servo_y_us);
    HAL_Delay(500); // Servoların merkeze gelmesi için bekle

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//!!!! bu goruntu isleme algoritmasini koyduktan sonra ustteki yerine calistiracagimiz while dongusu

  while (1)
  {
      // 1. HER DÖNGÜ BAŞINDA HATTI SIFIRLA
      // Eğer UART hata verdiyse veya bir şeyler takıldıysa temizler
      if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET) {
          __HAL_UART_CLEAR_OREFLAG(&huart3);
          HAL_UART_AbortReceive(&huart3);
      }

      uint8_t sync_buf[4] = {0};
      uint8_t b = 0;
      uint8_t found = 0;
      uint32_t gray_size = 0;

      for (int attempt = 0; attempt < 20000; attempt++) {
          if (HAL_UART_Receive(&huart3, &b, 1, 2) != HAL_OK) continue;
          sync_buf[0] = sync_buf[1];
          sync_buf[1] = sync_buf[2];
          sync_buf[2] = sync_buf[3];
          sync_buf[3] = b;
          if (sync_buf[0]==0xAA && sync_buf[1]==0xBB &&
              sync_buf[2]==0xCC && sync_buf[3]==0xDD) {
              found = 1;
              break;
          }
      }

      if (found) {
          uint8_t size_buf[4];
          if (HAL_UART_Receive(&huart3, size_buf, 4, 100) == HAL_OK) {
              gray_size = size_buf[0] | (size_buf[1]<<8) |
                          (size_buf[2]<<16) | (size_buf[3]<<24);

              if (gray_size == 9216) {  // 96x96x1 grayscale
                  if (HAL_UART_Receive(&huart3, image_buffer, gray_size, 250) == HAL_OK) {
                      loop_counter++;

                      // --- YENİ INT8 UYUMLU KOD ---
                      // Artik float pointer kullanmiyoruz, modelin kendi girdi tipini (ai_i8) hedef aliyoruz
                      ai_i8* model_input_ptr = (ai_i8*)in_data;

                      for(int i = 0; i < 9216; i++) {
                          // Görüntü tamponundaki 0-255 arası uint8 veriyi int8 formatına dönüştürüp atıyoruz.
                          // INT8 quantization girdiyi işaretli (signed) aralıkta bekleyebilir, cast etmek en güvenlisidir.
                          model_input_ptr[i] = (ai_i8)(image_buffer[i] - 128);
                      }

                      if (ai_yuzmodel_run(yuzmodel, ai_input, ai_output) > 0) {
                    	  // --- 1. ÇIKIŞ GRUBU: KOORDİNATLAR (OUT_1) ---
                    	      // Modelden gelen ham int8 veriler
                    	      ai_i8 raw_cx = ((ai_i8*)ai_output[0].data)[0];
                    	      ai_i8 raw_cy = ((ai_i8*)ai_output[0].data)[1];
                    	      ai_i8 raw_prob = ((ai_i8*)ai_output[1].data)[0];

							  if (raw_prob >= 120) {
								  // Model 120 ile 127 arasında bir değer ürettiyse resmen %95-%100 emindir.
								  // Yüz kesin olarak algılandı: Bildirim LED'ini yak
								  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
								  // int8 → 0.0-1.0 normalize
								  // Model çıktısı: -128..+127 → 0.0..1.0
//								  float cx = clamp_f((float)(raw_cx + 128) / 255.0f, 0.0f, 1.0f);
//								  float cy = clamp_f((float)(raw_cy + 128) / 255.0f, 0.0f, 1.0f);
								  float cx = ((float)raw_cx - (-70.0f)) / (10.0f - (-70.0f));
								  cx = cx < 0.0f ? 0.0f : (cx > 1.0f ? 1.0f : cx);

								  float cy = ((float)raw_cy - (-80.0f)) / (-10.0f - (-80.0f));
								  cy = cy < 0.0f ? 0.0f : (cy > 1.0f ? 1.0f : cy);
								  // yüzün merkezden farkı
								  float cx_m = 1.0f - cx;
								  float err_x = cx_m - TARGET_CX;
								  float err_y = cy  - TARGET_CY;

								  // Deadzone: çok küçük hatayı yoksay (titreme engeli)
								  if (err_x > -DEADZONE && err_x < DEADZONE) err_x = 0.0f;
								  if (err_y > -DEADZONE && err_y < DEADZONE) err_y = 0.0f;

								  // Kaldığı yerden proportional adım
								  float step_x = -err_x * (SERVO_MAX_US - SERVO_MIN_US) * SMOOTH_ALPHA; // Motor ters takılı: işareti ters çevir
								  float step_y = err_y * (700.0f - 0.0f) * SMOOTH_ALPHA;

								  servo_x_us = clamp_f(servo_x_us + step_x, SERVO_MIN_US, SERVO_MAX_US);
								  servo_y_us = clamp_f(servo_y_us + step_y, 0.0f, 700.0f);

								  servo_set(servo_x_us, servo_y_us);
								  send_debug(raw_cx, raw_cy, raw_prob, cx, cy);

							  } else {
								  // Diğer tüm durumlarda (masayı falan gördüğünde bu değer düşecektir) LED'i söndür
								  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
							  }





                      }
              } else {
                  HAL_UART_AbortReceive(&huart3);
              }
          }
      }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 2000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(Audio_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void send_debug(ai_i8 raw_cx, ai_i8 raw_cy, ai_i8 raw_prob, float cx, float cy) {
    char buf[120];
    int cx_pct = (int)(cx * 100);
    int cy_pct = (int)(cy * 100);
    uint32_t cur_x = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    uint32_t cur_y = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    int len = snprintf(buf, sizeof(buf),
        "prob=%d cx_raw=%d cy_raw=%d cx=%d%% cy=%d%% x_us=%lu y_us=%lu\r\n",
        (int)raw_prob, (int)raw_cx, (int)raw_cy,
        cx_pct, cy_pct, cur_x, cur_y);
    CDC_Transmit_FS((uint8_t*)buf, len);
}
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

#ifdef  USE_FULL_ASSERT
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
