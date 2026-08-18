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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart3;


/* USER CODE BEGIN PV */
float batteryVoltage = 0.0f;
float batteryCurrent = 0.0f;
float batterySOC = 0.0f;
float deviceTemperature = 0.0f;
float batteryTemperature = 0.0f;
float loadVoltage = 0.0f;
float loadCurrent = 0.0f;
float loadPower = 0.0f;
float pvVolt = 0.0f;
float pvCurrent = 0.0f;
float pvPower = 0.0f;
uint8_t lastResponse[8] = {0};   // untuk simpan respon terakhir Write_SOC_Cutoff
uint8_t lastRequest[8]  = {0};   // untuk simpan request terakhir (opsional)

//charge state
uint8_t chargeState = 0;
uint8_t loadState = 0;
uint8_t loadIntensity = 0;

//Parameter MPPT SRNE
float PVChargeCurrent = 0.0f;
float BatteryCapacity = 0.0f;
float BatteryRateVoltage = 0.0f;
float BatteryType = 0.0f;
float OverVoltage= 0.0f;
float ChargeLimit= 0.0f;
float EqualizingCV= 0.0f;
float BoostCV= 0.0f;
float FloatCV= 0.0f;
float BoostRV= 0.0f;
float EqualizingDuration = 0.0f;
float BoostDuration = 0.0f;
int chargeSOC = 0;
int dischargeSOC = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

//============================================================= Fungsi CRC-16 Modbus
uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==================== Read Device & Battery Temperature ====================
void Read_Temperatures(void)
{
    uint8_t request[] = {0x01, 0x03, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00};
    uint16_t crc = Modbus_CRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    uint8_t response[7];

    HAL_UART_Transmit(&huart3, request, sizeof(request), HAL_MAX_DELAY);
    if (HAL_UART_Receive(&huart3, response, sizeof(response), HAL_MAX_DELAY) == HAL_OK)
    {
        uint16_t crc_calc = Modbus_CRC16(response, 5);
        uint16_t crc_recv = response[5] | (response[6] << 8);
        if (crc_calc == crc_recv)
        {
            uint8_t devRaw = response[3];
            uint8_t batRaw = response[4];

            int8_t devSign = (devRaw & 0x80) ? -1 : 1;
            int8_t batSign = (batRaw & 0x80) ? -1 : 1;

            deviceTemperature = devSign * (devRaw & 0x7F);
            batteryTemperature = batSign * (batRaw & 0x7F);
        }
    }
}


// ===============================Charge State =================================================
void Read_Charge_State(void) {
    uint8_t request[8];
    uint8_t response[7];
    uint16_t crc;

    request[0] = 0x01; // Device ID
    request[1] = 0x03; // Function code
    request[2] = 0x00; // High byte
    request[3] = 0xFD; // Register 0x00FD
    request[4] = 0x00;
    request[5] = 0x01; // 1 register

    crc = Modbus_CRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    HAL_UART_Transmit(&huart3, request, 8, HAL_MAX_DELAY);

    if (HAL_UART_Receive(&huart3, response, 7, 500) == HAL_OK) {
        uint16_t crc_calc = Modbus_CRC16(response, 5);
        uint16_t crc_recv = response[5] | (response[6] << 8);

        if (crc_calc == crc_recv) {
            uint16_t raw = (response[3] << 8) | response[4];

            uint8_t high = (raw >> 8) & 0xFF;
            uint8_t low  = raw & 0xFF;

            loadState     = (high & 0x80) ? 1 : 0; // b7
            loadIntensity = high & 0x7F;           // b0-b6
            chargeState   = low;                   // langsung mapping
        }
    }
}
//====================================== VERSI 2 ModBus ============================================================
// Fungsi umum untuk baca 1 register Modbus
uint8_t Read_Modbus_Register(uint16_t regAddr, float scale, float *result) {
    uint8_t request[8];
    uint8_t response[16];
    uint16_t crc;

    request[0] = 0x01;                     // Address device SRNE
    request[1] = 0x03;                     // Function code
    request[2] = (regAddr >> 8) & 0xFF;    // Start address high
    request[3] = regAddr & 0xFF;           // Start address low
    request[4] = 0x00;                     // Number of registers high
    request[5] = 0x01;                     // Number of registers low

    crc = Modbus_CRC16(request, 6);
    request[6] = crc & 0xFF;        // CRC Low
    request[7] = (crc >> 8) & 0xFF; // CRC High

    // Kirim request
    HAL_UART_Transmit(&huart3, request, 8, HAL_MAX_DELAY);

    // Terima response
    if (HAL_UART_Receive(&huart3, response, 7, 500) == HAL_OK) {
        uint16_t crc_calc = Modbus_CRC16(response, 5);
        uint16_t crc_recv = response[5] | (response[6] << 8);

        if (crc_calc == crc_recv) {
            uint16_t raw_value = (response[3] << 8) | response[4];
            *result = raw_value * scale;
            return 1; // sukses
        }
    }
    return 0; // gagal
}

// ----------------- Fungsi spesifik -------------------
// ----------------- Battery Related -------------------
void Read_Battery_Voltage(void) {
    Read_Modbus_Register(0x0101, 0.1f, &batteryVoltage);
}
void Read_Battery_Current(void){
	Read_Modbus_Register(0x0102, 0.01f, &batteryCurrent);
}
void Read_Battery_SOC(void) {
    Read_Modbus_Register(0x0100, 1.0f, &batterySOC);
}
// ----------------- Load Related -------------------
void Read_Load_Voltage(void) {
    Read_Modbus_Register(0x0104, 0.1f, &loadVoltage);
}
void Read_Load_Current(void) {
    Read_Modbus_Register(0x0105, 0.01f, &loadCurrent);
}
void Read_Load_Power(void) {
    Read_Modbus_Register(0x0106, 1.0f, &loadPower);
}
// ----------------- PV Related -------------------
void Read_PV_Voltage(void) {
    Read_Modbus_Register(0x0107, 0.1f, &pvVolt);
}
void Read_PV_Current(void) {
    Read_Modbus_Register(0x0108, 0.01f, &pvCurrent);
}
void Read_PV_Power(void) {
	pvPower= pvVolt*pvCurrent;
}
void Read_PV_Charge_Current(void) {
	Read_Modbus_Register(0xE001, 0.01f, &PVChargeCurrent);
}
void Read_Battery_Capacity(void) {
	Read_Modbus_Register(0xE002, 1.0f, &BatteryCapacity);
}
void Read_Rate_Voltage(void) {
	Read_Modbus_Register(0xE003, 1.0f, &BatteryRateVoltage);
}
void Read_Battery_Type(void) {
	Read_Modbus_Register(0xE004, 1.0f, &BatteryType);
}
void Read_Over_Voltage(void) {
	Read_Modbus_Register(0xE005, 0.1f, &OverVoltage);
}
void Read_Charge_Limit(void) {
	Read_Modbus_Register(0xE006, 0.1f, &ChargeLimit);
}
void Read_Equalizing_CV(void) {
	Read_Modbus_Register(0xE007, 0.1f, &EqualizingCV);
}
void Read_Boost_CV(void) {
	Read_Modbus_Register(0xE008, 0.1f, &BoostCV);
}
void Read_Float_CV(void) {
	Read_Modbus_Register(0xE009, 0.1f, &FloatCV);
}
void Read_Boost_RV(void) {
	Read_Modbus_Register(0xE00A, 0.1f, &BoostRV);
}
void Read_Equalizing_Duration(void) {
	Read_Modbus_Register(0xE011, 0.1f, &EqualizingDuration);
}
void Read_Boost_Duration(void) {
	Read_Modbus_Register(0xE012, 0.1f, &BoostDuration);
}
// ====================== Read E00F (Charge Cut-off SOC & Discharge Cut-off SOC) ======================
void Read_SOC_Cutoff(void) {
    uint8_t request[8];
    uint8_t response[7];
    uint16_t crc;

    // Request frame
    request[0] = 0x01;        // Device ID
    request[1] = 0x03;        // Function code: Read Holding Register
    request[2] = 0xE0;        // Register address high byte
    request[3] = 0x0F;        // Register address low byte
    request[4] = 0x00;        // Number of registers high byte
    request[5] = 0x01;        // Number of registers low byte (1 register)

    crc = Modbus_CRC16(request, 6);
    request[6] = crc & 0xFF;          // CRC Low
    request[7] = (crc >> 8) & 0xFF;   // CRC High

    HAL_UART_Transmit(&huart3, request, 8, HAL_MAX_DELAY);

    if (HAL_UART_Receive(&huart3, response, 7, 500) == HAL_OK) {
        uint16_t crc_calc = Modbus_CRC16(response, 5);
        uint16_t crc_recv = response[5] | (response[6] << 8);


        if (crc_calc == crc_recv) {
            uint16_t raw = (response[3] << 8) | response[4];

            uint8_t high = (raw >> 8) & 0xFF;
            uint8_t low  = raw & 0xFF;

            chargeSOC    = high; // High byte
            dischargeSOC = low & 0xFF;        // Low byte
        }
    }
}
// ============================ Write Single Register ============================
// Function Code: 0x06
uint8_t Write_Modbus_Register(uint16_t regAddr, uint16_t value) {
    uint8_t request[8];
    uint8_t response[8];
    uint16_t crc;

    request[0] = 0x01;                     // Device ID (SRNE)
    request[1] = 0x06;                     // Function Code: Write Single Register
    request[2] = (regAddr >> 8) & 0xFF;    // High byte address
    request[3] = regAddr & 0xFF;           // Low byte address
    request[4] = (value >> 8) & 0xFF;      // High byte value
    request[5] = value & 0xFF;             // Low byte value

    crc = Modbus_CRC16(request, 6);
    request[6] = crc & 0xFF;        // CRC Low
    request[7] = (crc >> 8) & 0xFF; // CRC High

    HAL_UART_Transmit(&huart3, request, 8, HAL_MAX_DELAY);

    // Response biasanya sama persis dengan request
    if (HAL_UART_Receive(&huart3, response, 8, 500) == HAL_OK) {
        uint16_t crc_calc = Modbus_CRC16(response, 6);
        uint16_t crc_recv = response[6] | (response[7] << 8);

        if (crc_calc == crc_recv) {
            return 1; // sukses
        }
    }
    return 0; // gagal
}

// ---------------- Fungsi spesifik untuk register E001, E002, E003 ----------------
void Write_PV_Charge_Current(uint16_t currentValue) {
    Write_Modbus_Register(0xE001, currentValue);
}
void Write_Battery_Capacity(uint16_t capacityValue) {
    Write_Modbus_Register(0xE002, capacityValue);
}
void Write_Battery_Rated_Voltage(uint16_t voltValue) {
    Write_Modbus_Register(0xE003, voltValue);
}
void Write_Over_Voltage(uint16_t OverVolt) {
    Write_Modbus_Register(0xE005, OverVolt);
}
void Write_Charge_Limit(uint16_t ChargeLim) {
    Write_Modbus_Register(0xE006, ChargeLim);
}
void Write_Equalizing_CV(uint16_t EqualCV) {
    Write_Modbus_Register(0xE007, EqualCV);
}
void Write_Boost_CV(uint16_t Boost) {
    Write_Modbus_Register(0xE008, Boost);
}
void Write_Float_CV(uint16_t FloatV) {
    Write_Modbus_Register(0xE009, FloatV);
}
void Write_Boost_RV(uint16_t BoostR) {
    Write_Modbus_Register(0xE00A, BoostR);
}
void Write_Equalizing_Duration(uint16_t EqualD) {
    Write_Modbus_Register(0xE011, EqualD);
}
void Write_Boost_Duration(uint16_t BoostD) {
    Write_Modbus_Register(0xE012, BoostD);
}
// Fungsi untuk Write SOC Cutoff (register 0xE00F)
// Fungsi untuk Write SOC Cutoff (register 0xE00F)
//void Write_SOC_Cutoff(uint8_t chargeSOC_in, uint8_t dischargeSOC_in) {
//    uint8_t request[8];
//    uint8_t response[8];
//    uint16_t crc;
//
//    //uint16_t value = ((uint16_t)chargeSOC_in << 8) | dischargeSOC_in;
//
//    request[0] = 0x01;
//    request[1] = 0x06;
//    request[2] = 0xE0;
//    request[3] = 0x0F;
//    request[4] = (value >> 8) & 0xFF;
//    request[5] = value & 0xFF;
//
//    crc = Modbus_CRC16(request, 6);
//    request[6] = crc & 0xFF;
//    request[7] = (crc >> 8) & 0xFF;
//
//    // simpan request ke variabel global biar bisa dilihat di Live Expression
//   // memcpy(lastRequest, request, 8);
//
//    HAL_UART_Transmit(&huart3, request, 8, HAL_MAX_DELAY);
//
//    if (HAL_UART_Receive(&huart3, response, 8, 500) == HAL_OK) {
//        //memcpy(lastResponse, response, 8);   // simpan balasan biar bisa dilihat di Live Expression
//
//        uint16_t crc_calc = Modbus_CRC16(response, 6);
//        uint16_t crc_recv = response[6] | (response[7] << 8);
//
//        if (crc_calc == crc_recv) {
//            //if (memcmp(request, response, 6) == 0) {
//                chargeSOC    = response[4];
//                dischargeSOC = response[5];
////                return 1;
//            //}
//        }
//    }
////    return 0;
//}

void Write_SOC_Cutoff(uint8_t chargeSOC_in, uint8_t dischargeSOC_in) {
    uint8_t request[8];
    uint8_t response[8];
    uint16_t crc;

    // NOTE: value harus digabung high-byte (chargeSOC) + low-byte (dischargeSOC)
    uint16_t value = ((uint16_t)chargeSOC_in << 8) | dischargeSOC_in;

    request[0] = 0x01;        // Device ID
    request[1] = 0x06;        // Function code: Write Single Register
    request[2] = 0xE0;        // Register address high byte
    request[3] = 0x0F;        // Register address low byte
    request[4] = (value >> 8) & 0xFF;
    request[5] = value & 0xFF;

    crc = Modbus_CRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    HAL_UART_Transmit(&huart3, request, 8, HAL_MAX_DELAY);

    if (HAL_UART_Receive(&huart3, response, 8, 500) == HAL_OK) {
        uint16_t crc_calc = Modbus_CRC16(response, 6);
        uint16_t crc_recv = response[6] | (response[7] << 8);

        if (crc_calc == crc_recv) {
            chargeSOC    = response[4];
            dischargeSOC = response[5];
        }
    }
}


//==================================================================================================================

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
  // NOTE: gunakan skala "12V-equivalent" karena E003=24 (controller akan *x2* internal)
  Write_PV_Charge_Current(500);
  Write_Battery_Capacity(50);        // 50 Ah
  Write_Battery_Rated_Voltage(24);   // set system = 24V

  // nilai di-encode sebagai 12V-equivalent * 10 (0.1V unit)
  Write_Over_Voltage(148);           // 29.6V -> 14.8 *10 = 148
  Write_Charge_Limit(146);           // 29.2V -> 14.6 *10 = 146

  Write_Equalizing_CV(146);          // samakan dengan charge limit (LiFePO4 tidak perlu equalize)
  Write_Boost_CV(146);               // 29.2V -> 14.6*10 =146
  Write_Float_CV(136);               // 27.2V -> 13.6*10 =136
  Write_Boost_RV(136);               // reconnect 27.2V -> 136

  Write_Equalizing_Duration(0);      // disable equalize
  Write_Boost_Duration(30);          // 30 menit

  Write_SOC_Cutoff(100, 20);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  Write_SOC_Cutoff(100, 20);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  Read_Battery_Voltage();
	  Read_Battery_Current();
	  Read_Battery_SOC();
		// ----------------- Load Related -------------------
	  Read_Load_Voltage();
	  Read_Load_Current();
	  Read_Load_Power();
		// ----------------- PV Related -------------------
	  Read_PV_Voltage();
	  Read_PV_Current();
	  Read_PV_Power();
	  //====================Temperature======================
	  Read_Temperatures();
	  //===================Charging State====================
	  Read_Charge_State();
	  //===================Read Battery Parameter============
	  Read_PV_Charge_Current();
	  Read_Battery_Capacity();
	  Read_Rate_Voltage();
	  Read_Battery_Type();
	  Read_Over_Voltage();
	  Read_Charge_Limit();
	  Read_Equalizing_CV();
	  Read_Boost_CV();
	  Read_Float_CV();
	  Read_Boost_RV();
	  Read_Equalizing_Duration();
	  Read_Boost_Duration();
	  Read_SOC_Cutoff();
	  //====================Write Parameter========================
//	  Write_Over_Voltage(146);   // misalnya 10.0 A, kalau scale = 0.1
//	  Write_Charge_Limit(144);
//	  Write_Equalizing_CV(142);
//	  Write_Boost_CV(142);
//	  Write_Float_CV(135);
//	  Write_Boost_RV(124);
//	  Write_Equalizing_Duration(120);
//	  Write_Boost_Duration(120);
//	  Write_SOC_Cutoff(100, 20);
//	  Write_Battery_Capacity(50);    // 200 Ah


	  HAL_Delay(2000);
	  Read_SOC_Cutoff();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
