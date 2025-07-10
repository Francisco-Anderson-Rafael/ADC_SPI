/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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

/* USER CODE BEGIN PV */
// Variáveis globais/estáticas para o seu programa
uint32_t adc_raw_value; // Variável para armazenar o valor lido do ADC SPI
char msg[100];          // Buffer para a mensagem a ser enviada pela serial
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
// Protótipos das funções que você implementará para o seu ADC SPI
void ADC_SPI_Init(void);
uint32_t ADC_SPI_ReadValue(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Função de inicialização do ADC SPI externo.
  * Esta função deve ser adaptada de acordo com o datasheet do seu ADC.
  * @param  None
  * @retval None
  */
void ADC_SPI_Init(void)
{
    // --- EXEMPLO GENÉRICO: ADAPTE PARA O SEU ADC SPI ESPECÍFICO ---
    // Alguns ADCs SPI precisam de comandos de inicialização, configuração de registradores,
    // ou calibração. Use a comunicação SPI aqui para enviar esses comandos.
    // Lembre-se de controlar o pino CS (Chip Select) do seu ADC.

    // Exemplo de sequência (substitua pelos comandos reais do seu ADC!):
    // Desativa o CS (Chip Select) antes de começar (geralmente alto por padrão)
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(10); // Pequeno atraso

    // Ativa CS (nível baixo)
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);
    // Enviar comando de reset ou configuração (ex: para ADS1256, AD7124, etc.)
    // uint8_t reset_cmd = 0x06; // Exemplo de um comando de reset
    // HAL_SPI_Transmit(&hspi1, &reset_cmd, 1, HAL_MAX_DELAY);
    // Desativa CS
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(100); // Atraso após inicialização

    // Você pode precisar de mais comandos aqui para configurar ganho, taxa de amostragem, etc.
    // Consulte o datasheet do seu ADC!
}

/**
  * @brief  Função para ler um valor do ADC SPI externo.
  * Esta função deve ser adaptada de acordo com o datasheet do seu ADC.
  * @param  None
  * @retval Valor lido do ADC (uint32_t para acomodar até 32 bits).
  */
uint32_t ADC_SPI_ReadValue(void)
{
    // --- EXEMPLO GENÉRICO: ADAPTE PARA O SEU ADC SPI ESPECÍFICO ---
    uint8_t tx_dummy_byte = 0x00; // Byte dummy para iniciar a comunicação se necessário
    uint8_t rx_data[4];           // Buffer para receber dados (até 4 bytes para 32-bit ADC)
    uint32_t value = 0;

    // 1. Ativar o Chip Select (CS) - Nível baixo
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

    // 2. Enviar o comando de leitura ou registro (se o seu ADC precisar)
    // Muitos ADCs SPI precisam de um comando para iniciar a leitura ou selecionar um canal.
    // Por exemplo, para um ADS1256, você enviaria um comando RDATA.
    // uint8_t read_command = 0x11; // Exemplo: Comando para ler dados de um registro
    // HAL_SPI_Transmit(&hspi1, &read_command, 1, HAL_MAX_DELAY);

    // 3. Receber os dados do ADC
    // Adapte o número de bytes a serem recebidos (ex: 3 para 24-bit, 2 para 16-bit).
    // Dependendo do seu ADC, você pode precisar de HAL_SPI_Receive ou HAL_SPI_TransmitReceive.
    // HAL_SPI_Receive(&hspi1, rx_data, 3, HAL_MAX_DELAY); // Exemplo para 3 bytes (24-bit ADC)
    // Ou, se o ADC precisa que você envie "dummy bytes" para clockar os dados:
    HAL_SPI_TransmitReceive(&hspi1, &tx_dummy_byte, rx_data, 3, HAL_MAX_DELAY); // 3 bytes de dados

    // 4. Desativar o Chip Select (CS) - Nível alto
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

    // 5. Reconstruir o valor lido a partir dos bytes recebidos
    // ESTA PARTE É CRÍTICA E DEPENDE DA ORDEM DE BYTES DO SEU ADC (MSB primeiro, LSB primeiro)
    // Exemplo para um ADC de 24 bits que envia MSB primeiro (Byte0, Byte1, Byte2):
    value = ( (uint32_t)rx_data[0] << 16 ) | ( (uint32_t)rx_data[1] << 8 ) | ( (uint32_t)rx_data[2] );
    // Se for 16 bits:
    // value = ( (uint32_t)rx_data[0] << 8 ) | rx_data[1];

    return value;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Chame a função de inicialização do seu ADC SPI externo (se necessário)
  // Esta função pode enviar comandos de configuração para o ADC
  ADC_SPI_Init();

  // Teste inicial da comunicação serial
  printf(msg, sizeof(msg), "Iniciando leitura ADC SPI...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. Leia o valor bruto do ADC SPI
    adc_raw_value = ADC_SPI_ReadValue();

    // 2. Converta o valor lido para uma string para envio serial
    // Use %lu para unsigned long int (adequado para uint32_t)
    printf(msg, sizeof(msg), "Valor ADC SPI: %lu\r\n", adc_raw_value);

    // 3. Envie a string pelo terminal serial
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // 4. Pequeno atraso para não sobrecarregar o terminal serial e permitir estabilização
    HAL_Delay(500); // Atraso de 500ms entre as leituras
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
