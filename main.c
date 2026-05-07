#include "stm32f4xx_hal.h"
#include <string.h>

/*
 * Two-board UART test for Nucleo-F411RE
 *
 * USART2:
 *   PC serial monitor through the on-board ST-LINK virtual COM port.
 *
 * USART1:
 *   Board-to-board UART link.
 *
 * Wiring for board-to-board UART:
 *   Master PA9  (USART1_TX) -> Slave  PA10 (USART1_RX)
 *   Master PA10 (USART1_RX) <- Slave  PA9  (USART1_TX)
 *   Master GND              -> Slave  GND
 *
 * Build role is selected in platformio.ini:
 *   ROLE_MASTER=1 -> Master sends PING and waits for PONG.
 *   ROLE_MASTER=0 -> Slave waits for PING and replies PONG.
 */

#ifndef ROLE_MASTER
#define ROLE_MASTER 1
#endif

#define UART_TIMEOUT_MS 1000U
#define LINE_BUFFER_SIZE 32U

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void Error_Handler(void);

static void pc_print(const char *text);
static HAL_StatusTypeDef link_send_line(const char *text);
static HAL_StatusTypeDef link_receive_line(char *buffer, uint16_t buffer_size, uint32_t timeout_ms);
#if ROLE_MASTER
static void run_master(void);
#else
static void run_slave(void);
#endif

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

#if ROLE_MASTER
  pc_print("\r\n[MASTER] UART board-to-board test start\r\n");
  pc_print("[MASTER] Send PING to Slave through USART1 PA9/PA10\r\n\r\n");
  run_master();
#else
  pc_print("\r\n[SLAVE] UART board-to-board test start\r\n");
  pc_print("[SLAVE] Wait PING from Master through USART1 PA9/PA10\r\n\r\n");
  run_slave();
#endif
}

#if ROLE_MASTER
static void run_master(void)
{
  char rx_line[LINE_BUFFER_SIZE];

  while (1)
  {
    pc_print("[MASTER] TX: PING\r\n");
    link_send_line("PING\r\n");

    if (link_receive_line(rx_line, sizeof(rx_line), UART_TIMEOUT_MS) == HAL_OK)
    {
      pc_print("[MASTER] RX: ");
      pc_print(rx_line);
      pc_print("\r\n");

      if (strcmp(rx_line, "PONG") == 0)
      {
        pc_print("[MASTER] Result: OK\r\n\r\n");
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      }
      else
      {
        pc_print("[MASTER] Result: Unexpected response\r\n\r\n");
      }
    }
    else
    {
      pc_print("[MASTER] Result: Timeout, no response from Slave\r\n\r\n");
    }

    HAL_Delay(1000);
  }
}
#else
static void run_slave(void)
{
  char rx_line[LINE_BUFFER_SIZE];

  while (1)
  {
    if (link_receive_line(rx_line, sizeof(rx_line), UART_TIMEOUT_MS) == HAL_OK)
    {
      pc_print("[SLAVE] RX: ");
      pc_print(rx_line);
      pc_print("\r\n");

      if (strcmp(rx_line, "PING") == 0)
      {
        pc_print("[SLAVE] TX: PONG\r\n");
        link_send_line("PONG\r\n");
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      }
      else
      {
        pc_print("[SLAVE] Unknown command\r\n");
      }
    }
  }
}
#endif

static void pc_print(const char *text)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)text, strlen(text), 100);
}

static HAL_StatusTypeDef link_send_line(const char *text)
{
  return HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), 100);
}

static HAL_StatusTypeDef link_receive_line(char *buffer, uint16_t buffer_size, uint32_t timeout_ms)
{
  uint8_t rx_byte;
  uint16_t index = 0;
  uint32_t start_tick = HAL_GetTick();

  if (buffer_size == 0)
  {
    return HAL_ERROR;
  }

  while ((HAL_GetTick() - start_tick) < timeout_ms)
  {
    if (HAL_UART_Receive(&huart1, &rx_byte, 1, 10) == HAL_OK)
    {
      if (rx_byte == '\r')
      {
        continue;
      }

      if (rx_byte == '\n')
      {
        buffer[index] = '\0';
        return HAL_OK;
      }

      if (index < (buffer_size - 1U))
      {
        buffer[index] = (char)rx_byte;
        index++;
      }
    }
  }

  buffer[index] = '\0';
  return HAL_TIMEOUT;
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (uartHandle->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

static void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    for (volatile uint32_t i = 0; i < 200000; i++)
    {
    }
  }
}
