#include "main.h"
#include "i2c.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "dfu.h"
#include "lis2ds12_reg.h"

#define SENSOR_BUS hi2c1
#define BOOT_TIME 20 // ms

// Function prototypes
void SystemClock_Config(void);

// LIS2DS12 interface
static int32_t accel_write(void *handle, uint8_t reg, const uint8_t *bufp,
                           uint16_t len);
static int32_t accel_read(void *handle, uint8_t reg, uint8_t *bufp,
                          uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void accel_delay(uint32_t ms);

static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_Device_Init();

  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = accel_write;
  dev_ctx.read_reg = accel_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Wait sensor boot time */
  accel_delay(BOOT_TIME);
  /* Check device ID */
  lis2ds12_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LIS2DS12_ID)
    while (1)
    {
      HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
      HAL_Delay(50);
    }

  /* Restore default configuration */
  lis2ds12_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do
  {
    lis2ds12_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update. */
  lis2ds12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale 2g. */
  lis2ds12_xl_full_scale_set(&dev_ctx, LIS2DS12_2g);

  /* Configure filtering chain. */
  /* Accelerometer - High Pass / Slope path */
  // lis2ds12_xl_hp_path_set(&dev_ctx, LIS2DS12_HP_ON_OUTPUTS);

  /* Set Output Data Rate. */
  lis2ds12_xl_data_rate_set(&dev_ctx, LIS2DS12_XL_ODR_800Hz_HR);

  while (1)
  {
    /* Read output only if new value is available. */
    lis2ds12_reg_t reg;
    lis2ds12_status_reg_get(&dev_ctx, &reg.status);

    if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET)
    {
      jump_to_bootloader();
    }

    if (reg.status.drdy)
    {
      /* Read acceleration data. */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lis2ds12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] = lis2ds12_from_fs2g_to_mg(
          data_raw_acceleration[0]);
      acceleration_mg[1] = lis2ds12_from_fs2g_to_mg(
          data_raw_acceleration[1]);
      acceleration_mg[2] = lis2ds12_from_fs2g_to_mg(
          data_raw_acceleration[2]);
      // sprintf((char *)tx_buffer,
      //         "%4.2f\t%4.2f\t%4.2f\r\n",
      //         acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      sprintf((char *)tx_buffer, "%4.2f\r\n", acceleration_mg[2]);
      // tx_com(tx_buffer, strlen((char const *)tx_buffer));
      CDC_Transmit_FS((uint8_t)acceleration_mg[2], 4);

    }
  }
}

static int32_t accel_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LIS2DS12_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);
  return 0;
}

static int32_t accel_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LIS2DS12_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  CDC_Transmit_FS(tx_buffer, len);
}

static void accel_delay(uint32_t ms)
{
  HAL_Delay(ms);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 21;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
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
