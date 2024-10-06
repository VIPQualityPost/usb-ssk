#include "main.h"
#include "i2c.h"
#include "gpio.h"

#include "dfu.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "iis2iclx_reg.h"

#define SENSOR_BUS hi2c1
#define BOOT_TIME 20 // ms

// Function prototypes
void SystemClock_Config(void);

//IIS2ICLX driver
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

static int16_t data_raw[3];
static float data_formatted[3];
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
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = HAL_Delay;
  dev_ctx.handle = &SENSOR_BUS;

  HAL_Delay(BOOT_TIME);
  iis2iclx_bus_mode_set(&dev_ctx, IIS2ICLX_SEL_BY_HW);
  iis2iclx_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != IIS2ICLX_ID)
    while (1)
    {
      HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
      HAL_Delay(200);
    }

  iis2iclx_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do
  {
    iis2iclx_reset_get(&dev_ctx, &rst);
  } while (rst);

  iis2iclx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  iis2iclx_xl_data_rate_set(&dev_ctx, IIS2ICLX_ODR_FSM_104Hz);
  iis2iclx_xl_full_scale_set(&dev_ctx, IIS2ICLX_500mg);

  /* Configure filtering chain(No aux interface)
   * Accelerometer - LPF1 + LPF2 path
   */
  iis2iclx_xl_hp_path_on_out_set(&dev_ctx, IIS2ICLX_LP_ODR_DIV_100);
  iis2iclx_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

  while (1)
  {
    uint8_t reg_acc, reg_temp;

    iis2iclx_xl_flag_data_ready_get(&dev_ctx, &reg_acc);
    iis2iclx_temp_flag_data_ready_get(&dev_ctx, &reg_temp);
    memset(data_raw, 0x00, 3 * sizeof(int16_t));

    if (reg_acc)
    {
      iis2iclx_acceleration_raw_get(&dev_ctx, &data_raw[0]);
      data_formatted[0] = iis2iclx_from_fs500mg_to_mg(data_raw[0]);
      data_formatted[1] = iis2iclx_from_fs500mg_to_mg(data_raw[1]);

    }

    if(reg_temp){
      iis2iclx_temperature_raw_get(&dev_ctx, &data_raw[0] + 2);
      data_formatted[2] = iis2iclx_from_lsb_to_celsius(data_raw[2]);

    }

    if(reg_acc || reg_temp){
      memset(tx_buffer, 0x00, 3 * sizeof(float_t));
      memcpy(&tx_buffer, &data_formatted, 3 * sizeof(float_t));
      CDC_Transmit_FS((uint8_t*)tx_buffer, 3 * sizeof(float_t));
    }

    if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET)
    {
      jump_to_bootloader();
    }
  }
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, IIS2ICLX_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, IIS2ICLX_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
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
