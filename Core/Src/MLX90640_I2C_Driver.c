#include "MLX90640_I2C_Driver.h"

#include "i2c.h"
#include "gpio.h"
#include "cmsis_os.h"

/* The MLX90640 driver wraps I2C1 plus the TCA9548A channel switch so the
 * higher-level thermal app can work with sensors as independent units.
 */
static HAL_StatusTypeDef tca9548aLastHalStatus = HAL_OK;
static uint32_t tca9548aLastErrorCode = HAL_I2C_ERROR_NONE;
static HAL_StatusTypeDef mlx90640LastHalStatus = HAL_OK;
static uint32_t mlx90640LastErrorCode = HAL_I2C_ERROR_NONE;
static volatile uint8_t mlx90640I2cRxDone = 0U;
static volatile uint8_t mlx90640I2cTxDone = 0U;
static volatile uint8_t mlx90640I2cError = 0U;
static uint8_t i2c1StartupRecoveryDone = 0U;
static uint8_t tca9548aConsecutiveBusyCount = 0U;
static const uint8_t mlx90640SensorChannels[MLX90640_SENSOR_NUM] = {0U, 1U, 2U, 3U};

static HAL_StatusTypeDef MLX90640_I2CWaitForDmaRx(uint32_t tickstart);

static void mlx_delay_ms(uint32_t delay_ms)
{
  /* Delay with the RTOS when it is running, otherwise use bare-metal delay. */
  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

static void I2C1_RecoverBus(void)
{
  /* Recover a stuck I2C bus by clocking SCL and reinitializing I2C1. */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1);

  for (uint8_t pulse = 0U; pulse < 9U; pulse++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1);

  MX_I2C1_Init();
}

static uint8_t TCA9548A_IsBusyOrTimeout(void)
{
  /* Treat HAL_BUSY and timeout as a failed mux transaction. */
  return ((tca9548aLastHalStatus == HAL_BUSY) ||
          ((tca9548aLastErrorCode & HAL_I2C_ERROR_TIMEOUT) != 0U)) ? 1U : 0U;
}

static HAL_StatusTypeDef TCA9548A_WriteChannelMask(uint8_t channelMask)
{
  /* Select a downstream channel through the TCA9548A using DMA. */
  mlx90640I2cRxDone = 0U;
  mlx90640I2cTxDone = 0U;
  mlx90640I2cError = 0U;

  tca9548aLastHalStatus = HAL_I2C_Master_Transmit_DMA(
      &hi2c1, (uint16_t)(TCA9548A_ADDR_7BIT << 1), &channelMask, 1U);
  tca9548aLastErrorCode = hi2c1.ErrorCode;
  if (tca9548aLastHalStatus == HAL_OK)
  {
    tca9548aLastHalStatus = MLX90640_I2CWaitForDmaRx(HAL_GetTick());
    tca9548aLastErrorCode = hi2c1.ErrorCode;
  }
  return tca9548aLastHalStatus;
}

static HAL_StatusTypeDef MLX90640_I2CWaitForDmaRx(uint32_t tickstart)
{
  /* Wait for the DMA completion flags or abort if the transfer stalls. */
  while ((mlx90640I2cRxDone == 0U) && (mlx90640I2cTxDone == 0U) && (mlx90640I2cError == 0U))
  {
    if ((HAL_GetTick() - tickstart) > MLX90640_I2C_DMA_TIMEOUT_MS)
    {
      mlx90640LastHalStatus = HAL_TIMEOUT;
      mlx90640LastErrorCode = hi2c1.ErrorCode | HAL_I2C_ERROR_TIMEOUT;

      if (hi2c1.hdmarx != NULL)
      {
        (void)HAL_DMA_Abort(hi2c1.hdmarx);
      }
      if (hi2c1.hdmatx != NULL)
      {
        (void)HAL_DMA_Abort(hi2c1.hdmatx);
      }

      (void)HAL_I2C_DeInit(&hi2c1);
      (void)HAL_I2C_Init(&hi2c1);
      return HAL_TIMEOUT;
    }

    mlx_delay_ms(1U);
  }

  if (mlx90640I2cError != 0U)
  {
    mlx90640LastHalStatus = HAL_ERROR;
    mlx90640LastErrorCode = hi2c1.ErrorCode;
    return HAL_ERROR;
  }

  mlx90640LastHalStatus = HAL_OK;
  mlx90640LastErrorCode = hi2c1.ErrorCode;
  return HAL_OK;
}

static int MLX90640_HAL_StatusToError(HAL_StatusTypeDef status)
{
  /* Convert HAL-level outcomes into the sensor API's error model. */
  if (status == HAL_OK)
  {
    return MLX90640_NO_ERROR;
  }

  if ((status == HAL_ERROR) && ((hi2c1.ErrorCode & HAL_I2C_ERROR_AF) != 0U))
  {
    return -MLX90640_I2C_NACK_ERROR;
  }

  return -MLX90640_I2C_WRITE_ERROR;
}

void MLX90640_I2CInit(void)
{
  /* Reinitialize I2C1 before talking to the thermal sensor chain. */
  MX_I2C1_Init();
}

int MLX90640_I2CGeneralReset(void)
{
  /* Broadcast the MLX90640 reset command across the active I2C bus. */
  uint8_t command = 0x06U;
  mlx90640I2cRxDone = 0U;
  mlx90640I2cTxDone = 0U;
  mlx90640I2cError = 0U;

  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(&hi2c1, 0x00U, &command, 1U);
  if (status == HAL_OK)
  {
    status = MLX90640_I2CWaitForDmaRx(HAL_GetTick());
  }
  mlx90640LastHalStatus = status;
  mlx90640LastErrorCode = hi2c1.ErrorCode;
  return MLX90640_HAL_StatusToError(status);
}

int TCA9548A_SelectChannel(uint8_t channel)
{
  /* Pick the sensor channel and recover the bus if the mux looks unhealthy. */
  if (channel > 7U)
  {
    tca9548aLastHalStatus = HAL_ERROR;
    tca9548aLastErrorCode = HAL_I2C_ERROR_NONE;
    return -MLX90640_I2C_WRITE_ERROR;
  }

  uint8_t channelMask = (uint8_t)(1U << channel);

  if (i2c1StartupRecoveryDone == 0U)
  {
    I2C1_RecoverBus();
    (void)TCA9548A_WriteChannelMask(0U);
    mlx_delay_ms(1U);
    i2c1StartupRecoveryDone = 1U;
  }

  (void)TCA9548A_WriteChannelMask(channelMask);

  if (TCA9548A_IsBusyOrTimeout() != 0U)
  {
    tca9548aConsecutiveBusyCount++;

    if (tca9548aConsecutiveBusyCount >= 3U)
    {
      I2C1_RecoverBus();
      (void)TCA9548A_WriteChannelMask(0U);
      mlx_delay_ms(1U);
      (void)TCA9548A_WriteChannelMask(channelMask);
      tca9548aConsecutiveBusyCount = (TCA9548A_IsBusyOrTimeout() != 0U) ? 1U : 0U;
    }
  }
  else
  {
    tca9548aConsecutiveBusyCount = 0U;
  }

  int status = MLX90640_HAL_StatusToError(tca9548aLastHalStatus);
  if (status == MLX90640_NO_ERROR)
  {
    mlx_delay_ms(1U);
  }

  return status;
}

int MLX90640_SelectSensor(uint8_t sensorIndex)
{
  /* Map the logical sensor index to the corresponding mux channel. */
  if (sensorIndex >= MLX90640_SENSOR_NUM)
  {
    return -MLX90640_I2C_WRITE_ERROR;
  }

  return TCA9548A_SelectChannel(mlx90640SensorChannels[sensorIndex]);
}

uint8_t MLX90640_GetSensorChannel(uint8_t sensorIndex)
{
  /* Expose the mux channel for logging and diagnostics. */
  if (sensorIndex >= MLX90640_SENSOR_NUM)
  {
    return 0xFFU;
  }

  return mlx90640SensorChannels[sensorIndex];
}

HAL_StatusTypeDef TCA9548A_GetLastHalStatus(void)
{
  /* Surface the last mux HAL status for higher-level diagnostics. */
  return tca9548aLastHalStatus;
}

uint32_t TCA9548A_GetLastErrorCode(void)
{
  /* Surface the last mux error code for higher-level diagnostics. */
  return tca9548aLastErrorCode;
}

HAL_StatusTypeDef MLX90640_GetLastHalStatus(void)
{
  /* Surface the last sensor HAL status for higher-level diagnostics. */
  return mlx90640LastHalStatus;
}

uint32_t MLX90640_GetLastErrorCode(void)
{
  /* Surface the last sensor error code for higher-level diagnostics. */
  return mlx90640LastErrorCode;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
  /* Read the frame or EEPROM words in bounded chunks using DMA. */
  if (data == NULL)
  {
    return -MLX90640_FRAME_DATA_ERROR;
  }

  uint16_t wordsRemaining = nMemAddressRead;
  uint16_t currentAddress = startAddress;
  uint16_t dataIndex = 0U;
  uint8_t rawBuffer[MLX90640_I2C_READ_CHUNK_WORDS * 2U];

  while (wordsRemaining > 0U)
  {
    uint16_t wordsToRead = (wordsRemaining > MLX90640_I2C_READ_CHUNK_WORDS) ?
        MLX90640_I2C_READ_CHUNK_WORDS : wordsRemaining;

    mlx90640I2cRxDone = 0U;
    mlx90640I2cTxDone = 0U;
    mlx90640I2cError = 0U;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(
        &hi2c1,
        (uint16_t)(slaveAddr << 1),
        currentAddress,
        I2C_MEMADD_SIZE_16BIT,
        rawBuffer,
        (uint16_t)(wordsToRead * 2U));
    mlx90640LastHalStatus = status;
    mlx90640LastErrorCode = hi2c1.ErrorCode;

    if (status != HAL_OK)
    {
      return MLX90640_HAL_StatusToError(status);
    }

    status = MLX90640_I2CWaitForDmaRx(HAL_GetTick());
    if (status != HAL_OK)
    {
      return MLX90640_HAL_StatusToError(status);
    }

    for (uint16_t i = 0U; i < wordsToRead; ++i)
    {
      data[dataIndex + i] = (uint16_t)(((uint16_t)rawBuffer[2U * i] << 8) | rawBuffer[2U * i + 1U]);
    }

    dataIndex = (uint16_t)(dataIndex + wordsToRead);
    currentAddress = (uint16_t)(currentAddress + wordsToRead);
    wordsRemaining = (uint16_t)(wordsRemaining - wordsToRead);
  }

  return MLX90640_NO_ERROR;
}

int MLX90640_I2CProbe(uint8_t slaveAddr)
{
  /* Probe the device so the app can tell whether the sensor is present. */
  mlx90640LastHalStatus = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(slaveAddr << 1), 1U, MLX90640_I2C_TIMEOUT_MS);
  mlx90640LastErrorCode = hi2c1.ErrorCode;
  return MLX90640_HAL_StatusToError(mlx90640LastHalStatus);
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
  /* Write one 16-bit register value through the DMA-backed I2C path. */
  uint8_t rawBuffer[2];
  rawBuffer[0] = (uint8_t)(data >> 8);
  rawBuffer[1] = (uint8_t)(data & 0xFFU);

  mlx90640I2cRxDone = 0U;
  mlx90640I2cTxDone = 0U;
  mlx90640I2cError = 0U;

  HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(
      &hi2c1,
      (uint16_t)(slaveAddr << 1),
      writeAddress,
      I2C_MEMADD_SIZE_16BIT,
      rawBuffer,
      2U);
  if (status == HAL_OK)
  {
    status = MLX90640_I2CWaitForDmaRx(HAL_GetTick());
  }
  mlx90640LastHalStatus = status;
  mlx90640LastErrorCode = hi2c1.ErrorCode;
  return MLX90640_HAL_StatusToError(status);
}

void MLX90640_I2CFreqSet(int freq)
{
  /* Update the I2C clock only when the caller actually requests a change. */
  if ((freq <= 0) || (hi2c1.Init.ClockSpeed == (uint32_t)freq))
  {
    return;
  }

  hi2c1.Init.ClockSpeed = (uint32_t)freq;
  HAL_I2C_DeInit(&hi2c1);
  HAL_I2C_Init(&hi2c1);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Mark the active read transaction as complete for I2C1 only. */
  if (hi2c->Instance == I2C1)
  {
    mlx90640I2cRxDone = 1U;
    mlx90640I2cTxDone = 0U;
    mlx90640I2cError = 0U;
    mlx90640LastHalStatus = HAL_OK;
    mlx90640LastErrorCode = hi2c->ErrorCode;
  }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Mark the active master transmit transaction as complete for I2C1 only. */
  if (hi2c->Instance == I2C1)
  {
    mlx90640I2cTxDone = 1U;
    mlx90640I2cRxDone = 0U;
    mlx90640I2cError = 0U;
    mlx90640LastHalStatus = HAL_OK;
    mlx90640LastErrorCode = hi2c->ErrorCode;
  }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Mark the active memory-write transaction as complete for I2C1 only. */
  if (hi2c->Instance == I2C1)
  {
    mlx90640I2cTxDone = 1U;
    mlx90640I2cRxDone = 0U;
    mlx90640I2cError = 0U;
    mlx90640LastHalStatus = HAL_OK;
    mlx90640LastErrorCode = hi2c->ErrorCode;
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  /* Record bus-level errors so the app can retry or recover later. */
  if (hi2c->Instance == I2C1)
  {
    mlx90640I2cError = 1U;
    mlx90640I2cRxDone = 0U;
    mlx90640I2cTxDone = 0U;
    mlx90640LastHalStatus = HAL_ERROR;
    mlx90640LastErrorCode = hi2c->ErrorCode;
  }
}

