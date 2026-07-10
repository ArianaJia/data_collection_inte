/* MCP2518FD support code wraps register access, reset, DMA transfers and
 * receive/transmit helpers so the higher layers only see a ready-to-use bus.
 */

#include "mcp2518fd.h"

#include "cmsis_os.h"
#include <string.h>

typedef struct
{
  SPI_HandleTypeDef *hspi;
  volatile uint8_t transfer_done;
  volatile uint8_t transfer_error;
} MCP2518FD_SpiDmaState_t;

#define MCP2518FD_SPI_TRANSFER_BUFFER_SIZE   (2U + MCP2518FD_OBJECT_SIZE)

static HAL_StatusTypeDef MCP2518FD_Reset(MCP2518FD_Handle_t *handle);
static HAL_StatusTypeDef MCP2518FD_Read(MCP2518FD_Handle_t *handle, uint16_t address, uint8_t *data, uint16_t length);
static HAL_StatusTypeDef MCP2518FD_Write(MCP2518FD_Handle_t *handle, uint16_t address, const uint8_t *data, uint16_t length);
static HAL_StatusTypeDef MCP2518FD_ReadRegister(MCP2518FD_Handle_t *handle, uint16_t address, uint32_t *value);
static HAL_StatusTypeDef MCP2518FD_WriteRegister(MCP2518FD_Handle_t *handle, uint16_t address, uint32_t value);
static HAL_StatusTypeDef MCP2518FD_ModifyRegister(MCP2518FD_Handle_t *handle, uint16_t address, uint32_t clear_mask, uint32_t set_mask);
static HAL_StatusTypeDef MCP2518FD_SetMode(MCP2518FD_Handle_t *handle, uint32_t mode);
static HAL_StatusTypeDef MCP2518FD_WaitOscillatorReady(MCP2518FD_Handle_t *handle);
static HAL_StatusTypeDef MCP2518FD_InitRam(MCP2518FD_Handle_t *handle, uint8_t fill_value);
static HAL_StatusTypeDef MCP2518FD_ConfigRamAndFilters(MCP2518FD_Handle_t *handle);
static void MCP2518FD_Select(MCP2518FD_Handle_t *handle);
static void MCP2518FD_Deselect(MCP2518FD_Handle_t *handle);
static uint16_t MCP2518FD_CommandHeader(uint8_t command, uint16_t address);
static void MCP2518FD_StoreLe32(uint8_t *data, uint32_t value);
static uint32_t MCP2518FD_LoadLe32(const uint8_t *data);
static uint8_t MCP2518FD_DlcToBytes(uint8_t dlc);
static uint8_t MCP2518FD_LengthToDlc(uint8_t length);
static void MCP2518FD_DelayMs(uint32_t delay_ms);
static MCP2518FD_SpiDmaState_t *MCP2518FD_GetSpiDmaState(SPI_HandleTypeDef *hspi);
static HAL_StatusTypeDef MCP2518FD_WaitForSpiReady(SPI_HandleTypeDef *hspi, uint32_t timeout_ms);
static HAL_StatusTypeDef MCP2518FD_TransferDmaBlocking(MCP2518FD_Handle_t *handle, const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);

static MCP2518FD_SpiDmaState_t g_mcp2518fdSpiDmaStates[2];

HAL_StatusTypeDef MCP2518FD_Init(MCP2518FD_Handle_t *handle, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  /* Bring the external CAN controller into configuration mode and prepare RAM. */
  uint32_t value;

  if ((handle == NULL) || (hspi == NULL) || (cs_port == NULL))
  {
    return HAL_ERROR;
  }

  handle->hspi = hspi;
  handle->cs_port = cs_port;
  handle->cs_pin = cs_pin;
  handle->ready = 0U;

  MCP2518FD_Deselect(handle);
  MCP2518FD_DelayMs(2U);

  if (MCP2518FD_Reset(handle) != HAL_OK)
  {
    return HAL_ERROR;
  }
  MCP2518FD_DelayMs(2U);

  if (MCP2518FD_SetMode(handle, MCP2518FD_MODE_CONFIGURATION) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_OSC, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WaitOscillatorReady(handle) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_IOCON, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_ECCCON, MCP2518FD_ECCCON_ECCEN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_InitRam(handle, 0xFFU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_C1NBTCFG, MCP2518FD_NBTCFG_500K_40MHZ) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_C1DBTCFG, MCP2518FD_DBTCFG_CLASSIC_SAFE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_C1TDC, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_ReadRegister(handle, MCP2518FD_REG_C1CON, &value) != HAL_OK)
  {
    return HAL_ERROR;
  }
  value &= ~MCP2518FD_C1CON_REQOP_MASK;
  value |= (MCP2518FD_MODE_CONFIGURATION << MCP2518FD_C1CON_REQOP_POS) | MCP2518FD_C1CON_BRSDIS;
  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_C1CON, value) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_ConfigRamAndFilters(handle) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_C1INT, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_SetMode(handle, MCP2518FD_MODE_NORMAL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  handle->ready = 1U;
  return HAL_OK;
}

uint8_t MCP2518FD_IsReady(const MCP2518FD_Handle_t *handle)
{
  /* Expose a simple readiness check for the polling tasks. */
  if (handle == NULL)
  {
    return 0U;
  }

  return handle->ready;
}

HAL_StatusTypeDef MCP2518FD_PollReceive(MCP2518FD_Handle_t *handle, MCP2518FD_StdFrame_t *frame, uint8_t *received)
{
  /* Read one RX object from the FIFO and convert it into a compact frame. */
  uint32_t status;
  uint32_t user_address;
  uint32_t id_word;
  uint32_t control_word;
  uint8_t object[MCP2518FD_OBJECT_SIZE];
  uint8_t bytes;

  if ((handle == NULL) || (frame == NULL) || (received == NULL) || (handle->ready == 0U))
  {
    return HAL_ERROR;
  }

  *received = 0U;

  if (MCP2518FD_ReadRegister(handle, MCP2518FD_FIFO_STA(MCP2518FD_RX_FIFO), &status) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if ((status & MCP2518FD_FIFO_TFNRFNIF) == 0U)
  {
    return HAL_OK;
  }

  if (MCP2518FD_ReadRegister(handle, MCP2518FD_FIFO_UA(MCP2518FD_RX_FIFO), &user_address) != HAL_OK)
  {
    return HAL_ERROR;
  }
  user_address &= 0x0FFFU;

  if (((user_address & 0x3U) != 0U) ||
      (user_address < MCP2518FD_RAM_START_ADDR) ||
      (user_address > (MCP2518FD_RAM_END_ADDR - sizeof(object))))
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_Read(handle, (uint16_t)user_address, object, sizeof(object)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(handle, MCP2518FD_FIFO_CON(MCP2518FD_RX_FIFO),
                              MCP2518FD_RX_FIFO_CONFIG | MCP2518FD_FIFO_UINC) != HAL_OK)
  {
    return HAL_ERROR;
  }

  control_word = MCP2518FD_LoadLe32(&object[4]);
  if ((control_word & (1UL << 4)) != 0UL)
  {
    return HAL_OK;
  }

  id_word = MCP2518FD_LoadLe32(&object[0]);
  bytes = MCP2518FD_DlcToBytes((uint8_t)(control_word & 0x0FU));
  if (bytes > MCP2518FD_PAYLOAD_CAPACITY)
  {
    bytes = MCP2518FD_PAYLOAD_CAPACITY;
  }

  frame->frame_format = ((control_word & (1UL << 7)) != 0UL) ? 1U : 0U;
  frame->bit_rate_switch = ((control_word & (1UL << 6)) != 0UL) ? 1U : 0U;
  frame->esi = ((control_word & (1UL << 8)) != 0UL) ? 1U : 0U;
  frame->id = id_word & 0x7FFUL;
  frame->dlc = (uint8_t)(control_word & 0x0FU);
  frame->length = bytes;
  (void)memset(frame->data, 0, sizeof(frame->data));
  (void)memcpy(frame->data, &object[8], bytes);
  *received = 1U;

  return HAL_OK;
}

HAL_StatusTypeDef MCP2518FD_TransmitStd(MCP2518FD_Handle_t *handle, const MCP2518FD_StdFrame_t *frame)
{
  /* Serialize a standard CAN frame into the TX FIFO object format. */
  uint32_t status;
  uint32_t user_address;
  uint8_t object[MCP2518FD_OBJECT_SIZE];

  if ((handle == NULL) || (frame == NULL) || (handle->ready == 0U) || (frame->length > MCP2518FD_PAYLOAD_CAPACITY) || (frame->id > 0x7FFU))
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_ReadRegister(handle, MCP2518FD_FIFO_STA(MCP2518FD_TX_FIFO), &status) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if ((status & MCP2518FD_FIFO_TFNRFNIF) == 0U)
  {
    return HAL_BUSY;
  }

  if (MCP2518FD_ReadRegister(handle, MCP2518FD_FIFO_UA(MCP2518FD_TX_FIFO), &user_address) != HAL_OK)
  {
    return HAL_ERROR;
  }
  user_address &= 0x0FFFU;

  if (((user_address & 0x3U) != 0U) ||
      (user_address < MCP2518FD_RAM_START_ADDR) ||
      (user_address > (MCP2518FD_RAM_END_ADDR - sizeof(object))))
  {
    return HAL_ERROR;
  }

  (void)memset(object, 0, sizeof(object));
  MCP2518FD_StoreLe32(&object[0], frame->id & 0x7FFUL);
  MCP2518FD_StoreLe32(&object[4],
                      (uint32_t)MCP2518FD_LengthToDlc(frame->length) |
                      ((frame->bit_rate_switch != 0U) ? (1UL << 6) : 0UL) |
                      ((frame->frame_format != 0U) ? (1UL << 7) : 0UL));
  (void)memcpy(&object[8], frame->data, frame->length);

  if (MCP2518FD_Write(handle, (uint16_t)user_address, object, sizeof(object)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return MCP2518FD_WriteRegister(handle, MCP2518FD_FIFO_CON(MCP2518FD_TX_FIFO),
                                 MCP2518FD_TX_FIFO_CONFIG | MCP2518FD_FIFO_UINC | MCP2518FD_FIFO_TXREQ);
}

static HAL_StatusTypeDef MCP2518FD_Reset(MCP2518FD_Handle_t *handle)
{
  /* Issue the chip reset SPI command and wait for the transfer to finish. */
  uint16_t header = MCP2518FD_CommandHeader(MCP2518FD_CMD_RESET, 0x000U);
  uint8_t tx[2];
  uint8_t rx[2] = {0};
  HAL_StatusTypeDef status;

  tx[0] = (uint8_t)(header >> 8);
  tx[1] = (uint8_t)(header & 0xFFU);

  MCP2518FD_Select(handle);
  status = MCP2518FD_TransferDmaBlocking(handle, tx, rx, (uint16_t)sizeof(tx));
  MCP2518FD_Deselect(handle);

  return status;
}

static HAL_StatusTypeDef MCP2518FD_Read(MCP2518FD_Handle_t *handle, uint16_t address, uint8_t *data, uint16_t length)
{
  /* Read a register block using the SPI command header plus DMA transfer. */
  uint16_t header = MCP2518FD_CommandHeader(MCP2518FD_CMD_READ, address);
  uint8_t tx_buffer[MCP2518FD_SPI_TRANSFER_BUFFER_SIZE] = {0};
  uint8_t rx_buffer[MCP2518FD_SPI_TRANSFER_BUFFER_SIZE] = {0};
  HAL_StatusTypeDef status;

  if ((handle == NULL) || (handle->hspi == NULL) || (data == NULL) || (length == 0U) ||
      ((uint16_t)(length + 2U) > (uint16_t)sizeof(tx_buffer)))
  {
    return HAL_ERROR;
  }

  tx_buffer[0] = (uint8_t)(header >> 8);
  tx_buffer[1] = (uint8_t)(header & 0xFFU);

  MCP2518FD_Select(handle);
  status = MCP2518FD_TransferDmaBlocking(handle, tx_buffer, rx_buffer, (uint16_t)(length + 2U));
  MCP2518FD_Deselect(handle);

  if (status == HAL_OK)
  {
    (void)memcpy(data, &rx_buffer[2], length);
  }

  return status;
}

static HAL_StatusTypeDef MCP2518FD_Write(MCP2518FD_Handle_t *handle, uint16_t address, const uint8_t *data, uint16_t length)
{
  /* Write a register block using the SPI command header plus DMA transfer. */
  uint16_t header = MCP2518FD_CommandHeader(MCP2518FD_CMD_WRITE, address);
  uint8_t tx_buffer[MCP2518FD_SPI_TRANSFER_BUFFER_SIZE] = {0};
  uint8_t rx_buffer[MCP2518FD_SPI_TRANSFER_BUFFER_SIZE] = {0};
  HAL_StatusTypeDef status;

  if ((handle == NULL) || (handle->hspi == NULL) || (data == NULL) || (length == 0U) ||
      ((uint16_t)(length + 2U) > (uint16_t)sizeof(tx_buffer)))
  {
    return HAL_ERROR;
  }

  tx_buffer[0] = (uint8_t)(header >> 8);
  tx_buffer[1] = (uint8_t)(header & 0xFFU);
  (void)memcpy(&tx_buffer[2], data, length);

  MCP2518FD_Select(handle);
  status = MCP2518FD_TransferDmaBlocking(handle, tx_buffer, rx_buffer, (uint16_t)(length + 2U));
  MCP2518FD_Deselect(handle);

  return status;
}

static HAL_StatusTypeDef MCP2518FD_ReadRegister(MCP2518FD_Handle_t *handle, uint16_t address, uint32_t *value)
{
  /* Register helpers keep the caller insulated from raw byte order handling. */
  uint8_t data[4];

  if (value == NULL)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_Read(handle, address, data, sizeof(data)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *value = MCP2518FD_LoadLe32(data);
  return HAL_OK;
}

static HAL_StatusTypeDef MCP2518FD_WriteRegister(MCP2518FD_Handle_t *handle, uint16_t address, uint32_t value)
{
  /* Convert a 32-bit register value into the controller's wire format. */
  uint8_t data[4];

  MCP2518FD_StoreLe32(data, value);
  return MCP2518FD_Write(handle, address, data, sizeof(data));
}

static HAL_StatusTypeDef MCP2518FD_ModifyRegister(MCP2518FD_Handle_t *handle, uint16_t address, uint32_t clear_mask, uint32_t set_mask)
{
  /* Read-modify-write keeps bitfield updates localized and deterministic. */
  uint32_t value;

  if (MCP2518FD_ReadRegister(handle, address, &value) != HAL_OK)
  {
    return HAL_ERROR;
  }

  value &= ~clear_mask;
  value |= set_mask;

  return MCP2518FD_WriteRegister(handle, address, value);
}

static HAL_StatusTypeDef MCP2518FD_SetMode(MCP2518FD_Handle_t *handle, uint32_t mode)
{
  /* Wait until the controller reports the requested operating mode. */
  uint32_t value;
  uint32_t start = HAL_GetTick();

  if (MCP2518FD_ModifyRegister(handle,
                               MCP2518FD_REG_C1CON,
                               MCP2518FD_C1CON_REQOP_MASK,
                               mode << MCP2518FD_C1CON_REQOP_POS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  do
  {
    if (MCP2518FD_ReadRegister(handle, MCP2518FD_REG_C1CON, &value) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (((value & MCP2518FD_C1CON_OPMOD_MASK) >> MCP2518FD_C1CON_OPMOD_POS) == mode)
    {
      return HAL_OK;
    }
  } while ((uint32_t)(HAL_GetTick() - start) < MCP2518FD_MODE_TIMEOUT_MS);

  return HAL_TIMEOUT;
}

static HAL_StatusTypeDef MCP2518FD_WaitOscillatorReady(MCP2518FD_Handle_t *handle)
{
  /* Wait for the oscillator-ready bit before touching the rest of the chip. */
  uint32_t value;
  uint32_t start = HAL_GetTick();

  do
  {
    if (MCP2518FD_ReadRegister(handle, MCP2518FD_REG_OSC, &value) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if ((value & MCP2518FD_OSC_READY_MASK) == MCP2518FD_OSC_READY_MASK)
    {
      return HAL_OK;
    }
  } while ((uint32_t)(HAL_GetTick() - start) < MCP2518FD_OSC_TIMEOUT_MS);

  return HAL_TIMEOUT;
}

static HAL_StatusTypeDef MCP2518FD_InitRam(MCP2518FD_Handle_t *handle, uint8_t fill_value)
{
  /* Pre-fill the internal RAM so stale objects cannot leak into the first frames. */
  uint16_t address;
  uint8_t fill_buffer[MCP2518FD_RAM_FILL_CHUNK_SIZE];

  (void)memset(fill_buffer, fill_value, sizeof(fill_buffer));

  for (address = MCP2518FD_RAM_START_ADDR;
       address < MCP2518FD_RAM_END_ADDR;
       address = (uint16_t)(address + MCP2518FD_RAM_FILL_CHUNK_SIZE))
  {
    if (MCP2518FD_Write(handle, address, fill_buffer, sizeof(fill_buffer)) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef MCP2518FD_ConfigRamAndFilters(MCP2518FD_Handle_t *handle)
{
  /* Set FIFO/filter defaults used by the application-level CAN routing. */
  if (MCP2518FD_WriteRegister(handle, MCP2518FD_FIFO_CON(MCP2518FD_RX_FIFO), MCP2518FD_RX_FIFO_CONFIG) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(handle, MCP2518FD_FIFO_CON(MCP2518FD_TX_FIFO), MCP2518FD_TX_FIFO_CONFIG) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_C1FLTCON0, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_C1FLTOBJ0, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(handle, MCP2518FD_REG_C1MASK0, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return MCP2518FD_WriteRegister(handle, MCP2518FD_REG_C1FLTCON0, MCP2518FD_FILTER0_TO_FIFO1);
}

static void MCP2518FD_Select(MCP2518FD_Handle_t *handle)
{
  /* Assert chip select before starting an SPI transaction. */
  HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_RESET);
}

static void MCP2518FD_Deselect(MCP2518FD_Handle_t *handle)
{
  /* Release chip select after the SPI transaction has completed. */
  HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_SET);
}

static uint16_t MCP2518FD_CommandHeader(uint8_t command, uint16_t address)
{
  /* Build the 16-bit SPI command header expected by the controller. */
  return (uint16_t)((((uint16_t)command) << 12) | (address & 0x0FFFU));
}

static void MCP2518FD_StoreLe32(uint8_t *data, uint32_t value)
{
  /* Store a 32-bit value in the controller's little-endian byte order. */
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
  data[2] = (uint8_t)((value >> 16) & 0xFFU);
  data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static uint32_t MCP2518FD_LoadLe32(const uint8_t *data)
{
  /* Load a 32-bit value from the controller's little-endian byte order. */
  return ((uint32_t)data[3] << 24) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[1] << 8) |
         (uint32_t)data[0];
}

static uint8_t MCP2518FD_DlcToBytes(uint8_t dlc)
{
  /* Convert DLC to payload length using the CAN-FD lookup table. */
  static const uint8_t dlc_to_bytes[] = {
      0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
      8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
  };

  if (dlc < (sizeof(dlc_to_bytes) / sizeof(dlc_to_bytes[0])))
  {
    return dlc_to_bytes[dlc];
  }

  return 0U;
}

static uint8_t MCP2518FD_LengthToDlc(uint8_t length)
{
  /* Clamp the application payload length to a standard CAN DLC. */
  if (length >= 8U)
  {
    return 8U;
  }

  return length;
}

static void MCP2518FD_DelayMs(uint32_t delay_ms)
{
  /* Use RTOS delay when available, otherwise fall back to HAL delay. */
  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

static MCP2518FD_SpiDmaState_t *MCP2518FD_GetSpiDmaState(SPI_HandleTypeDef *hspi)
{
  /* Track per-SPI DMA completion flags so multiple controllers can coexist. */
  size_t i;

  if (hspi == NULL)
  {
    return NULL;
  }

  for (i = 0U; i < (sizeof(g_mcp2518fdSpiDmaStates) / sizeof(g_mcp2518fdSpiDmaStates[0])); ++i)
  {
    if ((g_mcp2518fdSpiDmaStates[i].hspi == hspi) || (g_mcp2518fdSpiDmaStates[i].hspi == NULL))
    {
      g_mcp2518fdSpiDmaStates[i].hspi = hspi;
      return &g_mcp2518fdSpiDmaStates[i];
    }
  }

  return NULL;
}

static HAL_StatusTypeDef MCP2518FD_WaitForSpiReady(SPI_HandleTypeDef *hspi, uint32_t timeout_ms)
{
  /* Wait until the HAL SPI layer is ready for another DMA transfer. */
  uint32_t start = HAL_GetTick();

  while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
  {
    if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms)
    {
      return HAL_TIMEOUT;
    }
    MCP2518FD_DelayMs(1U);
  }

  return HAL_OK;
}

static HAL_StatusTypeDef MCP2518FD_TransferDmaBlocking(MCP2518FD_Handle_t *handle,
                                                       const uint8_t *tx_data,
                                                       uint8_t *rx_data,
                                                       uint16_t length)
{
  /* Launch a full-duplex DMA transfer and block until the callback fires. */
  MCP2518FD_SpiDmaState_t *dma_state;
  HAL_StatusTypeDef status;
  uint32_t start = HAL_GetTick();

  if ((handle == NULL) || (handle->hspi == NULL) || (tx_data == NULL) || (rx_data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  dma_state = MCP2518FD_GetSpiDmaState(handle->hspi);
  if (dma_state == NULL)
  {
    return HAL_ERROR;
  }

  status = MCP2518FD_WaitForSpiReady(handle->hspi, MCP2518FD_SPI_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  dma_state->transfer_done = 0U;
  dma_state->transfer_error = 0U;

  status = HAL_SPI_TransmitReceive_DMA(handle->hspi, (uint8_t *)tx_data, rx_data, length);
  if (status != HAL_OK)
  {
    return status;
  }

  while ((dma_state->transfer_done == 0U) && (dma_state->transfer_error == 0U))
  {
    if ((uint32_t)(HAL_GetTick() - start) >= MCP2518FD_SPI_TIMEOUT_MS)
    {
      (void)HAL_SPI_Abort(handle->hspi);
      return HAL_TIMEOUT;
    }
    MCP2518FD_DelayMs(1U);
  }

  if (dma_state->transfer_error != 0U)
  {
    (void)HAL_SPI_DMAStop(handle->hspi);
    return HAL_ERROR;
  }

  return HAL_OK;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Mark the matching SPI DMA transfer as finished. */
  MCP2518FD_SpiDmaState_t *dma_state = MCP2518FD_GetSpiDmaState(hspi);

  if (dma_state != NULL)
  {
    dma_state->transfer_done = 1U;
    dma_state->transfer_error = 0U;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  /* Mark the matching SPI DMA transfer as failed. */
  MCP2518FD_SpiDmaState_t *dma_state = MCP2518FD_GetSpiDmaState(hspi);

  if (dma_state != NULL)
  {
    dma_state->transfer_error = 1U;
    dma_state->transfer_done = 0U;
  }
}
