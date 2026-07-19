/*
 * mcp2518fd.h
 *
 *  Created on: Jul 6, 2026
 *      Author: ArianaJia
 */

#ifndef INC_MCP2518FD_H_
#define INC_MCP2518FD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define MCP2518FD_CMD_RESET                  0x0U
#define MCP2518FD_CMD_WRITE                  0x2U
#define MCP2518FD_CMD_READ                   0x3U

#define MCP2518FD_REG_C1CON                  0x000U
#define MCP2518FD_REG_C1NBTCFG               0x004U
#define MCP2518FD_REG_C1DBTCFG               0x008U
#define MCP2518FD_REG_C1TDC                  0x00CU
#define MCP2518FD_REG_C1INT                  0x01CU
#define MCP2518FD_REG_C1FLTCON0              0x1D0U
#define MCP2518FD_REG_C1FLTOBJ0              0x1F0U
#define MCP2518FD_REG_C1MASK0                0x1F4U
#define MCP2518FD_REG_C1FLTCON(index)        (MCP2518FD_REG_C1FLTCON0 + (((uint16_t)(index)) * 4U))
#define MCP2518FD_REG_C1FLTOBJ(index)        (MCP2518FD_REG_C1FLTOBJ0 + (((uint16_t)(index)) * 8U))
#define MCP2518FD_REG_C1MASK(index)          (MCP2518FD_REG_C1MASK0 + (((uint16_t)(index)) * 8U))
#define MCP2518FD_REG_OSC                    0xE00U
#define MCP2518FD_REG_IOCON                  0xE04U
#define MCP2518FD_REG_ECCCON                 0xE0CU

#define MCP2518FD_RAM_START_ADDR             0x400U
#define MCP2518FD_RAM_END_ADDR               0xC00U
#define MCP2518FD_RAM_FILL_CHUNK_SIZE        32U

#define MCP2518FD_FIFO_CON(fifo)             (0x05CU + (((uint16_t)(fifo) - 1U) * 0x0CU))
#define MCP2518FD_FIFO_STA(fifo)             (MCP2518FD_FIFO_CON(fifo) + 0x04U)
#define MCP2518FD_FIFO_UA(fifo)              (MCP2518FD_FIFO_CON(fifo) + 0x08U)

#define MCP2518FD_C1CON_REQOP_POS            24U
#define MCP2518FD_C1CON_REQOP_MASK           (0x7UL << MCP2518FD_C1CON_REQOP_POS)
#define MCP2518FD_C1CON_OPMOD_POS            21U
#define MCP2518FD_C1CON_OPMOD_MASK           (0x7UL << MCP2518FD_C1CON_OPMOD_POS)
#define MCP2518FD_C1CON_BRSDIS               (1UL << 12)

#define MCP2518FD_MODE_NORMAL                0x0UL
#define MCP2518FD_MODE_CONFIGURATION         0x4UL

#define MCP2518FD_OSC_PLLRDY                 (1UL << 8)
#define MCP2518FD_OSC_OSCRDY                 (1UL << 10)
#define MCP2518FD_OSC_SCLKRDY                (1UL << 12)
#define MCP2518FD_OSC_READY_MASK             MCP2518FD_OSC_OSCRDY

#define MCP2518FD_ECCCON_ECCEN               (1UL << 0)

#define MCP2518FD_FIFO_PLSIZE_64             (7UL << 29)
#define MCP2518FD_FIFO_FSIZE(depth)          ((((uint32_t)(depth) - 1UL) & 0x1FUL) << 24)
#define MCP2518FD_FIFO_TXREQ                 (1UL << 9)
#define MCP2518FD_FIFO_UINC                  (1UL << 8)
#define MCP2518FD_FIFO_TXEN                  (1UL << 7)
#define MCP2518FD_FIFO_TFNRFNIF              (1UL << 0)

#define MCP2518FD_RX_FIFO                    1U
#define MCP2518FD_TX_FIFO                    2U
#define MCP2518FD_RX_FIFO_DEPTH              8U
#define MCP2518FD_TX_FIFO_DEPTH              8U
#define MCP2518FD_PAYLOAD_CAPACITY           64U
#define MCP2518FD_OBJECT_SIZE                (8U + MCP2518FD_PAYLOAD_CAPACITY)
#define MCP2518FD_RX_FIFO_CONFIG             (MCP2518FD_FIFO_PLSIZE_64 | MCP2518FD_FIFO_FSIZE(MCP2518FD_RX_FIFO_DEPTH))
#define MCP2518FD_TX_FIFO_CONFIG             (MCP2518FD_FIFO_PLSIZE_64 | MCP2518FD_FIFO_FSIZE(MCP2518FD_TX_FIFO_DEPTH) | MCP2518FD_FIFO_TXEN)

#define MCP2518FD_FILTER0_TO_FIFO1           0x81U
#define MCP2518FD_FILTER_TO_FIFO1            0x81U
#define MCP2518FD_FILTER_COUNT               32U
#define MCP2518FD_FILTERS_PER_CON_REGISTER   4U
#define MCP2518FD_STD_ID_MASK                0x7FFUL
#define MCP2518FD_FILTER_MIDE                (1UL << 30)
#define MCP2518FD_STD_FILTER_MASK            (MCP2518FD_FILTER_MIDE | MCP2518FD_STD_ID_MASK)

#define MCP2518FD_SPI_TIMEOUT_MS             50U
#define MCP2518FD_MODE_TIMEOUT_MS            20U
#define MCP2518FD_OSC_TIMEOUT_MS             20U

/* Nominal Bit Timing: 500 kbps @ 40 MHz crystal, sample point 75%.
 * Fosc = 40 MHz, TQ = 2*(BRP+1)/Fosc = 2*2/40M = 100 ns
 * NBT  = 1(Sync) + (TSEG1+1) + (TSEG2+1) = TSEG1 + TSEG2 + 3 = 20 TQ = 2 us
 * Sample Point = (1 + TSEG1 + 1) / NBT = (1+13+1)/20 = 75 %
 * SJW = 4 TQ <= PhaseSeg2 (5 TQ)  OK
 *
 * Register fields (bits):  [31:24]=SJW-1  [23:16]=TSEG2-1  [15:8]=TSEG1-1  [7:0]=BRP
 *                        (3)           (4)            (13)           (1)
 */
#define MCP2518FD_NBTCFG_500K_40MHZ          ((3UL << 24) | (4UL << 16) | (13UL << 8) | 1UL)
#define MCP2518FD_DBTCFG_CLASSIC_SAFE        0x00000000UL


typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  uint8_t ready;
} MCP2518FD_Handle_t;

typedef struct
{
  uint8_t frame_format;
  uint8_t bit_rate_switch;
  uint8_t esi;
  uint32_t id;
  uint8_t dlc;
  uint8_t length;
  uint8_t data[64];
} MCP2518FD_StdFrame_t;

/* Diagnostic log callback – supplied by the application (freertos.c) before Init. */
extern void App_DebugLogString(const char *text);

void MCP2518FD_SetStdFilters(MCP2518FD_Handle_t *handle, const uint16_t *std_filter_ids, uint8_t std_filter_count);
HAL_StatusTypeDef MCP2518FD_Init(MCP2518FD_Handle_t *handle, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
uint8_t MCP2518FD_IsReady(const MCP2518FD_Handle_t *handle);
HAL_StatusTypeDef MCP2518FD_PollReceive(MCP2518FD_Handle_t *handle, MCP2518FD_StdFrame_t *frame, uint8_t *received);
HAL_StatusTypeDef MCP2518FD_TransmitStd(MCP2518FD_Handle_t *handle, const MCP2518FD_StdFrame_t *frame);



#ifdef __cplusplus
}
#endif

#endif /* INC_MCP2518FD_H_ */
