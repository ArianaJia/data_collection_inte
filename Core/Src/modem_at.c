/* Standard AT/TCP/MQTT backend for modem_transport.h. */
#include "app_config.h"
#include "modem_transport.h"

#if APP_UPLINK_ENABLED

#include "cmsis_os2.h"
#include "app_log.h"
#include "modem_transport.h"
#include "usart.h"
#include "mqtt_credentials.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"

/* USART1 now follows the final standard-AT MQTT uplink route:
 * broker 82.157.204.124:1883, topic fsae/telemetry, payload = raw
 * TelemetryFrame protobuf. USART1 therefore stays in AT/TCP client mode and
 * the business task pushes each encoded protobuf through AT+CIPSEND.
 */
#define Y100M_AT_REPLY_BUFFER_SIZE        512U
#define Y100M_AT_LOG_BUFFER_SIZE          192U
#define Y100M_AT_CMD_TIMEOUT_MS          5000U
#define Y100M_AT_RESET_PULSE_MS          1000U
#define Y100M_AT_BOOT_WAIT_MS            5000U
#define Y100M_AT_MAX_HANDSHAKE_ATTEMPTS     2U
#define Y100M_AT_TCP_OPEN_TIMEOUT_MS    30000U
#define Y100M_AT_PDP_TIMEOUT_MS         15000U
#define Y100M_AT_MQTT_CONNACK_TIMEOUT_MS 15000U
#define Y100M_MQTT_BROKER                "bitfsae.com"//新网址
#define Y100M_MQTT_PORT                 1883U
#define Y100M_MQTT_CLIENT_ID            "vehicle_02" //新客户端ID
#define Y100M_MQTT_USERNAME "vehicle_gateway"
#define Y100M_MQTT_PASSWORD "Fsae2026"
/* MQTT TX must hold the MQTT fixed header, topic header/name, and one protobuf
 * payload. The thermal summary path has already reached 271 bytes on wire, so
 * keep this comfortably above the former 256-byte bring-up buffer while still
 * preserving the explicit packet-too-large guard below.
 */
#define Y100M_MQTT_TX_BUFFER_SIZE      1024U
/* Text logs share the USART3 queue. Raw hex dumps are optional. */
/* Restore the full startup sequence, including socket and PDP cleanup. */
#define Y100M_CLOSE_OLD_SESSION_ENABLED    1U
#define Y100M_DEBUG_RAW_HEX_ENABLED        1U

typedef struct
{
  const char *name;
  const char *command;
  const char *expected;
  uint32_t timeout_ms;
} Y100M_AtStep_t;

static uint8_t g_y100mBootstrapDone = 0U;
static uint8_t g_y100mBootstrapOk = 0U;
static char g_y100mAtReplyBuffer[Y100M_AT_REPLY_BUFFER_SIZE];
static char g_y100mLogBuffer[Y100M_AT_LOG_BUFFER_SIZE];

/* ── USART1 RX DMA for AT response reception ── */
#define Y100M_AT_RX_DMA_BUFFER_SIZE       512U
static uint8_t  g_y100mUsart1RxDmaBuffer[Y100M_AT_RX_DMA_BUFFER_SIZE];
static uint8_t  g_y100mMqttTxBuffer[Y100M_MQTT_TX_BUFFER_SIZE];
/* Single ISR producer / single modem-task consumer. DMA stays running across replies. */
#define Y100M_RX_RING_SIZE 2048U
static uint8_t g_y100mRxRing[Y100M_RX_RING_SIZE];
static volatile uint16_t g_y100mRxHead;
static volatile uint16_t g_y100mRxTail;
static volatile uint8_t g_y100mRxOverflow;
static uint16_t g_y100mDmaReadPos;

void Modem_OnRxEvent(uint16_t size)
{
  /* Sample the current DMA position for IDLE/half/full events alike. This also
   * avoids processing an older pending DMA event after a newer IDLE event. */
  uint16_t pos = (uint16_t)(Y100M_AT_RX_DMA_BUFFER_SIZE -
                            __HAL_DMA_GET_COUNTER(huart1.hdmarx));
  (void)size;
  if (pos == Y100M_AT_RX_DMA_BUFFER_SIZE) pos = 0U;
  while (g_y100mDmaReadPos != pos)
  {
    uint16_t next = (uint16_t)((g_y100mRxHead + 1U) % Y100M_RX_RING_SIZE);
    if (next == g_y100mRxTail)
    {
      g_y100mRxOverflow = 1U;
    }
    else
    {
      g_y100mRxRing[g_y100mRxHead] = g_y100mUsart1RxDmaBuffer[g_y100mDmaReadPos];
      __DMB();
      g_y100mRxHead = next;
    }
    g_y100mDmaReadPos = (uint16_t)((g_y100mDmaReadPos + 1U) % Y100M_AT_RX_DMA_BUFFER_SIZE);
  }
}

static uint8_t Y100M_ReadByte(uint8_t *value)
{
  if (g_y100mRxTail == g_y100mRxHead) return 0U;
  __DMB();
  *value = g_y100mRxRing[g_y100mRxTail];
  __DMB();
  g_y100mRxTail = (uint16_t)((g_y100mRxTail + 1U) % Y100M_RX_RING_SIZE);
  return 1U;
}

/* AT bootstrap sequence; application tasks use the transport interface. */
static const Y100M_AtStep_t g_y100mBootstrapSteps[] =
{
  { "Disable echo",          "ATE0",      "OK",      1000U },
  { "Check SIM ready",       "AT+CPIN?",  "READY",   2000U },
  { "Check signal quality",  "AT+CSQ",    "+CSQ:",   2000U },
  { "Check LTE register",    "AT+CEREG?", "+CEREG:", 5000U },
  { "Check packet attach",   "AT+CGATT?", "+CGATT:", 5000U },

};

#if (Y100M_DEBUG_RAW_HEX_ENABLED != 0U)
static void Y100M_LogRxChunkHex(const char *prefix, const uint8_t *data, uint16_t length);
#else
#define Y100M_LogRxChunkHex(prefix, data, length) ((void)0)
#endif
static void Y100M_Uart1DrainRx(void);
static HAL_StatusTypeDef Y100M_StartReplyRxWindow(void);
static HAL_StatusTypeDef Y100M_WaitReply(const char *expected, uint32_t timeout_ms, char *reply_buffer, uint16_t reply_capacity);
static HAL_StatusTypeDef Y100M_SendAtExpect(const char *command, const char *expected, uint32_t timeout_ms);
static HAL_StatusTypeDef Y100M_SendRawPayload(const uint8_t *data, uint16_t length, uint32_t timeout_ms);
static uint8_t Y100M_EncodeMqttRemainingLength(uint32_t value, uint8_t *buffer, uint8_t capacity);
static HAL_StatusTypeDef Y100M_PdpActivate(void);
static HAL_StatusTypeDef Y100M_GetLocalIp(void);
#if (Y100M_CLOSE_OLD_SESSION_ENABLED != 0U)
static HAL_StatusTypeDef Y100M_CloseOldSession(void);
#endif
static HAL_StatusTypeDef Y100M_OpenTcpSession(void);
static int8_t Y100M_CheckConnackWindow(const uint8_t *data, uint16_t length);
static HAL_StatusTypeDef Y100M_MqttConnect(void);
static void Y100M_ResetPulse(void);
static HAL_StatusTypeDef Y100M_RunBootstrapSteps(void);
void Modem_Init(void);

/* ── DMA-based USART1 send for AT command sequences ── */
static HAL_StatusTypeDef Y100M_Uart1DmaSend(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
  uint32_t start_tick;
  HAL_StatusTypeDef status;

  if ((data == NULL) || (length == 0U) || (huart1.Instance == NULL))
  {
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while ((HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) ||
         (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX))
  {
    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      return HAL_TIMEOUT;
    }
    osDelay(1);
  }

  status = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, length);
  if (status != HAL_OK)
  {
    return status;
  }

  start_tick = HAL_GetTick();
  while ((HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) ||
         (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX))
  {
    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      (void)HAL_UART_AbortTransmit(&huart1);
      return HAL_TIMEOUT;
    }
    osDelay(1);
  }

  return HAL_OK;
}

/* ── RTOS-aware millisecond delay ── */
static void Y100M_DelayMs(uint32_t delay_ms)
{
  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

/* Forward a text log to the USART3 debug output path. */

#if (Y100M_DEBUG_RAW_HEX_ENABLED != 0U)
static void Y100M_LogRxChunkHex(const char *prefix, const uint8_t *data, uint16_t length)
{
  uint16_t log_offset;
  uint16_t index;

  if ((prefix == NULL) || (data == NULL) || (length == 0U))
  {
    return;
  }

  log_offset = (uint16_t)snprintf(g_y100mLogBuffer,
                                  sizeof(g_y100mLogBuffer),
                                  "%s len=%u hex:",
                                  prefix,
                                  (unsigned int)length);
  for (index = 0U; (index < length) && (log_offset + 4U < sizeof(g_y100mLogBuffer)); index++)
  {
    log_offset += (uint16_t)snprintf(&g_y100mLogBuffer[log_offset],
                                     sizeof(g_y100mLogBuffer) - log_offset,
                                     " %02X",
                                     data[index]);
  }
  if (log_offset + 3U < sizeof(g_y100mLogBuffer))
  {
    g_y100mLogBuffer[log_offset++] = '\r';
    g_y100mLogBuffer[log_offset++] = '\n';
    g_y100mLogBuffer[log_offset] = '\0';
  }
  App_LogModem(g_y100mLogBuffer);
}
#endif

/* Drain only before a new AT command, never between SEND OK and CONNACK. */
static void Y100M_Uart1DrainRx(void)
{
  uint32_t start = HAL_GetTick();
  uint32_t quiet = start;
  uint8_t value;
  while ((HAL_GetTick() - start) < 500U && (HAL_GetTick() - quiet) < 50U)
  {
    while (Y100M_ReadByte(&value)) quiet = HAL_GetTick();
    osDelay(1);
  }
  g_y100mRxOverflow = 0U;
}

static HAL_StatusTypeDef Y100M_StartReplyRxWindow(void)
{
  if (huart1.RxState == HAL_UART_STATE_BUSY_RX) return HAL_OK;
  (void)HAL_UART_AbortReceive(&huart1);
  g_y100mRxHead = 0U;
  g_y100mRxTail = 0U;
  g_y100mDmaReadPos = 0U;
  g_y100mRxOverflow = 0U;
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  return HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_y100mUsart1RxDmaBuffer,
                                     Y100M_AT_RX_DMA_BUFFER_SIZE);
}

/* Accept only a complete CR/LF-terminated, four-octet, nonzero IPv4 line. */
static uint8_t Y100M_HasIpv4Line(const char *text)
{
  const char *line = text;
  while (*line != '\0')
  {
    const char *end = line;
    const char *p = line;
    uint8_t valid = 1U;
    uint32_t combined = 0U;
    while (*end != '\0' && *end != '\r' && *end != '\n') ++end;
    if (*end == '\0') return 0U;
    for (uint8_t part = 0U; part < 4U; ++part)
    {
      uint32_t value = 0U;
      uint8_t digits = 0U;
      while (p < end && *p >= '0' && *p <= '9')
      {
        value = value * 10U + (uint32_t)(*p++ - '0');
        if (++digits > 3U) { valid = 0U; break; }
      }
      if (!valid || digits == 0U || value > 255U) { valid = 0U; break; }
      combined |= value;
      if (part < 3U && (p >= end || *p++ != '.')) { valid = 0U; break; }
    }
    if (valid && p == end && combined != 0U) return 1U;
    line = end + 1;
  }
  return 0U;
}

/* Consume exactly through the match, preserving following binary bytes. */
static HAL_StatusTypeDef Y100M_WaitReply(const char *expected, uint32_t timeout_ms,
                                       char *reply_buffer, uint16_t reply_capacity)
{
  uint32_t start = HAL_GetTick();
  uint16_t length = 0U;
  uint8_t value;
  if (reply_buffer == NULL || reply_capacity < 2U) return HAL_ERROR;
  reply_buffer[0] = '\0';
  while ((HAL_GetTick() - start) < timeout_ms)
  {
    if (g_y100mRxOverflow || huart1.RxState != HAL_UART_STATE_BUSY_RX)
    {
      App_LogModem("[4G] RX overflow or UART receive stopped\r\n");
      return HAL_ERROR;
    }
    if (!Y100M_ReadByte(&value)) { osDelay(1); continue; }
    if (length >= reply_capacity - 1U) return HAL_ERROR;
    reply_buffer[length++] = (char)value;
    reply_buffer[length] = '\0';
    if (strstr(reply_buffer, "ERROR") || strstr(reply_buffer, "CONNECT FAIL"))
    {
      /* Wait for the rest of the error line so the numeric code is retained. */
      if (value == '\r' || value == '\n') return HAL_ERROR;
      continue;
    }
    if (expected != NULL && strcmp(expected, "<IPv4>") == 0)
    {
      if (Y100M_HasIpv4Line(reply_buffer)) return HAL_OK;
    }
    else if (expected != NULL && strstr(reply_buffer, expected) != NULL)
    {
      /* Query prefixes such as +CSQ: are not complete response lines.
       * The CIPSEND prompt has no line terminator. SEND OK must leave every
       * following binary CONNACK byte available to the MQTT receiver. */
      if (strcmp(expected, ">") == 0 || strcmp(expected, "SEND OK") == 0 ||
          value == '\r' || value == '\n')
        return HAL_OK;
    }
  }
  return HAL_TIMEOUT;
}

/* Send one AT command and confirm that the expected response arrives. */
static HAL_StatusTypeDef Y100M_SendAtExpect(const char *command, const char *expected, uint32_t timeout_ms)
{
  HAL_StatusTypeDef tx_status;
  HAL_StatusTypeDef rx_status;

  if ((command == NULL) || (command[0] == '\0'))
  {
    return HAL_OK;
  }

  rx_status = Y100M_StartReplyRxWindow();
  if (rx_status != HAL_OK)
  {
    return rx_status;
  }

  Y100M_Uart1DrainRx();

  

  tx_status = Y100M_Uart1DmaSend((const uint8_t *)command, (uint16_t)strlen(command), timeout_ms);
  if (tx_status != HAL_OK)
  {
    return tx_status;
  }

  tx_status = Y100M_Uart1DmaSend((const uint8_t *)"\r\n", 2U, timeout_ms);
  if (tx_status != HAL_OK)
  {
    return tx_status;
  }

  rx_status = Y100M_WaitReply(expected,
                              timeout_ms,
                              g_y100mAtReplyBuffer,
                              (uint16_t)sizeof(g_y100mAtReplyBuffer));

  

  return rx_status;
}

static HAL_StatusTypeDef Y100M_SendRawPayload(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
  HAL_StatusTypeDef tx_status;
  HAL_StatusTypeDef rx_status;
  char command[32];

  if ((data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  (void)snprintf(command, sizeof(command), "AT+CIPSEND=%u", (unsigned int)length);
  tx_status = Y100M_SendAtExpect(command, ">", timeout_ms);
  if (tx_status != HAL_OK)
  {
    return tx_status;
  }

  rx_status = Y100M_StartReplyRxWindow();
  if (rx_status != HAL_OK)
  {
    return rx_status;
  }

  tx_status = Y100M_Uart1DmaSend(data, length, timeout_ms);
  if (tx_status != HAL_OK)
  {
    return tx_status;
  }

  rx_status = Y100M_WaitReply("SEND OK",
                              timeout_ms,
                              g_y100mAtReplyBuffer,
                              (uint16_t)sizeof(g_y100mAtReplyBuffer));
  if (rx_status == HAL_OK)
  {
    Y100M_LogRxChunkHex("[4G][SENDRAW]", (const uint8_t *)g_y100mAtReplyBuffer, (uint16_t)strlen(g_y100mAtReplyBuffer));
    App_LogModem("[4G] raw payload SEND OK\r\n");
  }

  return rx_status;
}

static uint8_t Y100M_EncodeMqttRemainingLength(uint32_t value, uint8_t *buffer, uint8_t capacity)
{
  uint8_t count = 0U;

  if ((buffer == NULL) || (capacity == 0U))
  {
    return 0U;
  }

  do
  {
    uint8_t digit = (uint8_t)(value % 128U);
    value /= 128U;
    if (value > 0U)
    {
      digit |= 0x80U;
    }
    if (count >= capacity)
    {
      return 0U;
    }
    buffer[count++] = digit;
  } while (value > 0U);

  return count;
}

static HAL_StatusTypeDef Y100M_PdpActivate(void)
{
  HAL_StatusTypeDef status;

  status = Y100M_GetLocalIp();
  if (status != HAL_OK && strstr(g_y100mAtReplyBuffer, "ERROR") == NULL)
  {
    /* A missing/truncated response is not evidence that PDP is inactive. */
    App_LogModem("[4G] IP reply incomplete; retry query without changing PDP\r\n");
    status = Y100M_GetLocalIp();
    if (status != HAL_OK && strstr(g_y100mAtReplyBuffer, "ERROR") == NULL)
      return status;
  }
  if (status != HAL_OK)
  {
    status = Y100M_SendAtExpect("AT+CSTT=\"CMNET\"", "OK", Y100M_AT_CMD_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      /* Match the successful PC script strategy: a CSTT failure does not end
       * the flow because some modem states keep a valid PDP profile already.
       */
      App_LogModem("[4G] AT+CSTT failed, continue with existing PDP profile\r\n");
    }
  }
  else
  {
    App_LogModem("[4G] reuse existing IP context\r\n");
    return HAL_OK;
  }

  status = Y100M_SendAtExpect("AT+CIICR", "OK", Y100M_AT_PDP_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    /* The PC script does not treat CIICR failure as fatal if CIFSR can still
     * produce an IP afterwards. Keep the log and continue to the final check.
     */
    App_LogModem("[4G] AT+CIICR failed, retry local IP query\r\n");
  }

  status = Y100M_GetLocalIp();
  if (status == HAL_OK)
  {
    return HAL_OK;
  }

  App_LogModem("[4G] local IP unavailable after PDP attempts\r\n");
  return status;
}

#if (Y100M_CLOSE_OLD_SESSION_ENABLED != 0U)
static HAL_StatusTypeDef Y100M_CloseOldSession(void)
{
  App_LogModem("[4G] close previous TCP session if any\r\n");
  (void)Y100M_SendAtExpect("AT+CIPCLOSE", "OK", Y100M_AT_CMD_TIMEOUT_MS);
  Y100M_DelayMs(1000U);
  (void)Y100M_SendAtExpect("AT+CIPSHUT", "OK", Y100M_AT_CMD_TIMEOUT_MS);
  Y100M_DelayMs(2000U);
  Y100M_Uart1DrainRx();
  return HAL_OK;
}

#endif

static HAL_StatusTypeDef Y100M_GetLocalIp(void)
{
  HAL_StatusTypeDef status = Y100M_SendAtExpect("AT+CIFSR", "<IPv4>", Y100M_AT_CMD_TIMEOUT_MS);

  if (status == HAL_OK)
  {
    (void)snprintf(g_y100mLogBuffer,
                   sizeof(g_y100mLogBuffer),
                   "[4G] local IP: %.170s\r\n",
                   g_y100mAtReplyBuffer);
    App_LogModem(g_y100mLogBuffer);
  }

  return status;
}

static int8_t Y100M_CheckConnackWindow(const uint8_t *data, uint16_t length)
{
  uint16_t index;

  if ((data == NULL) || (length < 4U))
  {
    return 0;
  }

  for (index = 0U; index + 3U < length; index++)
  {
    if ((data[index] == 0x20U) && (data[index + 1U] == 0x02U) &&
        (data[index + 2U] == 0x00U) && (data[index + 3U] <= 0x05U))
    {
      if (data[index + 3U] == 0x00U)
      {
        App_LogModem("[4G] MQTT CONNACK accepted\r\n");
        return 1;
      }

      (void)snprintf(g_y100mLogBuffer,
                     sizeof(g_y100mLogBuffer),
                     "[4G] MQTT CONNACK reject code=0x%02X\r\n",
                     data[index + 3U]);
      App_LogModem(g_y100mLogBuffer);
      return -1;
    }
  }

  return 0;
}

static HAL_StatusTypeDef Y100M_OpenTcpSession(void)
{
  char command[96];
  (void)snprintf(command, sizeof(command), "AT+CIPSTART=\"TCP\",\"%s\",%u",
                 Y100M_MQTT_BROKER, (unsigned int)Y100M_MQTT_PORT);
  /* Immediate OK only acknowledges the command, not the TCP connection. */
  return Y100M_SendAtExpect(command, "CONNECT OK", Y100M_AT_TCP_OPEN_TIMEOUT_MS);
}

static HAL_StatusTypeDef Y100M_MqttConnect(void)
{
  HAL_StatusTypeDef status;
  const char *client_id = Y100M_MQTT_CLIENT_ID;
  uint16_t client_id_length = (uint16_t)strlen(client_id);
  const char *username = Y100M_MQTT_USERNAME;
  const char *password = Y100M_MQTT_PASSWORD;
  uint16_t username_length = (uint16_t)strlen(username);
  uint16_t password_length = (uint16_t)strlen(password);
  uint32_t connect_length = 10U + 2U + client_id_length
                          + 2U + username_length + 2U + password_length;
  uint8_t remaining_length[4];
  uint8_t encoded_count;
  uint8_t *packet = g_y100mMqttTxBuffer;
  uint16_t packet_length;
  uint16_t offset = 0U;

  encoded_count = Y100M_EncodeMqttRemainingLength(connect_length,
                                                  remaining_length,
                                                  (uint8_t)sizeof(remaining_length));
  if (encoded_count == 0U)
  {
    return HAL_ERROR;
  }

  if ((1U + encoded_count + connect_length) > Y100M_MQTT_TX_BUFFER_SIZE)
  {
    return HAL_ERROR;
  }

  packet_length = (uint16_t)(1U + encoded_count + connect_length);
  packet[offset++] = 0x10U;
  (void)memcpy(&packet[offset], remaining_length, encoded_count);
  offset = (uint16_t)(offset + encoded_count);
  packet[offset++] = 0x00U;
  packet[offset++] = 0x04U;
  packet[offset++] = 'M';
  packet[offset++] = 'Q';
  packet[offset++] = 'T';
  packet[offset++] = 'T';
  packet[offset++] = 0x04U;
  packet[offset++] = 0xC2U; /* Username, password and Clean Session. */
  packet[offset++] = 0x00U;
  packet[offset++] = 0x3CU;
  packet[offset++] = (uint8_t)(client_id_length >> 8);
  packet[offset++] = (uint8_t)(client_id_length & 0xFFU);
  (void)memcpy(&packet[offset], client_id, client_id_length);
  offset = (uint16_t)(offset + client_id_length);
  packet[offset++] = (uint8_t)(username_length >> 8);
  packet[offset++] = (uint8_t)(username_length & 0xFFU);
  (void)memcpy(&packet[offset], username, username_length);
  offset = (uint16_t)(offset + username_length);
  packet[offset++] = (uint8_t)(password_length >> 8);
  packet[offset++] = (uint8_t)(password_length & 0xFFU);
  (void)memcpy(&packet[offset], password, password_length);

  status = Y100M_OpenTcpSession();
  if (status != HAL_OK)
  {
    return status;
  }

  /* The module may complete socket setup asynchronously after the immediate
   * OK. Match the successful PC script behavior and leave a short settle time
   * before requesting CIPSEND for the MQTT CONNECT packet.
   */
  Y100M_DelayMs(1000U);

  status = Y100M_SendRawPayload(packet,
                                packet_length,
                                Y100M_AT_CMD_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  App_LogModem("[4G] MQTT CONNECT payload accepted by modem\r\n");

  /* SEND OK consumption leaves all following bytes in the ring, including a
   * CONNACK arriving in the same DMA event or fragmented across many events. */
  {
    uint8_t window[4] = {0};
    uint16_t used = 0U;
    uint8_t value;
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < Y100M_AT_MQTT_CONNACK_TIMEOUT_MS)
    {
      if (g_y100mRxOverflow || huart1.RxState != HAL_UART_STATE_BUSY_RX) return HAL_ERROR;
      if (!Y100M_ReadByte(&value)) { osDelay(1); continue; }
      if (used == sizeof(window))
      {
        (void)memmove(window, window + 1, sizeof(window) - 1U);
        --used;
      }
      window[used++] = value;
      int8_t result = Y100M_CheckConnackWindow(window, used);
      if (result != 0) return result > 0 ? HAL_OK : HAL_ERROR;
    }
  }

  App_LogModem("[4G] MQTT CONNACK timeout\r\n");
  return HAL_TIMEOUT;
}

/* Pulse the external reset pin to force the 4G module to reboot. */
static void Y100M_ResetPulse(void)
{
  App_LogModem("[4G] AT bootstrap failed, pulse RST_4G high for 1s\r\n");
  HAL_GPIO_WritePin(RST_4G_GPIO_Port, RST_4G_Pin, GPIO_PIN_SET);
  Y100M_DelayMs(Y100M_AT_RESET_PULSE_MS);
  HAL_GPIO_WritePin(RST_4G_GPIO_Port, RST_4G_Pin, GPIO_PIN_RESET);
  Y100M_DelayMs(Y100M_AT_BOOT_WAIT_MS);
  Y100M_Uart1DrainRx();
}

/* Run the bounded one-shot AT bootstrap sequence for the 4G module. */
static HAL_StatusTypeDef Y100M_RunBootstrapSteps(void)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t index = 0U;
  uint8_t attempt = 0U;

  for (attempt = 0U; attempt < Y100M_AT_MAX_HANDSHAKE_ATTEMPTS; attempt++)
  {
    status = Y100M_SendAtExpect("AT", "OK", Y100M_AT_CMD_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      break;
    }

  }

  if (status != HAL_OK)
  {
    Y100M_ResetPulse();
    status = Y100M_SendAtExpect("AT", "OK", Y100M_AT_CMD_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  for (index = 0U; index < (sizeof(g_y100mBootstrapSteps) / sizeof(g_y100mBootstrapSteps[0])); index++)
  {
    const Y100M_AtStep_t *step = &g_y100mBootstrapSteps[index];

    if ((step->command == NULL) || (step->command[0] == '\0'))
    {
      continue;
    }

    status = Y100M_SendAtExpect(step->command, step->expected, step->timeout_ms);
    if (status != HAL_OK)
    {
      (void)snprintf(g_y100mLogBuffer,
                     sizeof(g_y100mLogBuffer),
                     "[4G] bootstrap step failed: %s\r\n",
                     step->name);
      App_LogModem(g_y100mLogBuffer);
      return status;
    }
  }

#if (Y100M_CLOSE_OLD_SESSION_ENABLED != 0U)
  (void)Y100M_CloseOldSession();
#else
  App_LogModem("[4G] skip PDP shutdown; check existing IP context\r\n");
#endif

  status = Y100M_PdpActivate();
  if (status != HAL_OK)
  {
    App_LogModem("[4G] PDP activation failed\r\n");
    return status;
  }

  status = Y100M_MqttConnect();
  if (status != HAL_OK)
  {
    App_LogModem("[4G] MQTT connect failed\r\n");
    return status;
  }

  return HAL_OK;
}

/* Execute the one-shot bootstrap only once during startup. */
void Modem_Init(void)
{
  HAL_StatusTypeDef status;

  if (g_y100mBootstrapDone != 0U)
  {
    return;
  }

  g_y100mBootstrapDone = 1U;
  App_LogModem("[4G] start one-shot AT bootstrap\r\n");

  status = Y100M_RunBootstrapSteps();
  if (status == HAL_OK)
  {
    g_y100mBootstrapOk = 1U;
    App_LogModem("[4G] bootstrap complete, MQTT uplink ready on USART1\r\n");
  }
  else
  {
    g_y100mBootstrapOk = 0U;
    (void)snprintf(g_y100mLogBuffer,
                   sizeof(g_y100mLogBuffer),
                   "[4G] bootstrap incomplete, status=%d\r\n",
                   (int)status);
    App_LogModem(g_y100mLogBuffer);
  }
}

/* Use only when startup configuration is deliberately bypassed. */
void Modem_AssumeReady(void)
{
  g_y100mBootstrapDone = 1U;
  g_y100mBootstrapOk = 1U;
}

uint8_t Modem_IsReady(void)
{
  return g_y100mBootstrapOk;
}

HAL_StatusTypeDef Modem_Send(const uint8_t *payload, uint16_t payload_length)
{
  uint8_t mqtt_header[5];
  uint8_t topic_header[2];
  uint8_t remaining_length[4];
  uint16_t topic_length = 14U;
  uint8_t encoded_count = 0U;
  uint8_t *packet;
  uint16_t packet_length;
  uint16_t offset;
  HAL_StatusTypeDef status;
  static const uint8_t topic[] = "fsae/telemetry";

  App_LogModem("[4G-PUB] enter mqtt publish\r\n");

  if ((payload == NULL) || (payload_length == 0U) || (g_y100mBootstrapOk == 0U))
  {
    (void)snprintf(g_y100mLogBuffer,
                   sizeof(g_y100mLogBuffer),
                   "[4G-PUB] fail: invalid args or not ready payload=%p len=%u ready=%u\r\n",
                   (const void *)payload,
                   (unsigned int)payload_length,
                   (unsigned int)g_y100mBootstrapOk);
    App_LogModem(g_y100mLogBuffer);
    return HAL_ERROR;
  }

  App_LogModem("[4G-PUB] payload accepted for packing\r\n");

  mqtt_header[0] = 0x30U;
  encoded_count = Y100M_EncodeMqttRemainingLength((uint32_t)(2U + topic_length + payload_length),
                                                  remaining_length,
                                                  (uint8_t)sizeof(remaining_length));
  if (encoded_count == 0U)
  {
    App_LogModem("[4G-PUB] fail: remaining length encode failed\r\n");
    return HAL_ERROR;
  }

#if (APP_MODEM_LOG_ENABLED != 0U)
  (void)snprintf(g_y100mLogBuffer,
                 sizeof(g_y100mLogBuffer),
                 "[4G-PUB] remaining length bytes=%u\r\n",
                 (unsigned int)encoded_count);
  App_LogModem(g_y100mLogBuffer);
#endif

  (void)memcpy(&mqtt_header[1], remaining_length, encoded_count);
  topic_header[0] = (uint8_t)(topic_length >> 8);
  topic_header[1] = (uint8_t)(topic_length & 0xFFU);

  packet_length = (uint16_t)(1U + encoded_count + 2U + topic_length + payload_length);
  packet = g_y100mMqttTxBuffer;
  if (packet_length > Y100M_MQTT_TX_BUFFER_SIZE)
  {
    (void)snprintf(g_y100mLogBuffer,
                   sizeof(g_y100mLogBuffer),
                   "[4G-PUB] fail: packet too large len=%u cap=%u\r\n",
                   (unsigned int)packet_length,
                   (unsigned int)Y100M_MQTT_TX_BUFFER_SIZE);
    App_LogModem(g_y100mLogBuffer);
    return HAL_ERROR;
  }

#if (APP_MODEM_LOG_ENABLED != 0U)
  (void)snprintf(g_y100mLogBuffer,
                 sizeof(g_y100mLogBuffer),
                 "[4G-PUB] packet length=%u\r\n",
                 (unsigned int)packet_length);
  App_LogModem(g_y100mLogBuffer);
#endif

  offset = 0U;
  packet[offset++] = mqtt_header[0];
  (void)memcpy(&packet[offset], &mqtt_header[1], encoded_count);
  offset = (uint16_t)(offset + encoded_count);
  packet[offset++] = topic_header[0];
  packet[offset++] = topic_header[1];
  (void)memcpy(&packet[offset], topic, topic_length);
  offset = (uint16_t)(offset + topic_length);
  (void)memcpy(&packet[offset], payload, payload_length);

  /* The following publish packet dump was used to compare the final MQTT
   * PUBLISH bytes with the PC script. It is intentionally excluded from normal
   * builds because one dump per publish frame consumes USART3 bandwidth and can
   * perturb the 0.1 s fast-publish timing under FreeRTOS.
   */
#if (APP_MODEM_LOG_ENABLED != 0U)
  (void)snprintf(g_y100mLogBuffer,
                 sizeof(g_y100mLogBuffer),
                 "[4G] MQTT publish payload bytes=%u\r\n",
                 (unsigned int)payload_length);
  App_LogModem(g_y100mLogBuffer);
  {
    uint16_t log_offset = 0U;
    uint16_t index;

    log_offset = (uint16_t)snprintf(g_y100mLogBuffer,
                                    sizeof(g_y100mLogBuffer),
                                    "[4G] MQTT publish packet hex:");
    for (index = 0U; (index < packet_length) && (log_offset + 4U < sizeof(g_y100mLogBuffer)); index++)
    {
      log_offset += (uint16_t)snprintf(&g_y100mLogBuffer[log_offset],
                                       sizeof(g_y100mLogBuffer) - log_offset,
                                       " %02X",
                                       packet[index]);
    }
    if (log_offset + 3U < sizeof(g_y100mLogBuffer))
    {
      g_y100mLogBuffer[log_offset++] = '\r';
      g_y100mLogBuffer[log_offset++] = '\n';
      g_y100mLogBuffer[log_offset] = '\0';
    }
    App_LogModem(g_y100mLogBuffer);
  }
#endif

  App_LogModem("[4G-PUB] send raw payload begin\r\n");

  status = Y100M_SendRawPayload(packet, packet_length, Y100M_AT_CMD_TIMEOUT_MS);
  (void)snprintf(g_y100mLogBuffer,
                 sizeof(g_y100mLogBuffer),
                 "[4G-PUB] send raw payload status=%d\r\n",
                 (int)status);
  App_LogModem(g_y100mLogBuffer);

  return status;
}


#else
/* Local acquisition mode has no AT code or MQTT credential dependency. */
void Modem_Init(void) {}
void Modem_AssumeReady(void) {}
uint8_t Modem_IsReady(void) { return 0U; }
void Modem_OnRxEvent(uint16_t size) { (void)size; }
HAL_StatusTypeDef Modem_Send(const uint8_t *payload, uint16_t length)
{
  (void)payload;
  (void)length;
  return HAL_ERROR;
}
#endif
