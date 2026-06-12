// ============================================================
// esp8266_espnow_ota_sender.ino
// ESP-NOW OTA Transmitter — real-time streaming, no SPIFFS
//
// Flow:
//   1. PC sends CMD_INIT  (fw_size + crc32) → start listening
//   2. Target boots, beacons → offer sent, target accepts
//   3. Sender sends RSP_DEVICE_FOUND + RSP_READY_DATA to PC
//   4. PC sends CMD_DATA chunk → sender forwards via ESP-NOW
//   5. Target ACKs → sender sends RSP_READY_DATA → repeat
//   6. After last chunk ACK → sender sends MSG_OTA_END to target
//   7. Target flashes, reboots → sender sends RSP_COMPLETE to PC
//
// Board:    ESP8266 (NodeMCU / ESP-12E/F)
// Flash:    any layout; SPIFFS not used
// Requires: espnow.h (built into ESP8266 Arduino core)
// ============================================================

#include <ESP8266WiFi.h>
#include <espnow.h>
extern "C" {
  #include <user_interface.h>  // wifi_set_channel()
}

// ======================================================
// Protocol constants  (must match receiver)
// ======================================================
#define ESPNOW_CHANNEL   1
#define CHUNK_DATA_SIZE  200   // payload bytes per ESP-NOW data packet

// ESP-NOW message types
#define MSG_BEACON       0x01  // Target  -> broadcast
#define MSG_OTA_OFFER    0x02  // Sender  -> target
#define MSG_OTA_ACCEPT   0x03  // Target  -> sender
#define MSG_OTA_REJECT   0x04  // Target  -> sender
#define MSG_OTA_DATA     0x05  // Sender  -> target
#define MSG_OTA_ACK      0x06  // Target  -> sender
#define MSG_OTA_NAK      0x07  // Target  -> sender
#define MSG_OTA_END      0x08  // Sender  -> target (all chunks delivered)
#define MSG_OTA_DONE     0x09  // Target  -> sender (flash result)
#define MSG_OTA_ERROR    0x0A  // Either direction

// Serial framing
#define SERIAL_START     0xAA  // PC -> Sender
#define SERIAL_RESP      0xBB  // Sender -> PC

// OTA offer flags
#define OTA_FLAG_FORCE   0x01  // skip version check on target

// Commands (PC -> Sender)
#define CMD_INIT         0x01  // Start session: [fw_size:4LE][crc32:4LE][version:4LE][flags:1]
#define CMD_DATA         0x02  // Next chunk: [raw bytes, CHUNK_DATA_SIZE max]
#define CMD_STATUS       0x03  // Query (no payload)
#define CMD_CANCEL       0x04  // Abort (no payload)

// Responses (Sender -> PC)
#define RSP_OK            0x01  // CMD_INIT accepted
#define RSP_ERROR         0x02  // [error_code:1]
#define RSP_DEVICE_FOUND  0x03  // Target detected and accepted: [mac:6]
#define RSP_READY_DATA    0x04  // Send next CMD_DATA chunk now
#define RSP_COMPLETE      0x05  // OTA finished successfully
#define RSP_VERSION_MATCH 0x06  // Target already runs this version: [mac:6]

// Error codes
#define ERR_ESPNOW       0x01
#define ERR_TIMEOUT      0x02
#define ERR_REJECTED     0x03
#define ERR_CRC          0x04  // (reserved; CRC verified by target's Update lib)
#define ERR_BUSY         0x05
#define ERR_TARGET_FLASH 0x06

// Timing
#define MAX_RETRIES       5
#define ACK_TIMEOUT_MS    2000
#define OFFER_TIMEOUT_MS  5000
#define DONE_TIMEOUT_MS   20000  // Update.end() can be slow

// ======================================================
// Packet structures  (packed, shared with receiver)
// ======================================================
typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  seq;
  uint16_t len;
  uint8_t  payload[244];  // 4+244 = 248 ≤ 250 (ESP-NOW hard limit)
} espnow_msg_t;

typedef struct __attribute__((packed)) {
  uint32_t firmware_size;
  uint32_t crc32;
  uint16_t chunk_size;
  uint32_t offered_version;  // version of the firmware being offered
  uint8_t  flags;            // OTA_FLAG_FORCE (0x01) = skip version check
} ota_offer_t;

typedef struct __attribute__((packed)) {
  uint32_t offset;
  uint16_t data_len;
  uint8_t  data[CHUNK_DATA_SIZE];
} ota_data_t;

typedef struct __attribute__((packed)) {
  uint32_t firmware_version;
  char     device_name[12];
} beacon_info_t;

// ======================================================
// State machine
// ======================================================
enum SenderState {
  S_IDLE,           // waiting for CMD_INIT
  S_WAIT_DEVICE,    // listening for target beacon
  S_OFFERING,       // offer sent, waiting accept/reject
  S_WAIT_PC_CHUNK,  // ready, waiting for CMD_DATA
  S_WAIT_ACK,       // chunk forwarded via ESP-NOW, waiting ACK
  S_WAIT_DONE,      // MSG_OTA_END sent, waiting MSG_OTA_DONE
};

// ======================================================
// Globals
// ======================================================
static SenderState g_state          = S_IDLE;
static uint8_t     g_peer_mac[6]    = {};
static bool        g_device_found   = false;
static bool        g_offer_accepted = false;
static bool        g_offer_rejected = false;
static uint8_t     g_reject_reason  = 0;   // reason byte from MSG_OTA_REJECT
static bool        g_ack_received   = false;
static bool        g_nak_received   = false;
static bool        g_ota_done       = false;
static uint8_t     g_done_status    = 0;
static uint8_t     g_seq            = 0;
static uint32_t    g_firmware_size    = 0;
static uint32_t    g_firmware_crc     = 0;
static uint32_t    g_firmware_version = 0;  // version of firmware being offered
static uint8_t     g_firmware_flags   = 0;  // OTA_FLAG_FORCE etc.
static uint32_t    g_bytes_acked    = 0;  // confirmed by target
static uint32_t    g_timer          = 0;
static uint8_t     g_retries        = 0;

// Single-chunk buffer — all we ever keep in RAM
static uint8_t  g_chunk_buf[CHUNK_DATA_SIZE];
static uint16_t g_chunk_len    = 0;
static uint32_t g_chunk_offset = 0;

// ESP-NOW receive staging  (ISR -> main loop)
static volatile bool g_msg_pending = false;
static uint8_t       g_msg_mac[6];
static uint8_t       g_msg_buf[250];
static uint8_t       g_msg_len;

// Serial frame parser
enum SerialParse { SP_IDLE, SP_CMD, SP_LEN_H, SP_LEN_L, SP_DATA };
static SerialParse g_sp           = SP_IDLE;
static uint8_t     g_cmd          = 0;
static uint16_t    g_expect_len   = 0;
static uint16_t    g_serial_pos   = 0;
// Buffer must hold one full CMD_DATA payload (≤ CHUNK_DATA_SIZE)
static uint8_t     g_serial_buf[CHUNK_DATA_SIZE + 8];

// ======================================================
// Serial helpers
// ======================================================
static void serialRsp(uint8_t rsp, const uint8_t *data = nullptr, uint16_t len = 0) {
  Serial.write(SERIAL_RESP);
  Serial.write(rsp);
  Serial.write((uint8_t)(len >> 8));
  Serial.write((uint8_t)(len & 0xFF));
  if (data && len) Serial.write(data, len);
}

static void serialError(uint8_t code) {
  serialRsp(RSP_ERROR, &code, 1);
}

// ======================================================
// ESP-NOW helpers
// ======================================================
static bool espnowAddPeer(uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  return esp_now_add_peer(mac, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, nullptr, 0) == 0;
}

static void espnowSend(uint8_t *mac, espnow_msg_t *m, uint16_t payload_len) {
  esp_now_send(mac, (uint8_t *)m, 4 + payload_len);
}

static void sendSimple(uint8_t *mac, uint8_t type, uint8_t seq = 0) {
  espnow_msg_t m;
  m.type = type;
  m.seq  = seq;
  m.len  = 0;
  espnowSend(mac, &m, 0);
}

// ======================================================
// ESP-NOW callbacks
// ======================================================
static void ICACHE_RAM_ATTR onEspNowReceive(uint8_t *mac, uint8_t *data, uint8_t len) {
  if (g_msg_pending) return;
  memcpy(g_msg_mac, mac, 6);
  memcpy(g_msg_buf, data, len);
  g_msg_len = len;
  g_msg_pending = true;
}

static void onEspNowSent(uint8_t *mac, uint8_t status) {
  (void)mac; (void)status;
}

// ======================================================
// Process one received ESP-NOW message  (main loop)
// ======================================================
static void processEspNowMsg() {
  if (!g_msg_pending) return;
  const espnow_msg_t *m = (const espnow_msg_t *)g_msg_buf;

  switch (m->type) {
    case MSG_BEACON:
      if (g_state == S_WAIT_DEVICE && !g_device_found) {
        memcpy(g_peer_mac, g_msg_mac, 6);
        g_device_found = true;
      }
      break;
    case MSG_OTA_ACCEPT:  g_offer_accepted = true; break;
    case MSG_OTA_REJECT:
      g_reject_reason  = (m->len >= 1) ? m->payload[0] : 0;
      g_offer_rejected = true;
      break;
    case MSG_OTA_ACK:
      if (m->seq == g_seq) g_ack_received = true;
      break;
    case MSG_OTA_NAK:
      g_nak_received = true;
      break;
    case MSG_OTA_DONE:
      g_done_status = (m->len >= 1) ? m->payload[0] : 0;
      g_ota_done = true;
      break;
    default: break;
  }
  g_msg_pending = false;
}

// ======================================================
// Send (or re-send) the currently buffered chunk
// ======================================================
static void sendCurrentChunk() {
  espnow_msg_t m;
  m.type = MSG_OTA_DATA;
  m.seq  = g_seq;

  ota_data_t *dp = (ota_data_t *)m.payload;
  dp->offset   = g_chunk_offset;
  dp->data_len = g_chunk_len;
  memcpy(dp->data, g_chunk_buf, g_chunk_len);

  // payload = offset(4) + data_len(2) + data(g_chunk_len)
  uint16_t pay = (uint16_t)(sizeof(ota_data_t) - CHUNK_DATA_SIZE + g_chunk_len);
  m.len = pay;
  espnowSend(g_peer_mac, &m, pay);

  g_ack_received = false;
  g_nak_received = false;
  g_timer = millis();
}

// ======================================================
// Serial command processor
// ======================================================
static void processSerialCmd(uint8_t cmd, const uint8_t *data, uint16_t len) {
  switch (cmd) {

    case CMD_INIT: {
      if (g_state != S_IDLE) { serialError(ERR_BUSY); return; }
      if (len < 12)          { serialError(ERR_BUSY); return; }
      memcpy(&g_firmware_size,    data,      4);
      memcpy(&g_firmware_crc,     data + 4,  4);
      memcpy(&g_firmware_version, data + 8,  4);
      g_firmware_flags = (len >= 13) ? data[12] : 0;
      g_bytes_acked  = 0;
      g_seq          = 0;
      g_device_found = false;
      g_state        = S_WAIT_DEVICE;
      serialRsp(RSP_OK);
      break;
    }

    case CMD_DATA: {
      if (g_state != S_WAIT_PC_CHUNK)        { serialError(ERR_BUSY); return; }
      if (len == 0 || len > CHUNK_DATA_SIZE) { serialError(ERR_BUSY); return; }
      g_chunk_offset = g_bytes_acked;
      g_chunk_len    = len;
      memcpy(g_chunk_buf, data, len);
      g_retries = 0;
      g_state   = S_WAIT_ACK;
      sendCurrentChunk();
      break;
    }

    case CMD_STATUS: {
      uint8_t s[2] = {
        (uint8_t)g_state,
        (g_firmware_size > 0)
          ? (uint8_t)((g_bytes_acked * 100UL) / g_firmware_size)
          : 0
      };
      serialRsp(RSP_OK, s, 2);
      break;
    }

    case CMD_CANCEL: {
      if (g_state >= S_WAIT_PC_CHUNK)
        sendSimple(g_peer_mac, MSG_OTA_ERROR);
      g_state = S_IDLE;
      serialRsp(RSP_OK);
      break;
    }

    default:
      serialError(ERR_BUSY);
      break;
  }
}

// ======================================================
// Serial framing parser  (one byte at a time)
// ======================================================
static void handleSerialByte(uint8_t b) {
  switch (g_sp) {
    case SP_IDLE:
      if (b == SERIAL_START) g_sp = SP_CMD;
      break;
    case SP_CMD:
      g_cmd = b;
      g_sp  = SP_LEN_H;
      break;
    case SP_LEN_H:
      g_expect_len = (uint16_t)b << 8;
      g_sp = SP_LEN_L;
      break;
    case SP_LEN_L:
      g_expect_len |= b;
      g_serial_pos  = 0;
      if (g_expect_len == 0) {
        processSerialCmd(g_cmd, nullptr, 0);
        g_sp = SP_IDLE;
      } else {
        g_sp = SP_DATA;
      }
      break;
    case SP_DATA:
      if (g_serial_pos < sizeof(g_serial_buf))
        g_serial_buf[g_serial_pos++] = b;
      if (g_serial_pos >= g_expect_len) {
        processSerialCmd(g_cmd, g_serial_buf, g_serial_pos);
        g_sp = SP_IDLE;
      }
      break;
  }
}

// ======================================================
// OTA state machine  (called from loop)
// ======================================================
static void handleStateMachine() {
  switch (g_state) {

    case S_WAIT_DEVICE:
      if (g_device_found) {
        espnowAddPeer(g_peer_mac);
        espnow_msg_t m;
        m.type = MSG_OTA_OFFER;
        m.seq  = 0;
        ota_offer_t *off = (ota_offer_t *)m.payload;
        off->firmware_size   = g_firmware_size;
        off->crc32           = g_firmware_crc;
        off->chunk_size      = CHUNK_DATA_SIZE;
        off->offered_version = g_firmware_version;
        off->flags           = g_firmware_flags;
        m.len = sizeof(ota_offer_t);
        espnowSend(g_peer_mac, &m, sizeof(ota_offer_t));
        g_offer_accepted = false;
        g_offer_rejected = false;
        g_timer = millis();
        g_state = S_OFFERING;
      }
      break;

    case S_OFFERING:
      if (g_offer_accepted) {
        serialRsp(RSP_DEVICE_FOUND, g_peer_mac, 6);
        serialRsp(RSP_READY_DATA);
        g_state = S_WAIT_PC_CHUNK;
      } else if (g_offer_rejected) {
        if (g_reject_reason == 0x01) {
          // Target already runs this version — inform PC, keep listening
          serialRsp(RSP_VERSION_MATCH, g_peer_mac, 6);
        }
        g_device_found = false;
        g_state = S_WAIT_DEVICE;
      } else if (millis() - g_timer > OFFER_TIMEOUT_MS) {
        g_device_found = false;
        g_state = S_WAIT_DEVICE;
      }
      break;

    case S_WAIT_ACK:
      if (g_ack_received) {
        g_ack_received = false;
        g_retries      = 0;
        g_bytes_acked += g_chunk_len;

        if (g_bytes_acked >= g_firmware_size) {
          sendSimple(g_peer_mac, MSG_OTA_END, g_seq);
          g_timer = millis();
          g_state = S_WAIT_DONE;
        } else {
          g_seq = (g_seq + 1) & 0xFF;
          serialRsp(RSP_READY_DATA);
          g_state = S_WAIT_PC_CHUNK;
        }
      } else if (g_nak_received || (millis() - g_timer > ACK_TIMEOUT_MS)) {
        g_nak_received = false;
        if (++g_retries > MAX_RETRIES) {
          g_state = S_IDLE;
          serialError(ERR_TIMEOUT);
        } else {
          sendCurrentChunk();  // resend same chunk, g_bytes_acked unchanged
        }
      }
      break;

    case S_WAIT_DONE:
      if (g_ota_done) {
        g_ota_done = false;
        g_state    = S_IDLE;
        if (g_done_status == 0) {
          serialRsp(RSP_COMPLETE);
        } else {
          serialError(g_done_status ? g_done_status : ERR_TARGET_FLASH);
        }
      } else if (millis() - g_timer > DONE_TIMEOUT_MS) {
        g_state = S_IDLE;
        serialError(ERR_TIMEOUT);
      }
      break;

    default:
      break;
  }
}

// ======================================================
// Arduino entry points
// ======================================================
void setup() {
  Serial.begin(921600);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  wifi_set_channel(ESPNOW_CHANNEL);

  if (esp_now_init() != 0) {
    uint8_t e = ERR_ESPNOW;
    serialRsp(RSP_ERROR, &e, 1);
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onEspNowReceive);
  esp_now_register_send_cb(onEspNowSent);

  // Broadcast peer — needed to hear incoming beacons from unknown MACs
  uint8_t bc[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_now_add_peer(bc, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, nullptr, 0);

  serialRsp(RSP_OK);
}

void loop() {
  while (Serial.available())
    handleSerialByte((uint8_t)Serial.read());

  processEspNowMsg();
  handleStateMachine();
}
