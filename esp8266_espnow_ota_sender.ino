// ============================================================
// esp8266_espnow_ota_sender.ino
// ESP-NOW OTA Transmitter — per-device independent streaming
//
// Each accepting device gets its own unicast chunk stream.
// Devices advance at their own pace; a slow/retry-heavy device
// does not hold back any other device.
//
// Protocol (PC ↔ Sender, binary framing AA cmd lenH lenL [data]):
//
//   CMD_INIT      0x01  [fw_size:4LE][crc32:4LE][version:4LE][flags:1][device_type:12][discovery_ms:4LE]
//   CMD_CHUNK_FOR 0x05  [device_idx:1][chunk_bytes:<=200]
//   CMD_STATUS    0x03  (no payload)
//   CMD_CANCEL    0x04  (no payload)
//
//   RSP_OK           0x01  CMD_INIT accepted
//   RSP_ERROR        0x02  [error_code:1]
//   RSP_DEVICE_FOUND 0x03  [device_idx:1][mac:6]  device accepted offer
//   RSP_VERSION_MATCH 0x06 [mac:6]  device already current
//   RSP_TYPE_MISMATCH 0x09 [mac:6]  device rejected — wrong firmware type
//   RSP_NEED_CHUNK   0x07  [device_idx:1][offset:4LE]  send this chunk next
//   RSP_DEVICE_DONE  0x08  [device_idx:1][status:1]  0=ok, else flash error
//   RSP_COMPLETE     0x05  all devices finished (success or not)
//
// Flow:
//   1. PC sends CMD_INIT, sender opens a discovery_ms beacon window (PC-supplied)
//   2. Sender broadcasts offer; devices accept/reject
//   3. RSP_DEVICE_FOUND [idx][mac] sent per accepting device
//   4. Sender begins sending RSP_NEED_CHUNK for each device independently
//   5. PC responds with CMD_CHUNK_FOR for each request (reactive, event-driven)
//   6. Sender unicasts chunk to device, waits for MSG_OTA_ACK
//   7. On ACK: advance device, send next RSP_NEED_CHUNK; repeat until done
//   8. RSP_DEVICE_DONE sent when each device finishes flashing
//   9. RSP_COMPLETE when all devices have reported done/error
//
// Board:    ESP8266 (NodeMCU / ESP-12E/F)
// Requires: espnow.h (built into ESP8266 Arduino core)
// ============================================================

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <math.h>
extern "C" {
  #include <user_interface.h>
}

// ── Protocol constants ────────────────────────────────────────────────────────
#define ESPNOW_CHANNEL   1
#define CHUNK_DATA_SIZE  200

#define MSG_BEACON       0x01
#define MSG_OTA_OFFER    0x02
#define MSG_OTA_ACCEPT   0x03
#define MSG_OTA_REJECT   0x04
#define MSG_OTA_DATA     0x05
#define MSG_OTA_ACK      0x06
#define MSG_OTA_NAK      0x07
#define MSG_OTA_END      0x08
#define MSG_OTA_DONE     0x09
#define MSG_OTA_ERROR    0x0A

#define SERIAL_START     0xAA
#define SERIAL_RESP      0xBB

#define OTA_FLAG_FORCE   0x01

#define CMD_INIT         0x01
#define CMD_CHUNK_FOR    0x05  // [device_idx:1][data:N]
#define CMD_STATUS       0x03
#define CMD_CANCEL       0x04

#define RSP_OK            0x01
#define RSP_ERROR         0x02
#define RSP_DEVICE_FOUND  0x03  // [device_idx:1][mac:6]
#define RSP_COMPLETE      0x05
#define RSP_VERSION_MATCH 0x06
#define RSP_NEED_CHUNK    0x07  // [device_idx:1][offset:4LE]
#define RSP_DEVICE_DONE   0x08  // [device_idx:1][status:1]
#define RSP_TYPE_MISMATCH 0x09  // [mac:6]  device rejected wrong firmware type

#define ERR_ESPNOW       0x01
#define ERR_TIMEOUT      0x02
#define ERR_REJECTED     0x03
#define ERR_CRC          0x04
#define ERR_BUSY         0x05
#define ERR_TARGET_FLASH 0x06

#define MAX_PEERS            5
#define DISCOVERY_WINDOW_MS  2500  // default; PC overrides via CMD_INIT discovery_ms field
#define DISCOVERY_SETTLE_MS  800   // time to wait for more peers after the 1st is found —
                                    // must stay well under the target's own OTA_WINDOW_MS
                                    // reply window, or it'll have given up listening by
                                    // the time the offer goes out
#define OFFER_COLLECT_MS     1500
#define MAX_RETRIES          5
#define ACK_TIMEOUT_MS       2000
#define PC_CHUNK_TIMEOUT_MS  10000  // timeout waiting for CMD_CHUNK_FOR
#define DONE_TIMEOUT_MS      20000

#define PIN_LED           2
#define BREATHE_PERIOD_MS 2000
#define CHUNK_FLASH_MS    40

// ── Packet structures ─────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  seq;
  uint16_t len;
  uint8_t  payload[244];
} espnow_msg_t;

typedef struct __attribute__((packed)) {
  uint32_t firmware_size;
  uint32_t crc32;
  uint16_t chunk_size;
  uint32_t offered_version;
  uint8_t  flags;
  char     device_type[12];   // null-padded target type e.g. "TX-NODE"
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

// ── Per-device state ──────────────────────────────────────────────────────────
enum PeerPhase : uint8_t {
  PEER_WAIT_OFFER,   // in offering phase, waiting for accept/reject
  PEER_NEED_CHUNK,   // chunk ack'd (or fresh start), ready for next request
  PEER_WAIT_PC,      // RSP_NEED_CHUNK sent, waiting CMD_CHUNK_FOR from PC
  PEER_WAIT_ACK,     // chunk unicast to device, waiting MSG_OTA_ACK
  PEER_SEND_END,     // last chunk ack'd, need to send MSG_OTA_END
  PEER_WAIT_DONE,    // MSG_OTA_END sent, waiting MSG_OTA_DONE
  PEER_DONE,         // completed
  PEER_ERROR,        // failed
};

struct OtaPeer {
  uint8_t   mac[6];
  bool      active;       // accepted the offer
  bool      responded;    // accepted or rejected (for offer timeout logic)
  PeerPhase phase;

  // Independent chunk stream for this device
  uint32_t  bytes_acked;
  uint8_t   seq;
  uint8_t   chunk_buf[CHUNK_DATA_SIZE];
  uint16_t  chunk_len;
  uint32_t  chunk_offset;
  uint8_t   retries;
  uint32_t  timer;
  uint8_t   done_status;
};

// ── Global state ──────────────────────────────────────────────────────────────
enum SenderState {
  S_IDLE,
  S_DISCOVERY,
  S_OFFERING,
  S_STREAMING,   // per-device state machines run here
};

static SenderState g_state        = S_IDLE;
static OtaPeer     g_peers[MAX_PEERS] = {};
static uint8_t     g_peer_count   = 0;
static uint8_t     g_active_count = 0;
static bool        g_pc_busy      = false;  // RSP_NEED_CHUNK sent, awaiting CMD_CHUNK_FOR
static uint32_t    g_timer        = 0;
static uint32_t    g_first_peer_ms = 0;  // millis() when the 1st peer of this session was found

static uint32_t    g_firmware_size    = 0;
static uint32_t    g_firmware_crc     = 0;
static uint32_t    g_firmware_version = 0;
static uint8_t     g_firmware_flags   = 0;
static char        g_firmware_type[12] = {};  // null-padded, from CMD_INIT
static uint32_t    g_discovery_window_ms = DISCOVERY_WINDOW_MS;  // from CMD_INIT

static uint32_t    g_flash_until  = 0;  // LED flash timestamp

// ESP-NOW receive staging
static volatile bool g_msg_pending = false;
static uint8_t       g_msg_mac[6];
static uint8_t       g_msg_buf[250];
static uint8_t       g_msg_len;

// Serial parser
enum SerialParse { SP_IDLE, SP_CMD, SP_LEN_H, SP_LEN_L, SP_DATA };
static SerialParse g_sp         = SP_IDLE;
static uint8_t     g_cmd        = 0;
static uint16_t    g_expect_len = 0;
static uint16_t    g_serial_pos = 0;
// Must hold CMD_CHUNK_FOR: 1 (idx) + CHUNK_DATA_SIZE
static uint8_t     g_serial_buf[CHUNK_DATA_SIZE + 8];

static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ── Serial helpers ────────────────────────────────────────────────────────────
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

// ── ESP-NOW helpers ───────────────────────────────────────────────────────────
static bool espnowAddPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist((uint8_t *)mac)) return true;
  return esp_now_add_peer((uint8_t *)mac, ESP_NOW_ROLE_COMBO,
                          ESPNOW_CHANNEL, nullptr, 0) == 0;
}

static void espnowSend(const uint8_t *mac, espnow_msg_t *m, uint16_t payload_len) {
  esp_now_send((uint8_t *)mac, (uint8_t *)m, 4 + payload_len);
}

static void sendSimpleTo(const uint8_t *mac, uint8_t type, uint8_t seq = 0) {
  espnow_msg_t m;
  m.type = type;
  m.seq  = seq;
  m.len  = 0;
  espnowSend(mac, &m, 0);
}

// ── Peer list helpers ─────────────────────────────────────────────────────────
static OtaPeer *findOtaPeer(const uint8_t *mac) {
  for (uint8_t i = 0; i < g_peer_count; i++)
    if (memcmp(g_peers[i].mac, mac, 6) == 0) return &g_peers[i];
  return nullptr;
}

static OtaPeer *addOtaPeer(const uint8_t *mac) {
  if (g_peer_count >= MAX_PEERS) return nullptr;
  if (g_peer_count == 0) g_first_peer_ms = millis();
  OtaPeer *p = &g_peers[g_peer_count++];
  memset(p, 0, sizeof(OtaPeer));
  memcpy(p->mac, mac, 6);
  p->phase = PEER_WAIT_OFFER;
  espnowAddPeer(mac);
  g_flash_until = millis() + 600;  // diagnostic: long flash = peer actually registered
  return p;
}

// ── Send current chunk to a specific device (unicast) ────────────────────────
static void sendChunkTo(OtaPeer *p) {
  espnow_msg_t m;
  m.type = MSG_OTA_DATA;
  m.seq  = p->seq;

  ota_data_t *dp = (ota_data_t *)m.payload;
  dp->offset   = p->chunk_offset;
  dp->data_len = p->chunk_len;
  memcpy(dp->data, p->chunk_buf, p->chunk_len);

  uint16_t pay = (uint16_t)(sizeof(ota_data_t) - CHUNK_DATA_SIZE + p->chunk_len);
  m.len = pay;
  espnowSend(p->mac, &m, pay);  // unicast — only this device gets it

  p->timer       = millis();
  g_flash_until  = p->timer + CHUNK_FLASH_MS;
}

// ── Request next chunk for a peer from the PC ─────────────────────────────────
static void requestChunkFor(uint8_t idx) {
  OtaPeer *p = &g_peers[idx];
  uint8_t  payload[5];
  payload[0] = idx;
  memcpy(payload + 1, &p->bytes_acked, 4);
  serialRsp(RSP_NEED_CHUNK, payload, 5);
  p->phase  = PEER_WAIT_PC;
  p->timer  = millis();
  g_pc_busy = true;
}

// ── ESP-NOW callbacks ─────────────────────────────────────────────────────────
static void ICACHE_RAM_ATTR onEspNowReceive(uint8_t *mac, uint8_t *data, uint8_t len) {
  // Diagnostic: flash the LED on ANY received ESP-NOW packet, independent of
  // protocol state — lets you visually confirm the radio is hearing something
  // without needing the serial link (which is reserved for the PC protocol).
  g_flash_until = millis() + 120;

  if (g_msg_pending) return;
  memcpy(g_msg_mac, mac, 6);
  memcpy(g_msg_buf, data, len);
  g_msg_len     = len;
  g_msg_pending = true;
}

static void onEspNowSent(uint8_t *mac, uint8_t status) {
  (void)mac; (void)status;
}

// ── Process one ESP-NOW message ───────────────────────────────────────────────
static void processEspNowMsg() {
  if (!g_msg_pending) return;
  const espnow_msg_t *m = (const espnow_msg_t *)g_msg_buf;

  switch (m->type) {

    case MSG_BEACON:
      if (g_state == S_DISCOVERY) {
        if (!findOtaPeer(g_msg_mac))
          addOtaPeer(g_msg_mac);
      }
      break;

    case MSG_OTA_ACCEPT:
      if (g_state == S_OFFERING) {
        OtaPeer *p = findOtaPeer(g_msg_mac);
        if (p && !p->responded) {
          p->responded = true;
          p->active    = true;
          p->phase     = PEER_NEED_CHUNK;
          g_active_count++;
        }
      }
      break;

    case MSG_OTA_REJECT:
      if (g_state == S_OFFERING) {
        OtaPeer *p = findOtaPeer(g_msg_mac);
        if (p && !p->responded) {
          p->responded = true;
          p->phase     = PEER_ERROR;
          if (m->len >= 1) {
            if (m->payload[0] == 0x01)
              serialRsp(RSP_VERSION_MATCH,  g_msg_mac, 6);
            else if (m->payload[0] == 0x02)
              serialRsp(RSP_TYPE_MISMATCH, g_msg_mac, 6);
          }
        }
      }
      break;

    case MSG_OTA_ACK: {
      OtaPeer *p = findOtaPeer(g_msg_mac);
      if (p && p->active && p->phase == PEER_WAIT_ACK && m->seq == p->seq) {
        p->retries     = 0;
        p->bytes_acked += p->chunk_len;
        if (p->bytes_acked >= g_firmware_size) {
          p->phase = PEER_SEND_END;   // handled in state machine
        } else {
          p->seq   = (p->seq + 1) & 0xFF;
          p->phase = PEER_NEED_CHUNK;
        }
      }
      break;
    }

    case MSG_OTA_NAK: {
      OtaPeer *p = findOtaPeer(g_msg_mac);
      if (p && p->active && p->phase == PEER_WAIT_ACK) {
        if (++p->retries > MAX_RETRIES) {
          p->phase = PEER_ERROR;
          uint8_t pl[2] = { (uint8_t)(p - g_peers), ERR_TARGET_FLASH };
          serialRsp(RSP_DEVICE_DONE, pl, 2);
        } else {
          sendChunkTo(p);
        }
      }
      break;
    }

    case MSG_OTA_DONE: {
      OtaPeer *p = findOtaPeer(g_msg_mac);
      if (p && p->active && p->phase == PEER_WAIT_DONE) {
        p->done_status = (m->len >= 1) ? m->payload[0] : 0;
        p->phase       = PEER_DONE;
        uint8_t pl[2]  = { (uint8_t)(p - g_peers), p->done_status };
        serialRsp(RSP_DEVICE_DONE, pl, 2);
      }
      break;
    }

    default:
      break;
  }

  g_msg_pending = false;
}

// ── Serial command processor ──────────────────────────────────────────────────
static void processSerialCmd(uint8_t cmd, const uint8_t *data, uint16_t len) {
  switch (cmd) {

    case CMD_INIT: {
      // Expected: [fw_size:4LE][crc32:4LE][version:4LE][flags:1][device_type:12][discovery_ms:4LE] = 29 bytes
      if (g_state != S_IDLE) { serialError(ERR_BUSY); return; }
      if (len < 29)          { serialError(ERR_BUSY); return; }
      memcpy(&g_firmware_size,    data,      4);
      memcpy(&g_firmware_crc,     data +  4, 4);
      memcpy(&g_firmware_version, data +  8, 4);
      g_firmware_flags = data[12];
      memcpy(g_firmware_type,     data + 13, 12);
      g_firmware_type[11] = '\0';  // safety: ensure null-terminated for print
      memcpy(&g_discovery_window_ms, data + 25, 4);

      memset(g_peers, 0, sizeof(g_peers));
      g_peer_count   = 0;
      g_active_count = 0;
      g_pc_busy      = false;
      g_timer        = millis();
      g_state        = S_DISCOVERY;
      serialRsp(RSP_OK);
      break;
    }

    case CMD_CHUNK_FOR: {
      // PC delivers chunk data for a specific device
      if (g_state != S_STREAMING || !g_pc_busy) { serialError(ERR_BUSY); return; }
      if (len < 2 || len > CHUNK_DATA_SIZE + 1)  { serialError(ERR_BUSY); return; }

      uint8_t idx = data[0];
      if (idx >= g_peer_count) { serialError(ERR_BUSY); return; }
      OtaPeer *p = &g_peers[idx];
      if (p->phase != PEER_WAIT_PC) { serialError(ERR_BUSY); return; }

      uint16_t chunk_len = len - 1;
      p->chunk_offset = p->bytes_acked;
      p->chunk_len    = chunk_len;
      memcpy(p->chunk_buf, data + 1, chunk_len);
      p->retries  = 0;
      p->phase    = PEER_WAIT_ACK;
      g_pc_busy   = false;
      sendChunkTo(p);
      break;
    }

    case CMD_STATUS: {
      uint8_t s[2] = { (uint8_t)g_state, 0 };
      serialRsp(RSP_OK, s, 2);
      break;
    }

    case CMD_CANCEL: {
      if (g_state >= S_STREAMING)
        sendSimpleTo(BROADCAST, MSG_OTA_ERROR);
      g_state   = S_IDLE;
      g_pc_busy = false;
      serialRsp(RSP_OK);
      break;
    }

    default:
      serialError(ERR_BUSY);
      break;
  }
}

// ── Serial framing parser ─────────────────────────────────────────────────────
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

// ── OTA state machine ─────────────────────────────────────────────────────────
static void handleStateMachine() {
  switch (g_state) {

    // ── Discovery: collect beacons until window expires, or — once the first
    //    peer is heard — until a short settle period passes (the target only
    //    listens for a reply for its own brief OTA_WINDOW_MS, so we can't sit
    //    on a found peer for the rest of a multi-minute discovery window) ────
    case S_DISCOVERY: {
      bool windowExpired = (millis() - g_timer >= g_discovery_window_ms);
      bool settled = (g_peer_count > 0) &&
                     (millis() - g_first_peer_ms >= DISCOVERY_SETTLE_MS);
      if (windowExpired || settled) {
        if (g_peer_count == 0) {
          serialError(ERR_TIMEOUT);
          g_state = S_IDLE;
          break;
        }
        espnow_msg_t m;
        m.type = MSG_OTA_OFFER;
        m.seq  = 0;
        ota_offer_t *off = (ota_offer_t *)m.payload;
        off->firmware_size   = g_firmware_size;
        off->crc32           = g_firmware_crc;
        off->chunk_size      = CHUNK_DATA_SIZE;
        off->offered_version = g_firmware_version;
        off->flags           = g_firmware_flags;
        memcpy(off->device_type, g_firmware_type, 12);
        m.len = sizeof(ota_offer_t);
        espnowSend(BROADCAST, &m, sizeof(ota_offer_t));
        g_timer = millis();
        g_state = S_OFFERING;
      }
      break;
    }

    // ── Offering: collect accepts/rejects ─────────────────────────────────
    case S_OFFERING: {
      uint8_t responded = 0;
      for (uint8_t i = 0; i < g_peer_count; i++)
        if (g_peers[i].responded) responded++;

      bool all_responded = (responded >= g_peer_count);
      bool timed_out     = (millis() - g_timer > OFFER_COLLECT_MS);

      if (g_active_count > 0 && (all_responded || timed_out)) {
        // Report each accepting device to PC with its index
        for (uint8_t i = 0; i < g_peer_count; i++) {
          if (!g_peers[i].active) continue;
          uint8_t pl[7];
          pl[0] = i;
          memcpy(pl + 1, g_peers[i].mac, 6);
          serialRsp(RSP_DEVICE_FOUND, pl, 7);
        }
        g_state = S_STREAMING;
        // Don't send RSP_READY_DATA — first RSP_NEED_CHUNK signals streaming start
      } else if (g_active_count == 0 && timed_out) {
        serialError(ERR_REJECTED);
        g_state = S_IDLE;
      }
      break;
    }

    // ── Streaming: run per-device state machines ──────────────────────────
    case S_STREAMING: {
      uint8_t finished = 0;

      for (uint8_t i = 0; i < g_peer_count; i++) {
        OtaPeer *p = &g_peers[i];
        if (!p->active) continue;

        switch (p->phase) {

          case PEER_NEED_CHUNK:
            // Request next chunk from PC — but only one request outstanding at a time
            if (!g_pc_busy)
              requestChunkFor(i);
            break;

          case PEER_WAIT_PC:
            // Timeout if PC doesn't respond (shouldn't happen on USB serial)
            if (millis() - p->timer > PC_CHUNK_TIMEOUT_MS) {
              g_pc_busy = false;
              requestChunkFor(i);  // re-request
            }
            break;

          case PEER_WAIT_ACK:
            if (millis() - p->timer > ACK_TIMEOUT_MS) {
              if (++p->retries > MAX_RETRIES) {
                p->phase = PEER_ERROR;
                uint8_t pl[2] = { i, ERR_TIMEOUT };
                serialRsp(RSP_DEVICE_DONE, pl, 2);
              } else {
                sendChunkTo(p);  // unicast retry to this device only
              }
            }
            break;

          case PEER_SEND_END:
            sendSimpleTo(p->mac, MSG_OTA_END, p->seq);
            p->phase = PEER_WAIT_DONE;
            p->timer = millis();
            break;

          case PEER_WAIT_DONE:
            if (millis() - p->timer > DONE_TIMEOUT_MS) {
              p->phase = PEER_ERROR;
              uint8_t pl[2] = { i, ERR_TIMEOUT };
              serialRsp(RSP_DEVICE_DONE, pl, 2);
            }
            break;

          case PEER_DONE:
          case PEER_ERROR:
            finished++;
            break;

          default:
            break;
        }
      }

      // All active devices have finished (done or error)
      if (finished >= g_active_count) {
        g_state = S_IDLE;
        serialRsp(RSP_COMPLETE);
      }
      break;
    }

    default:
      break;
  }
}

// ── LED driver ────────────────────────────────────────────────────────────────
static void updateLed() {
  uint32_t now = millis();
  bool streaming = (g_state == S_STREAMING);

  if (streaming || now < g_flash_until) {
    analogWrite(PIN_LED, (now < g_flash_until) ? 0 : 1023);
  } else {
    static const float kFactor = 2.0f * (float)M_PI / (float)BREATHE_PERIOD_MS;
    float    phase = (float)(now % (uint32_t)BREATHE_PERIOD_MS) * kFactor;
    uint16_t val   = (uint16_t)((1.0f - cosf(phase)) * 511.5f);
    analogWrite(PIN_LED, 1023 - val);
  }
}

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(921600);

  pinMode(PIN_LED, OUTPUT);
  analogWrite(PIN_LED, 1023);

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

  esp_now_add_peer((uint8_t *)BROADCAST, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, nullptr, 0);

  serialRsp(RSP_OK);
}

void loop() {
  while (Serial.available())
    handleSerialByte((uint8_t)Serial.read());

  processEspNowMsg();
  handleStateMachine();
  updateLed();
}
