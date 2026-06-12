// ============================================================
// esp8266_espnow_ota_receiver.ino
// ESP-NOW OTA Target Device
//
// Boot sequence:
//   1. Send ESP-NOW beacon every 250ms for BEACON_DURATION_MS (3s)
//   2. If OTA offer received: accept, receive chunks, flash, reboot
//   3. If no offer within timeout: call startApplication()
//
// IMPORTANT – compile with an OTA-capable flash layout, e.g.:
//   Tools -> Flash Size -> "4M (1M SPIFFS)" or "4M (2M SPIFFS)"
//   This ensures two firmware slots exist for Update to work.
//
// Board:    ESP8266 (NodeMCU / ESP-12E/F)
// Requires: espnow.h, Updater.h (both built-in to ESP8266 Arduino core)
// ============================================================

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Updater.h>
extern "C" {
  #include <user_interface.h>
}

// ======================================================
// Protocol constants  (must match sender)
// ======================================================
#define ESPNOW_CHANNEL    1
#define CHUNK_DATA_SIZE   200   // must match sender definition

// Bump this number each time you release a new firmware build.
// The receiver will reject any OTA offer that carries the same version.
#define FIRMWARE_VERSION  4u

// OTA offer flags (must match sender)
#define OTA_FLAG_FORCE   0x01  // sender set --force; skip version check

// ESP-NOW message types
#define MSG_BEACON        0x01
#define MSG_OTA_OFFER     0x02
#define MSG_OTA_ACCEPT    0x03
#define MSG_OTA_REJECT    0x04
#define MSG_OTA_DATA      0x05
#define MSG_OTA_ACK       0x06
#define MSG_OTA_NAK       0x07
#define MSG_OTA_END       0x08
#define MSG_OTA_DONE      0x09
#define MSG_OTA_ERROR     0x0A

// OTA timing
#define BEACON_DURATION_MS  3000  // how long to broadcast beacons on boot
#define BEACON_INTERVAL_MS  250   // beacon repeat interval
#define OTA_TOTAL_TIMEOUT   300000UL  // 5 min max for full OTA transfer

// ======================================================
// Packet structures (must match sender)
// ======================================================
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

// Magic tag embedded in the binary so the PC tool can scan for it and extract
// the version without needing a --version argument.
// getVersion() reads from the tag at runtime — a genuine code reference that
// forces the linker to keep the array even with --gc-sections.
// Placed AFTER all struct typedefs so the Arduino preprocessor's auto-generated
// function prototypes don't appear before the type declarations.
volatile const uint8_t OTA_VERSION_TAG[] = {
  'O', 'T', 'A', 'V',
  (FIRMWARE_VERSION >>  0) & 0xFF,
  (FIRMWARE_VERSION >>  8) & 0xFF,
  (FIRMWARE_VERSION >> 16) & 0xFF,
  (FIRMWARE_VERSION >> 24) & 0xFF
};

static uint32_t getVersion() {
  return (uint32_t)OTA_VERSION_TAG[4]
       | ((uint32_t)OTA_VERSION_TAG[5] << 8)
       | ((uint32_t)OTA_VERSION_TAG[6] << 16)
       | ((uint32_t)OTA_VERSION_TAG[7] << 24);
}

// ======================================================
// OTA state
// ======================================================
enum OtaPhase {
  OTA_BEACONING,   // Sending beacons, waiting for offer
  OTA_RECEIVING,   // Receiving and flashing chunks
  OTA_DONE,        // Flash written successfully – reboot pending
  OTA_SKIP         // No update; continue with application
};

static OtaPhase  g_phase         = OTA_BEACONING;
static uint8_t   g_sender_mac[6] = {};
static uint32_t  g_fw_size       = 0;
static uint32_t  g_fw_crc        = 0;
static uint32_t  g_bytes_written = 0;
static uint8_t   g_expected_seq  = 0;

// ESP-NOW receive staging buffer (ISR -> main)
static volatile bool g_msg_pending = false;
static uint8_t       g_msg_mac[6];
static uint8_t       g_msg_buf[250];
static uint8_t       g_msg_len;

// Scratch buffer for Update.write() (must be non-const)
static uint8_t g_write_buf[CHUNK_DATA_SIZE];

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
// Send helpers
// ======================================================
static bool espnowAddPeer(uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  return esp_now_add_peer(mac, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, nullptr, 0) == 0;
}

static void sendMsg(uint8_t *mac, espnow_msg_t *m, uint16_t payload_len) {
  esp_now_send(mac, (uint8_t *)m, 4 + payload_len);
}

static void sendSimple(uint8_t *mac, uint8_t type, uint8_t seq = 0) {
  espnow_msg_t m;
  m.type = type;
  m.seq  = seq;
  m.len  = 0;
  sendMsg(mac, &m, 0);
}

// reason: 0x00 = generic (Update.begin failed), 0x01 = same version
static void sendReject(uint8_t *mac, uint8_t reason) {
  espnow_msg_t m;
  m.type       = MSG_OTA_REJECT;
  m.seq        = 0;
  m.len        = 1;
  m.payload[0] = reason;
  sendMsg(mac, &m, 1);
}

static void sendBeacon() {
  espnow_msg_t m;
  m.type = MSG_BEACON;
  m.seq  = 0;

  beacon_info_t *b   = (beacon_info_t *)m.payload;
  b->firmware_version = getVersion();
  strncpy(b->device_name, "ESP-NODE", sizeof(b->device_name));
  m.len = sizeof(beacon_info_t);

  uint8_t bc[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  sendMsg(bc, &m, sizeof(beacon_info_t));
}

static void sendDone(uint8_t status) {
  espnow_msg_t m;
  m.type       = MSG_OTA_DONE;
  m.seq        = 0;
  m.len        = 1;
  m.payload[0] = status;
  sendMsg(g_sender_mac, &m, 1);
}

// ======================================================
// Process one incoming ESP-NOW message
// ======================================================
static void processMessage() {
  if (!g_msg_pending) return;

  const espnow_msg_t *m = (const espnow_msg_t *)g_msg_buf;

  switch (g_phase) {

    // ---------------------------------------------------
    case OTA_BEACONING: {
      if (m->type == MSG_OTA_OFFER) {
        const ota_offer_t *off = (const ota_offer_t *)m->payload;
        g_fw_size = off->firmware_size;
        g_fw_crc  = off->crc32;
        memcpy(g_sender_mac, g_msg_mac, 6);
        espnowAddPeer(g_sender_mac);

        Serial.printf("[OTA] Offer: v%u, %u bytes, CRC32=0x%08X\n",
                      off->offered_version, g_fw_size, g_fw_crc);

        bool force = (off->flags & OTA_FLAG_FORCE) != 0;
        if (!force && off->offered_version == getVersion()) {
          Serial.printf("[OTA] Already running v%u, rejecting\n", getVersion());
          sendReject(g_sender_mac, 0x01);  // 0x01 = same version
          g_phase = OTA_SKIP;
        } else if (Update.begin(g_fw_size, U_FLASH)) {
          g_bytes_written = 0;
          g_expected_seq  = 0;
          g_phase         = OTA_RECEIVING;
          sendSimple(g_sender_mac, MSG_OTA_ACCEPT);
          Serial.printf("[OTA] Accepted v%u → v%u%s, receiving...\n",
                        getVersion(), off->offered_version, force ? " (forced)" : "");
        } else {
          Serial.printf("[OTA] Update.begin failed (error %d), rejecting\n", Update.getError());
          sendReject(g_sender_mac, 0x00);  // 0x00 = generic
          g_phase = OTA_SKIP;
        }
      }
      break;
    }

    // ---------------------------------------------------
    case OTA_RECEIVING: {
      if (m->type == MSG_OTA_DATA) {
        if (m->seq == g_expected_seq) {
          const ota_data_t *dp = (const ota_data_t *)m->payload;

          // Copy to non-const buffer (Update.write expects uint8_t*)
          memcpy(g_write_buf, dp->data, dp->data_len);
          size_t written = Update.write(g_write_buf, dp->data_len);

          if (written != dp->data_len) {
            Serial.printf("[OTA] Write error at offset %u (err %d)\n",
                          dp->offset, Update.getError());
            sendSimple(g_sender_mac, MSG_OTA_NAK, m->seq);
          } else {
            g_bytes_written += written;
            g_expected_seq   = (g_expected_seq + 1) & 0xFF;
            sendSimple(g_sender_mac, MSG_OTA_ACK, m->seq);

            if (g_bytes_written % 10240 == 0 || g_bytes_written >= g_fw_size) {
              Serial.printf("[OTA] Written %u / %u bytes\n", g_bytes_written, g_fw_size);
            }
          }
        } else if (m->seq == (uint8_t)(g_expected_seq - 1)) {
          // Duplicate: sender missed our ACK, resend it
          sendSimple(g_sender_mac, MSG_OTA_ACK, m->seq);
        } else {
          // Unexpected sequence, NAK
          sendSimple(g_sender_mac, MSG_OTA_NAK, m->seq);
        }

      } else if (m->type == MSG_OTA_END) {
        Serial.println("[OTA] All chunks received, finalising flash...");
        if (Update.end()) {
          if (Update.isFinished()) {
            Serial.println("[OTA] Flash OK, sending DONE");
            sendDone(0);
            g_phase = OTA_DONE;
          } else {
            Serial.printf("[OTA] Incomplete flash (err %d)\n", Update.getError());
            sendDone((uint8_t)Update.getError());
            g_phase = OTA_SKIP;
          }
        } else {
          Serial.printf("[OTA] Update.end() failed (err %d)\n", Update.getError());
          sendDone((uint8_t)Update.getError());
          g_phase = OTA_SKIP;
        }
      }
      break;
    }

    default:
      break;
  }

  g_msg_pending = false;
}

// ======================================================
// YOUR APPLICATION CODE
// Replace this with the actual application logic.
// Called after OTA check completes without update.
// ======================================================
static void startApplication() {
  Serial.println("[APP] Starting application...");
  // --- Put your application initialisation here ---
  pinMode(2, OUTPUT);
}

// ======================================================
// Arduino setup – OTA check runs entirely here
// ======================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n--- ESP-NOW OTA Receiver ---");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  wifi_set_channel(ESPNOW_CHANNEL);

  if (esp_now_init() != 0) {
    Serial.println("[OTA] ESP-NOW init failed, skipping OTA");
    g_phase = OTA_SKIP;
  } else {
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_register_recv_cb(onEspNowReceive);
    esp_now_register_send_cb(onEspNowSent);

    // Broadcast peer for beacon transmissions
    uint8_t bc[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_add_peer(bc, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, nullptr, 0);

    // ---- Beacon phase ----
    Serial.printf("[OTA] Broadcasting beacons for %d ms...\n", BEACON_DURATION_MS);
    uint32_t start      = millis();
    uint32_t lastBeacon = 0;

    while (millis() - start < BEACON_DURATION_MS) {
      if (millis() - lastBeacon >= BEACON_INTERVAL_MS) {
        sendBeacon();
        lastBeacon = millis();
      }
      processMessage();
      if (g_phase != OTA_BEACONING) break;  // offer accepted or rejected
      yield();
    }

    if (g_phase == OTA_BEACONING) {
      Serial.println("[OTA] No offer received, skipping OTA");
      g_phase = OTA_SKIP;
    }
  }

  // ---- Receive phase (if offer accepted) ----
  if (g_phase == OTA_RECEIVING) {
    uint32_t ota_start = millis();
    while (g_phase == OTA_RECEIVING) {
      processMessage();
      if (millis() - ota_start > OTA_TOTAL_TIMEOUT) {
        Serial.println("[OTA] Transfer timeout, aborting");
        Update.end(false);
        g_phase = OTA_SKIP;
        break;
      }
      yield();
    }
  }

  // ---- Result ----
  if (g_phase == OTA_DONE) {
    Serial.println("[OTA] Update complete, rebooting in 500 ms...");
    delay(500);
    ESP.restart();
    // Does not return
  }

  // OTA_SKIP: clean up and start application
  esp_now_deinit();
  startApplication();
}

// ======================================================
// Application main loop (only reached when no OTA)
// ======================================================
void loop() {
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);
}
