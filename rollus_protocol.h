#pragma once

#include "esphome.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <vector>
#include <map>
#include <deque>
#include <cmath>
#include <functional>

// Rollus Timing Konstanten (aus SIGNALduino Analyse)
// Bit-Dauer: 800 us (1250 Baud), Halb-Bit: 400 us
#define ROLLUS_HALF_BIT_US    400
#define ROLLUS_SYNC_GAP_US    4600
#define ROLLUS_PREAMBLE_PAIRS 35
#define ROLLUS_REPEAT_COUNT   3

struct RollusMasks {
  uint8_t b5 = 0x6E;
  uint8_t b7_offset = 0x51;

  RollusMasks() = default;
  RollusMasks(uint8_t b5_, uint8_t b7_off_)
      : b5(b5_), b7_offset(b7_off_) {}

  // mask_b6_b7 errechnet sich aus ID (b4) und b5 mit V_B4[7] (0x31)
  uint8_t getMaskB6B7(uint8_t target_id) const {
    uint8_t b4_base = target_id & 0x7F;
    return static_cast<uint8_t>(((b4_base ^ b5 ^ 0x31) & 0x7F) | 0x80);
  }

  // b3_parity ist mathematisch immer b5 ^ mask_b6_b7 ^ 0x01
  uint8_t getB3Parity(uint8_t target_id) const {
    return static_cast<uint8_t>(b5 ^ getMaskB6B7(target_id) ^ 0x01);
  }
};

struct RollusShutter {
  uint8_t id;
  uint8_t current_T;
  esphome::cover::Cover *cover;
  RollusMasks masks;
};

struct RollusTxItem {
  uint8_t target_id;
  uint8_t cmdCode;
};

class RollusProtocol {
 private:
  std::map<uint8_t, RollusShutter> shutters;
  std::deque<RollusTxItem> tx_queue;
  uint32_t last_tx_time = 0;
  uint32_t tx_cooldown_ms = 250;

  const uint8_t V_B4[8] = {0x16, 0x2C, 0x58, 0xB0, 0x67, 0xCE, 0x9B, 0x31};
  const uint8_t V[8]    = {0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0x87, 0x09};

  std::function<void(const std::vector<int32_t>&)> transmit_func = nullptr;

  void getNvsKey(uint8_t id, char* out_key) {
    snprintf(out_key, 16, "c_%02X", id);
  }

 public:
  void set_transmitter_fn(std::function<void(const std::vector<int32_t>&)> fn) {
    this->transmit_func = fn;
  }

  void set_tx_cooldown(uint32_t ms) {
    this->tx_cooldown_ms = ms;
  }

  void notify_tx() {
    this->last_tx_time = millis();
  }

  // Sofortiges Senden ohne Warteschlange
  void send_immediate(uint8_t target_id, uint8_t cmdCode) {
    if (!this->transmit_func) {
      ESP_LOGE("Rollus", "Transmitter nicht gesetzt! Bitte 'rollus.set_transmitter_fn(...);' in on_boot aufrufen.");
      return;
    }
    auto pulses = this->buildPulseVector(target_id, cmdCode);
    this->transmit_func(pulses);
    this->last_tx_time = millis();
  }

  // Reiht einen Rollus-Befehl in die Sende-Warteschlange (Queue) ein
  void send(uint8_t target_id, uint8_t cmdCode) {
    if (this->tx_queue.size() >= 16) {
      ESP_LOGW("Rollus", "TX-Queue voll (16 Einträge)! Befehl 0x%02X für ID 0x%02X verworfen.",
               cmdCode, target_id);
      return;
    }
    this->tx_queue.push_back({target_id, cmdCode});
    ESP_LOGD("Rollus", "TX-Queue: Befehl 0x%02X fuer ID 0x%02X eingereiht (Warteschlange: %u)",
             cmdCode, target_id, (unsigned)this->tx_queue.size());
  }

  void send(uint8_t cmdCode) {
    if (shutters.empty()) {
      ESP_LOGE("Rollus", "Kein Rolladen registriert! Bitte ID uebergeben: rollus.send(id, cmd);");
      return;
    }
    send(shutters.begin()->first, cmdCode);
  }

  // Abarbeiten der Sende-Warteschlange (wird über on_loop in rollus.yaml aufgerufen)
  void loop() {
    if (this->tx_queue.empty()) return;

    uint32_t now = millis();
    if (now - this->last_tx_time < this->tx_cooldown_ms) {
      return; // Mindestabstand zwischen Funkaussendungen einhalten
    }

    auto item = this->tx_queue.front();
    this->tx_queue.pop_front();

    send_immediate(item.target_id, item.cmdCode);
  }

  // Registrierung mit individuellen Masken
  void register_shutter(uint8_t id, esphome::cover::Cover *cover, const RollusMasks &masks) {
    RollusShutter s;
    s.id = id;
    s.cover = cover;
    s.current_T = 1;
    s.masks = masks;

    // Gespeicherten Zählerstand für diesen Rolladen aus NVS laden
    char key[16];
    getNvsKey(id, key);
    nvs_handle_t handle;
    if (nvs_open("rollus", NVS_READWRITE, &handle) == ESP_OK) {
      uint8_t val = 1;
      if (nvs_get_u8(handle, key, &val) == ESP_OK) {
        s.current_T = val;
      }
      nvs_close(handle);
    }

    shutters[id] = s;
    ESP_LOGI("Rollus", "Rolladen registriert -> ID: 0x%02X, Zaehlerstand T: %u (0x%02X)",
             id, s.current_T, s.current_T);
  }

  // Überladung falls Masken nicht angegeben werden (nutzt Standard-Masken)
  void register_shutter(uint8_t id, esphome::cover::Cover *cover = nullptr) {
    register_shutter(id, cover, RollusMasks());
  }

  uint8_t get_counter(uint8_t id) {
    if (shutters.find(id) != shutters.end()) {
      return shutters[id].current_T;
    }
    return 0;
  }

  uint8_t get_counter() {
    if (!shutters.empty()) {
      return shutters.begin()->second.current_T;
    }
    return 1;
  }

  void set_counter(uint8_t id, uint8_t new_t) {
    if (shutters.find(id) == shutters.end()) {
      register_shutter(id);
    }
    shutters[id].current_T = new_t;

    char key[16];
    getNvsKey(id, key);
    nvs_handle_t handle;
    if (nvs_open("rollus", NVS_READWRITE, &handle) == ESP_OK) {
      nvs_set_u8(handle, key, new_t);
      nvs_commit(handle);
      nvs_close(handle);
    }
  }

  // T -> c (taster-spezifische Sprünge mit festen XOR-Offsets: STOP ^ 0xC0, RUNTER ^ 0xA0)
  uint8_t getCFromT(uint8_t T, uint8_t cmd) {
    if (cmd == 0x41) return T ^ 0xC0;
    if (cmd == 0x21) return T ^ 0xA0;
    return T;
  }

  // c -> T (Umkehrfunktion, bei XOR identisch)
  uint8_t getTFromC(uint8_t c, uint8_t cmd) {
    if (cmd == 0x41) return c ^ 0xC0;
    if (cmd == 0x21) return c ^ 0xA0;
    return c;
  }

  // Berechnet die b7-Tastenmasken (UP, STOP, DOWN, PROG) universell aus ID und b7_offset (feste Offsets zu STOP)
  void getB7Masks(uint8_t target_id, uint8_t b7_offset, uint8_t &b7_u, uint8_t &b7_s, uint8_t &b7_d, uint8_t &b7_p) {
    uint8_t b4_p = 0;
    for (int i = 0; i < 8; i++) {
      if ((target_id >> i) & 1) b4_p ^= V_B4[i];
    }

    b7_s = b4_p ^ b7_offset;
    b7_u = b7_s ^ 0x5F; // HOCH (0x81)
    b7_d = b7_s ^ 0xAC; // RUNTER (0x21)
    b7_p = b7_s ^ 0x3B; // PROG (0xA1)
  }

  void calculateFrame(uint8_t target_remote_id, uint8_t T, uint8_t cmdCode, uint8_t* rawFrame, uint8_t* decodedFrame) {
    RollusMasks m;
    if (shutters.find(target_remote_id) != shutters.end()) {
      m = shutters[target_remote_id].masks;
    } else {
      register_shutter(target_remote_id);
      m = shutters[target_remote_id].masks;
    }

    uint8_t c = getCFromT(T, cmdCode);
    uint8_t b2 = cmdCode;
    uint8_t b3 = b2 ^ target_remote_id ^ m.getB3Parity(target_remote_id);
    uint8_t b5 = m.b5;

    uint8_t b7_u = 0, b7_s = 0, b7_d = 0, b7_p = 0;
    getB7Masks(target_remote_id, m.b7_offset, b7_u, b7_s, b7_d, b7_p);

    uint8_t base_mask = b7_s;
    if (cmdCode == 0x81) base_mask = b7_u;
    else if (cmdCode == 0x21) base_mask = b7_d;
    else if (cmdCode == 0xA1) base_mask = b7_p;

    uint8_t b7 = base_mask;
    for (int i = 0; i < 8; i++) {
      if ((c >> i) & 1) b7 ^= V[i];
    }

    uint8_t b6 = b7 ^ m.getMaskB6B7(target_remote_id);
    uint8_t b0 = b6 ^ c;
    uint8_t b1 = 0x01;
    uint8_t b4 = target_remote_id;

    decodedFrame[0] = b0;
    decodedFrame[1] = b1;
    decodedFrame[2] = b2;
    decodedFrame[3] = b3;
    decodedFrame[4] = b4;
    decodedFrame[5] = b5;
    decodedFrame[6] = b6;
    decodedFrame[7] = b7;

    rawFrame[0] = decodedFrame[0];
    for (int i = 1; i < 8; i++) {
      rawFrame[i] = decodedFrame[i] ^ rawFrame[i - 1];
    }
  }

  std::vector<int32_t> buildPulseVector(uint8_t cmdCode) {
    if (shutters.empty()) return {};
    return buildPulseVector(shutters.begin()->first, cmdCode);
  }

  // Erzeugt den Roh-Pulsvektor für remote_transmitter für eine spezifische ID
  std::vector<int32_t> buildPulseVector(uint8_t target_id, uint8_t cmdCode) {
    if (shutters.find(target_id) == shutters.end()) {
      register_shutter(target_id);
    }

    // Zaehler herunterzaehlen (echte Fernbedienung zaehlt 255 -> 0 rueckwaerts)
    uint8_t new_T = (shutters[target_id].current_T - 1) & 0xFF;
    set_counter(target_id, new_T);

    uint8_t rawFrame[8];
    uint8_t decodedFrame[8];
    calculateFrame(target_id, new_T, cmdCode, rawFrame, decodedFrame);

    const char* cmd_str = (cmdCode == 0x81) ? "HOCH" : ((cmdCode == 0x21) ? "RUNTER" : "STOP");
    ESP_LOGI("Rollus", "Sende %s (0x%02X) fuer Rolladen ID=0x%02X mit neuem Zaehler T=%u (0x%02X)",
             cmd_str, cmdCode, target_id, new_T, new_T);
    ESP_LOGD("Rollus", "SendeFrame: %02X %02X %02X %02X %02X %02X %02X %02X",
             rawFrame[0], rawFrame[1], rawFrame[2], rawFrame[3],
             rawFrame[4], rawFrame[5], rawFrame[6], rawFrame[7]);

    std::vector<int32_t> pulses;

    for (int rep = 0; rep < ROLLUS_REPEAT_COUNT; rep++) {
      // 1. Präambel: 35 Paare HIGH / LOW je 400 us
      for (int i = 0; i < ROLLUS_PREAMBLE_PAIRS; i++) {
        pulses.push_back(ROLLUS_HALF_BIT_US);
        pulses.push_back(-ROLLUS_HALF_BIT_US);
      }

      // 2. Sync-Lücke: 4600 us LOW
      pulses.push_back(-ROLLUS_SYNC_GAP_US);

      // 3. Start-Marker: 400 us HIGH, 400 us LOW
      pulses.push_back(ROLLUS_HALF_BIT_US);
      pulses.push_back(-ROLLUS_HALF_BIT_US);

      // 4. 64 Manchester-Datenbits (MSB zuerst)
      for (int byteIdx = 0; byteIdx < 8; byteIdx++) {
        uint8_t b = rawFrame[byteIdx];
        for (int bitIdx = 7; bitIdx >= 0; bitIdx--) {
          uint8_t bit = (b >> bitIdx) & 1;
          if (bit == 0) {
            pulses.push_back(ROLLUS_HALF_BIT_US);
            pulses.push_back(-ROLLUS_HALF_BIT_US);
          } else {
            pulses.push_back(-ROLLUS_HALF_BIT_US);
            pulses.push_back(ROLLUS_HALF_BIT_US);
          }
        }
      }
    }
    return pulses;
  }

  std::vector<int32_t> rx_stream;

  // Verarbeitet eingehende Roh-Pulse von remote_receiver
  void handleRxPulses(const std::vector<int32_t>& pulses) {
    if (pulses.empty()) return;

    // Pulse an den fortlaufenden Stream anhaengen
    rx_stream.insert(rx_stream.end(), pulses.begin(), pulses.end());

    // Wenn der Stream zu gross wird (ueber 800 Pulse), alte Daten verwerfen
    if (rx_stream.size() > 800) {
      rx_stream.erase(rx_stream.begin(), rx_stream.begin() + 400);
    }

    // Finde alle Sync-Gaps im Puffer (-3000 bis -7000 us)
    for (size_t g = 0; g < rx_stream.size(); g++) {
      if (rx_stream[g] < -3000 && rx_stream[g] > -7000) {
        int gap_idx = g;
        size_t last_p_idx = gap_idx;
        std::vector<uint8_t> half_bits;
        for (size_t i = gap_idx + 1; i < rx_stream.size(); i++) {
          int32_t p = rx_stream[i];
          uint8_t level = (p > 0) ? 1 : 0;
          int32_t abs_p = std::abs(p);
          int count = 0;
          if (abs_p >= 150 && abs_p <= 590) {
            count = 1;
          } else if (abs_p > 590 && abs_p <= 1200) {
            count = 2;
          } else {
            // Puls passt nicht (z.B. naechster Sync-Gap oder Pause)
            break;
          }
          for (int c = 0; c < count; c++) {
            half_bits.push_back(level);
          }
          last_p_idx = i;
          if (half_bits.size() >= 130) {
            break; // Frame komplett!
          }
        }

        if (half_bits.size() < 129) {
          if (last_p_idx + 1 < rx_stream.size()) {
            rx_stream.erase(rx_stream.begin(), rx_stream.begin() + gap_idx + 1);
            g = -1;
          }
          continue; // Ansonsten warten, falls das Signal am Stream-Ende noch unvollstaendig ist
        }

        // Wenn genau 129 Halb-Bits empfangen wurden und der nächste Puls ein LOW/Gap ist:
        if (half_bits.size() == 129) {
          half_bits.push_back(0); // Letztes Halb-Bit ist LOW (Sync-Lücke / Ruhepegel)
        }

        if (half_bits[0] != 1 || half_bits[1] != 0) {
          continue;
        }

        uint8_t raw_bytes[8] = {0};
        bool manchester_error = false;
        for (int bitIdx = 0; bitIdx < 64; bitIdx++) {
          uint8_t s1 = half_bits[2 + bitIdx * 2];
          uint8_t s2 = half_bits[2 + bitIdx * 2 + 1];
          uint8_t bit_val = 0;
          if (s1 == 1 && s2 == 0) {
            bit_val = 0;
          } else if (s1 == 0 && s2 == 1) {
            bit_val = 1;
          } else {
            manchester_error = true;
            break;
          }
          int bIdx = bitIdx / 8;
          int bitPos = 7 - (bitIdx % 8);
          raw_bytes[bIdx] |= (bit_val << bitPos);
        }
        if (manchester_error) continue;

    uint8_t dec_bytes[8];
    dec_bytes[0] = raw_bytes[0];
    for (int k = 1; k < 8; k++) {
      dec_bytes[k] = raw_bytes[k] ^ raw_bytes[k - 1];
    }

    // Wenn der Frame bereits unverschleiert übertragen wurde (raw_bytes[1] == 0x01):
    uint8_t* f = (raw_bytes[1] == 0x01) ? raw_bytes : dec_bytes;

    uint8_t b0 = f[0];
    uint8_t b1 = f[1];
    uint8_t b2 = f[2];
    uint8_t b3 = f[3];
    uint8_t b4 = f[4];
    uint8_t b5 = f[5];
    uint8_t b6 = f[6];
    uint8_t b7 = f[7];

    // DEBUG: Dekodierte Bytes jedes empfangenen 64-Bit-Pakets (zum Deaktivieren einfach auskommentieren)
    // ESP_LOGI("Rollus", "Dekodierte Bytes: %02X %02X %02X %02X %02X %02X %02X %02X", b0, b1, b2, b3, b4, b5, b6, b7);

    // Rollus-Telegramm (Start-Byte b1 ist 0x01)
    if (b1 == 0x01) {
      if (b2 == 0x81 || b2 == 0x41 || b2 == 0x21 || b2 == 0xA1) {
        uint8_t c = b0 ^ b6;
        uint8_t remote_cmd = b2;
        uint8_t remote_id = b4;
        const char* cmd_str = (remote_cmd == 0x81) ? "HOCH" : ((remote_cmd == 0x21) ? "RUNTER" : ((remote_cmd == 0xA1) ? "PROG" : "STOP"));

        // Pruefen, ob die Fernbedienung bereits registriert und einem Cover zugewiesen ist
        if (shutters.find(remote_id) != shutters.end() && shutters[remote_id].cover != nullptr) {
          auto &s = shutters[remote_id];
          uint8_t new_T = getTFromC(c, remote_cmd);
          set_counter(remote_id, new_T);

          // ESP_LOGI("Rollus", ">>> Rolladen 0x%02X: Taste %s (0x%02X) empfangen | T=%u",
          //          remote_id, cmd_str, remote_cmd, new_T);
          auto *cov = s.cover;
          if (remote_cmd == 0x81) {
            cov->current_operation = esphome::cover::COVER_OPERATION_OPENING;
            cov->position = esphome::cover::COVER_OPEN;
            cov->publish_state();
          } else if (remote_cmd == 0x21) {
            cov->current_operation = esphome::cover::COVER_OPERATION_CLOSING;
            cov->position = esphome::cover::COVER_CLOSED;
            cov->publish_state();
          } else if (remote_cmd == 0x41) {
            cov->current_operation = esphome::cover::COVER_OPERATION_IDLE;
            cov->publish_state();
          } else {
            ESP_LOGI("Rollus", ">>> Rolladen 0x%02X: Programmieren (0xA1) empfangen | T=%u", remote_id, new_T);
          }
        } else {
          // Unbekannte Fernbedienung: Masken direkt aus den Empfangsdaten ableiten
          uint8_t new_T = getTFromC(c, remote_cmd);

          // b7_offset universell rekonstruieren
          uint8_t c_v = 0;
          for (int i = 0; i < 8; i++) {
            if ((c >> i) & 1) c_v ^= V[i];
          }
          uint8_t base_mask = b7 ^ c_v;
          uint8_t base_stop = base_mask;
          if (remote_cmd == 0x81) base_stop = base_mask ^ 0x5F;
          else if (remote_cmd == 0x21) base_stop = base_mask ^ 0xAC;
          else if (remote_cmd == 0xA1) base_stop = base_mask ^ 0x3B;

          uint8_t b4_p = 0;
          for (int i = 0; i < 8; i++) {
            if ((remote_id >> i) & 1) b4_p ^= V_B4[i];
          }
          uint8_t b7_offset = base_stop ^ b4_p;

          RollusMasks detected_masks(b5, b7_offset);
          if (shutters.find(remote_id) == shutters.end()) {
            register_shutter(remote_id, nullptr, detected_masks);
          }
          set_counter(remote_id, new_T);

          static uint32_t last_unknown_ms = 0;
          static uint8_t last_unknown_id = 0;
          static uint8_t last_unknown_t = 0;
          uint32_t now = millis();
          if (remote_id != last_unknown_id || new_T != last_unknown_t || (now - last_unknown_ms > 1500)) {
            last_unknown_ms = now;
            last_unknown_id = remote_id;
            last_unknown_t = new_T;

            ESP_LOGW("Rollus", "Unbekannte FB -> ID: 0x%02X | Taste: %s | T: %u",
                     remote_id, cmd_str, new_T);
            ESP_LOGW("Rollus", ">>> In rollus.yaml unter on_boot einfuegen:");
            ESP_LOGW("Rollus", "    rollus.register_shutter(0x%02X, id(rolladen_X), {0x%02X, 0x%02X});",
                     remote_id, b5, b7_offset);
          }
        }
      } else {
        // Sonder-Signal (z.B. Programmier-/Pairing-Taste P2)
        static uint32_t last_special_ms = 0;
        static uint8_t last_special_cmd = 0;
        uint32_t now = millis();
        if (b2 != last_special_cmd || (now - last_special_ms > 1000)) {
          last_special_ms = now;
          last_special_cmd = b2;
          ESP_LOGW("Rollus", "Sonder-Signal / Taste 0x%02X -> ID: 0x%02X | Bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                   b2, b4, b0, b1, b2, b3, b4, b5, b6, b7);
        }
      }

      // Verwerfe verarbeitete Pulse bis hinter dieses Frame
      rx_stream.erase(rx_stream.begin(), rx_stream.begin() + std::min(last_p_idx + 1, rx_stream.size()));
      // g auf -1 setzen, damit nach dem Schleifeninkrement (g++) wieder bei Index 0 begonnen wird
      g = -1;
      continue;
    }
      }
    }
  }
};

inline RollusProtocol rollus;
