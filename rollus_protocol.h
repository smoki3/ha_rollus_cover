#pragma once

#include "esphome.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <vector>
#include <map>
#include <cmath>
#include <functional>

// Rollus Timing Konstanten (aus SIGNALduino Analyse)
// Bit-Dauer: 800 us (1250 Baud), Halb-Bit: 400 us
#define ROLLUS_HALF_BIT_US    400
#define ROLLUS_SYNC_GAP_US    4600
#define ROLLUS_PREAMBLE_PAIRS 35
#define ROLLUS_REPEAT_COUNT   3

enum RollusJumpType : uint8_t {
  JUMP_DYNAMIC = 0, // Bit-Invertierung bei STOP/RUNTER (Standard)
  JUMP_XOR = 1      // Feste XOR-Offsets (STOP: 0xC0, RUNTER: 0xA0)
};

struct RollusMasks {
  uint8_t b5 = 0xD8;
  uint8_t mask_b6_b7 = 0xC4;
  uint8_t b3_parity = 0x1D;
  uint8_t b7_up = 0x7A;
  uint8_t b7_stop = 0x25;
  uint8_t b7_down = 0x89;
  RollusJumpType jump_type = JUMP_DYNAMIC;

  RollusMasks() = default;
  RollusMasks(uint8_t b5_, uint8_t m_b6_b7_, uint8_t b3_par_, uint8_t b7_u_, uint8_t b7_s_, uint8_t b7_d_, RollusJumpType jt_ = JUMP_DYNAMIC)
      : b5(b5_), mask_b6_b7(m_b6_b7_), b3_parity(b3_par_), b7_up(b7_u_), b7_stop(b7_s_), b7_down(b7_d_), jump_type(jt_) {}
};

struct RollusShutter {
  uint8_t id;
  uint8_t current_T;
  esphome::cover::Cover *cover;
  RollusMasks masks;
};

class RollusProtocol {
 private:
  std::map<uint8_t, RollusShutter> shutters;

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

  // Sendet einen Rollus-Befehl fuer eine bestimmte ID
  void send(uint8_t target_id, uint8_t cmdCode) {
    if (!this->transmit_func) {
      ESP_LOGE("Rollus", "Transmitter nicht gesetzt! Bitte 'rollus.set_transmitter_fn(...);' in on_boot aufrufen.");
      return;
    }
    auto pulses = this->buildPulseVector(target_id, cmdCode);
    this->transmit_func(pulses);
  }

  void send(uint8_t cmdCode) {
    if (shutters.empty()) {
      ESP_LOGE("Rollus", "Kein Rolladen registriert! Bitte ID uebergeben: rollus.send(id, cmd);");
      return;
    }
    send(shutters.begin()->first, cmdCode);
  }

  // Abwaertskompatibilitaet: Falls rollus.init(id) noch in der YAML steht
  void init(uint8_t id) {
    register_shutter(id);
  }

  // Universal-Registrierung mit individuellen Masken (Variante A)
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

  // Überladung falls Masken nicht angegeben werden
  void register_shutter(uint8_t id, esphome::cover::Cover *cover = nullptr) {
    register_shutter(id, cover, RollusMasks());
  }

  // Abwärtskompatibilität
  void register_shutter_custom(uint8_t id, esphome::cover::Cover *cover, const RollusMasks &m) {
    register_shutter(id, cover, m);
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

  // T -> c (taster-spezifische Spruenge basierend auf jump_type)
  uint8_t getCFromT(uint8_t T, uint8_t cmd, RollusJumpType jump_type) {
    if (jump_type == JUMP_XOR) {
      if (cmd == 0x41) return T ^ 0xC0;
      if (cmd == 0x21) return T ^ 0xA0;
      return T;
    }
    // JUMP_DYNAMIC (Standard)
    if (cmd == 0x81 || cmd == 0xA1) {
      return T;
    } else if (cmd == 0x41) {
      return (T & 0x40) ? T : (T ^ 0x80);
    } else if (cmd == 0x21) {
      if (T & 0x20) {
        return (T & 0x40) ? (T ^ 0xC0) : (T ^ 0x40);
      }
      return T;
    }
    return T;
  }

  // c -> T (Umkehrfunktion)
  uint8_t getTFromC(uint8_t c, uint8_t cmd, RollusJumpType jump_type) {
    if (jump_type == JUMP_XOR) {
      if (cmd == 0x41) return c ^ 0xC0;
      if (cmd == 0x21) return c ^ 0xA0;
      return c;
    }
    // JUMP_DYNAMIC (Standard)
    if (cmd == 0x81 || cmd == 0xA1) {
      return c;
    } else if (cmd == 0x41) {
      return (c & 0x40) ? c : (c ^ 0x80);
    } else if (cmd == 0x21) {
      if (c & 0x20) {
        return (c & 0x40) ? (c ^ 0x40) : (c ^ 0xC0);
      }
      return c;
    }
    return c;
  }

  void calculateFrame(uint8_t target_remote_id, uint8_t T, uint8_t cmdCode, uint8_t* rawFrame, uint8_t* decodedFrame) {
    RollusMasks m;
    if (shutters.find(target_remote_id) != shutters.end()) {
      m = shutters[target_remote_id].masks;
    } else {
      register_shutter(target_remote_id);
      m = shutters[target_remote_id].masks;
    }

    uint8_t c = getCFromT(T, cmdCode, m.jump_type);
    uint8_t b2 = cmdCode;
    uint8_t b3 = b2 ^ target_remote_id ^ m.b3_parity;
    uint8_t b5 = m.b5;

    uint8_t base_mask = m.b7_stop;
    if (cmdCode == 0x81) base_mask = m.b7_up;
    else if (cmdCode == 0x21) base_mask = m.b7_down;
    else if (cmdCode == 0xA1) base_mask = m.b7_up ^ 0x64;

    uint8_t b7 = base_mask;
    for (int i = 0; i < 8; i++) {
      if ((c >> i) & 1) b7 ^= V[i];
    }

    uint8_t b6 = b7 ^ m.mask_b6_b7;
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
          uint8_t new_T = getTFromC(c, remote_cmd, s.masks.jump_type);
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
          RollusJumpType jump_type = (b5 == 0x6E) ? JUMP_XOR : JUMP_DYNAMIC;
          uint8_t new_T = getTFromC(c, remote_cmd, jump_type);

          uint8_t mask_b6_b7 = b6 ^ b7;
          uint8_t b3_par = b2 ^ b4 ^ b3;

          // Basis aus B4 Polynom berechnen
          uint8_t b4_p = 0;
          for (int i = 0; i < 8; i++) {
            if ((b4 >> i) & 1) b4_p ^= V_B4[i];
          }

          uint8_t b7_u = 0, b7_s = 0, b7_d = 0;
          if (b5 == 0x6E) {
            b7_u = b4_p ^ 0x0E;
            b7_s = b7_u ^ 0x5F;
            b7_d = b7_u ^ 0xF3;
          } else {
            b7_s = b4_p ^ 0x15;
            b7_u = b7_s ^ 0x5F;
            b7_d = b7_s ^ 0xAC;
          }

          RollusMasks detected_masks(b5, mask_b6_b7, b3_par, b7_u, b7_s, b7_d, jump_type);
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
            ESP_LOGW("Rollus", "    rollus.register_shutter(0x%02X, id(rolladen_X), {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, %s});",
                     remote_id, b5, mask_b6_b7, b3_par, b7_u, b7_s, b7_d,
                     (jump_type == JUMP_XOR) ? "JUMP_XOR" : "JUMP_DYNAMIC");
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
