/*
  RCSwitchRmt - ESP32 library for transmitting and receiving 433/315 MHz OOK signals
  using the ESP32 RMT peripheral with multitasking (TX/RX).

  Inspired by rc-switch (https://github.com/sui77/rc-switch)
  Original rc-switch Copyright (c) 2011 Suat Özgür

  Author: Efrain Ovalle (Upartech)
  Website: https://github.com/Upartech/RCSwitchRmt
  License: MIT
  Version: 1.0.0
  Date: 2025

  -----------------------------------------------------------------------------
  Description:
  RCSwitchRmt is a modern, non-blocking implementation of RF OOK communication
  designed specifically for the ESP32 family. It uses the RMT peripheral to
  handle precise pulse timing at hardware level, allowing asynchronous
  transmission and reception of digital signals without CPU blocking.

  Unlike rc-switch, which relies on software bit-banging, RCSwitchRmt is fully
  hardware-driven and multitask-safe. Both transmitter (TX) and receiver (RX)
  can operate independently with separate protocols. Users may define custom
  protocols with 2 or 4 timing elements (HLHLHLHL) for advanced signal formats.

  Compatible with all rc-switch protocols and ideal for applications where
  timing precision and non-blocking performance are required.

  -----------------------------------------------------------------------------
  Contributors:
  - Efrain Ovalle / Upartech (https://github.com/Upartech)
  - Based conceptually on rc-switch by Suat Özgür and contributors

  -----------------------------------------------------------------------------
  License:
  MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
*/


#ifndef RCSWITCHRMT_H
#define RCSWITCHRMT_H

#pragma once

// ── Validación de plataforma/driver (falla rápido) ─────────────────────────────
#if defined(ARDUINO) && defined(ARDUINO_ARCH_ESP32)
// OK: Arduino-ESP32 (cubre todas las variantes ESP32)
#elif defined(ESP_PLATFORM)
// OK: ESP-IDF → validar que exista RMT v2
  #if !__has_include("driver/rmt_tx.h")
    #error "RCSwitchRmt requiere RMT v2 (ESP-IDF con driver/rmt_tx.h)."
  #endif
  #include "soc/soc_caps.h"
  #if !SOC_RMT_SUPPORTED
    #error "RCSwitchRmt requiere periférico RMT (SOC_RMT_SUPPORTED==1)."
  #endif
#else
  #error "RCSwitchRmt solo soporta ESP32 (Arduino-ESP32 o ESP-IDF)."
#endif

#include "esp_idf_version.h"
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0)
  #error "Se requiere ESP-IDF >= 5.0"
#endif

// ── Includes (los tuyos, colocados después de validar) ────────────────────────
#if defined(ARDUINO) && defined(ARDUINO_ARCH_ESP32)
  #include <Arduino.h>
  #include <pgmspace.h>
  #include "driver/rmt_tx.h"
  #include "driver/rmt_rx.h"
  #include "driver/rmt_common.h"
  #include "driver/rmt_encoder.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/queue.h"
#elif defined(ESP_PLATFORM)
  // En ESP-IDF no existe <Arduino.h> ni <pgmspace.h> por defecto
  #include <string.h>             // memcpy, etc.
  #include "driver/rmt_tx.h"
  #include "driver/rmt_rx.h"
  #include "driver/rmt_common.h"
  #include "driver/rmt_encoder.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/queue.h"
  // Si usas PROGMEM en IDF, defínelo vacío o migra a const en .rodata:
  #ifndef PROGMEM
    #define PROGMEM
  #endif
#endif

#endif // RCSWITCHRMT_H


// ===== Debug global =====
#ifndef RMT_OOK_DEBUG
#define RMT_OOK_DEBUG 0
#endif

struct PulseSeq { uint8_t len; uint8_t t[4]; };
struct Protocol { uint16_t T_us; PulseSeq sync, zero, one; bool inverted; };

static const Protocol PROGMEM proto[] = {
  { 300, {2,{  1, 31}}, {2,{ 1, 3}},  {2,{ 3, 1}}, false }, // 1
  { 650, {2,{  1, 10}}, {2,{ 1, 2}},  {2,{ 2, 1}}, false }, // 2
  { 100, {2,{ 30, 71}}, {2,{ 4,11}},  {2,{ 9, 6}}, false }, // 3
  { 380, {2,{  1,  6}}, {2,{ 1, 3}},  {2,{ 3, 1}}, false }, // 4
  { 500, {2,{  6, 14}}, {2,{ 1, 2}},  {2,{ 2, 1}}, false }, // 5
  { 450, {2,{ 23,  1}}, {2,{ 1, 2}},  {2,{ 2, 1}}, true  }, // 6
  { 150, {2,{  2, 62}}, {2,{ 1, 6}},  {2,{ 6, 1}}, false }, // 7
  { 200, {2,{  3,130}}, {2,{ 7,16}},  {2,{ 3,16}}, false }, // 8
  { 200, {2,{130,  7}}, {2,{16, 7}},  {2,{16, 3}}, true  }, // 9
  { 365, {2,{ 18,  1}}, {2,{ 3, 1}},  {2,{ 1, 3}}, true  }, //10
  { 270, {2,{ 36,  1}}, {2,{ 1, 2}},  {2,{ 2, 1}}, true  }, //11
  { 500, {2,{ 36,  1}}, {2,{ 1, 3}},  {2,{ 3, 1}}, false }  //12
};
static constexpr int PROTO_COUNT = sizeof(proto)/sizeof(proto[0]);

struct Decoded {
  uint32_t code;
  uint8_t  bits;
  uint16_t T_us;
  uint8_t  proto;
  uint32_t t_ms;    // timestamp (millis) cuando lo publicaste
};

// ===== RMT OOK NG =====
class RCSwitchRmt {
public:
  // Validación y estado
  enum class ValidationMode : uint8_t { LONG_RANGE, RELIABLE, SAFE_MODERATE_RANGE };

  //bool begin(int8_t txPin, int8_t rxPin, const Protocol& p, uint16_t sep_us=0, uint8_t tol_pct=60);
  // Config protocolo
  void setProtocolRx(int n);
  void setProtocolCustomRx(uint16_t T_us,const uint8_t* sync,uint8_t sl,const uint8_t* zero,uint8_t zl,const uint8_t* one,uint8_t ol,bool inv);
  void setProtocolAutoRx();

/**
 * @brief Sets the validation mode for received frames.
 * @param m Validation strategy (LONG_RANGE, RELIABLE, SAFE_MODERATE_RANGE)
 */
  void setValidation(ValidationMode m){ _vMode = m; _cand_bits = 0; }

  /**
 * @brief Adjusts RX timing tolerance.
 * @param pct Allowed percentage tolerance for bit timings.
 */
  void setReceiveTolerance(uint8_t pct){ _rx_tol=pct; }

  /**
 * @brief Sets the maximum time separation (in µs) between frames.
 * @param us Separation limit.
 */
  void setSeparationLimit(uint32_t us){ _sep_limit_us=us; }  
  void setExpectedBits(uint8_t n);
 
  // RX   
  bool enableReceive(int pin);
  bool enableReceive();
  bool disableReceive();
#if RMT_OOK_DEBUG
  bool available();                 // se ejecuta en el loop (no const porque procesa)  
#else
  bool available() const { return _nReceivedBitlength != 0; } 
#endif

// ¿Cuántos hay? (útil para UI/diagnóstico)
  size_t availableCount() const {  return _rb_count;}
  unsigned long getReceivedValue() const { return _nReceivedValue; }
  unsigned int  getReceivedBitlength() const { return _nReceivedBitlength; }
  unsigned int  getReceivedDelay() const { return _nReceivedDelay; }
  unsigned int  getReceivedProtocol() const { return _nReceivedProtocol; }
  void resetAvailable(){ _nReceivedBitlength=0; _cc=0; _repeat=0;}
  bool read(Decoded* out); // lee el buffer de datos
  void clearBuffer(); // limpia el buffer rx

  // TX
  bool enableTransmit(int pin);
  bool disableTransmit();
  bool setProtocolTx(int n);  // 1..PROTO_COUNT; valida (2 o 4) y sin ceros
  bool setProtocolCustomTx(uint16_t T_us,
                         const uint8_t* sync, uint8_t sync_len,
                         const uint8_t* zero, uint8_t zero_len,
                         const uint8_t* one,  uint8_t one_len,
                         bool inverted);
  bool lastTxHadSymbols() const { return _last_tx_syms_k > 0; }
  void setRepeatTransmit(uint16_t n);      // n >= 1; n=1 => SINGLE (una trama)
  void setMinTransmitTime(uint32_t ms);    // ms=0 => desactiva BY_TIME
  bool send(uint32_t code, uint8_t bitlen); // decide: BY_REPS (n>=2) > BY_TIME(ms>0) > SINGLE
  bool txActive() const;                   // true si canal ocupado o hay cola
  bool queue(uint32_t code, uint8_t bitlen, uint16_t reps=1);
  void clearQueue();

  
/**
 * @brief Indicates if the TX hardware channel is currently transmitting.
 * @return True if TX channel is busy.
 */
  bool txBusy() const { return _tx_busy; }

  /**
 * @brief Internal TX task loop that continues queued transmissions.
 * Called automatically; no need to invoke manually.
 */
  void repeater();
  

private:
  // RMT RX buffers (64) + flags
  static constexpr size_t RX_RAW_BUF_SYMS = 64;
  rmt_symbol_word_t _rx_buf[RX_RAW_BUF_SYMS]; 
  uint8_t _expected_clusters{25};
// Doble buffer y toggle
rmt_symbol_word_t _rx_buf0[RX_RAW_BUF_SYMS];
rmt_symbol_word_t _rx_buf1[RX_RAW_BUF_SYMS];
volatile uint8_t  _rx_sel = 0;  // 0 -> buf0; 1 -> buf1
uint32_t _max_ns_current{4800000};
// Config cacheada (evita armar rxcfg dentro de la ISR)
rmt_receive_config_t _rx_cfg = {
  .signal_range_min_ns = 3100,           // tu min (3 us)
  .signal_range_max_ns = _max_ns_current,        // por ejemplo; actualiza donde corresponda
};

rmt_receive_config_t _rx_cfg_next{};  // la próxima (precalculo fuera de ISR)
volatile bool        _rx_cfg_dirty = false;

  static constexpr size_t MAX_ELEMENTS = 32;
  Decoded       _rb[MAX_ELEMENTS];


  // Si quieres proteger con sección crítica:
portMUX_TYPE _rbMux = portMUX_INITIALIZER_UNLOCKED;
// (opcional) para proteger _rx_cfg_next en SMP u otras tareas
portMUX_TYPE _cfgMux = portMUX_INITIALIZER_UNLOCKED;

// Contadores de diagnóstico (opcionales)
volatile uint32_t _drop_badlen = 0;
struct RxPkt { const volatile rmt_symbol_word_t* p; size_t n; };

#if RMT_OOK_DEBUG
  // --- Modo debug: no cola, no tarea ---
  volatile const rmt_symbol_word_t* _dbg_p = nullptr;
  volatile size_t _dbg_n = 0;
  volatile bool _dbg_ready = false;   // “hay frame en espera”
#else
  // cola+tarea solo en producción
  void rx_task_loop_();                               // <- método de instancia
  static void rx_task_trampoline_(void* arg);         // <- estática, puente a this
  bool ensure_service_task_();
    // Cola / task:
  QueueHandle_t _rx_q   = nullptr;
  TaskHandle_t  _rx_task= nullptr;
#endif

  // Timings acumulador
  static constexpr unsigned TIMINGS_CAP = 128;
  uint32_t _timings[TIMINGS_CAP]; unsigned _cc{0};
 
  ValidationMode _vMode{ValidationMode::SAFE_MODERATE_RANGE};
  bool _has_fixed_proto{false};
  uint32_t _cand_code{0}; uint8_t _cand_bits{0};
  uint32_t _prev_first{0}; unsigned _repeat{0};
  uint32_t  _mill{0};
    uint32_t  _t_last_frame_ok{0};

  // Resultado último
  volatile uint32_t _nReceivedValue{0};
  volatile uint16_t _nReceivedDelay{0};
  volatile uint8_t  _nReceivedBitlength{0};
  volatile uint8_t  _nReceivedProtocol{0};


volatile size_t _rb_head {0};   // próxima escritura
volatile size_t _rb_tail {0};   // próxima lectura
volatile size_t _rb_count {0};

 
uint8_t  _sync_mask[4]  {0,0,0,0 };   // copia editable del SYNC (valores T-multiplo o -1)
uint8_t _sync_len{0};    // = P.sync.len   


  // Protocolo activo y params
  Protocol _cur{}; uint8_t _rx_tol{60}; uint32_t _sep_limit_us{4300};
  uint8_t _lvlH{1}, _lvlL{0};

  // RMT handles
  rmt_channel_handle_t _rx_ch{nullptr}, _tx_ch{nullptr};
  rmt_encoder_handle_t _tx_encoder{nullptr};
  volatile bool _tx_busy{false}, _tx_kick{false};
  int8_t _txPin{-1}, _rxPin{-1};
  
uint16_t _noise_min_us{50};
uint8_t  _min_bits_publish{9};     // ignora <18 bits (ajusta 12..20)
uint8_t  _expected_bitlen{24};       // por defecto; configurable si fijo/personalizado

bool match_seq(const PulseSeq& seq, const uint32_t* t, unsigned i, unsigned cc,
                             uint16_t T, uint32_t tol_abs);

void refresh_rx_cfg_();

  // Convertir símbolos a duraciones (timings) y acumular (descarta duration1==0)
void sym_to_timings_append_(const volatile rmt_symbol_word_t* s, size_t n);

void update_sync_mask();
//void collect_proto_scales_(const Protocol& P, uint8_t* S, uint8_t& Slen);
uint32_t compute_max_ns_for_proto_(const Protocol&) const;
void init_sync_mask_from_proto_(const Protocol& P);
bool maybe_publish_(uint32_t code, uint8_t bits, uint16_t T_us, uint8_t proto_idx);
bool publish_now_(uint32_t code, uint8_t bits, uint16_t T_us, uint8_t proto_idx);

uint16_t refine_T_iimf_(const uint32_t* t, unsigned i, unsigned j,
                                     uint16_t T0, uint8_t tol_pct, uint8_t iters);
bool try_decode_with_proto_(const Protocol& P,
                                         uint32_t& code, uint8_t& bits, uint16_t& T_us);

  bool create_rx_channel_on_pin_(int pin);
  bool arm_first_receive_();
  bool ensure_rx_ready_();
  // Flags de estado
  bool _rx_enabled{false};
  static bool rxDoneCB(rmt_channel_handle_t, const rmt_rx_done_event_data_t*, void*);
  void service(const volatile rmt_symbol_word_t* syms, size_t n);
  

   // TX queue y símbolos (tu código)
 
  static constexpr uint8_t  QSIZE=32, MAXBITS=32;
  static constexpr uint16_t MAXSYM=256;
  rmt_symbol_word_t _sym_buf[MAXSYM]; size_t _sym_cap=MAXSYM;
   Protocol _cur_tx{};          // protocolo activo PARA TX
  bool     _tx_proto_set{false};
  size_t   _last_tx_syms_k{0};
 

  enum class TxMode : uint8_t { BY_REPS, BY_TIME };

  struct TxItem {
    uint32_t code{0};
    uint8_t  bitlen{0};
    uint16_t reps{0};         // BY_REPS
    uint16_t remain{0};       // BY_REPS
    TxMode   mode{TxMode::BY_REPS};
    uint32_t min_ms{0};       // BY_TIME
    uint32_t t_start{0};      // BY_TIME
    bool     started{false};  // BY_TIME
    bool     used{false};
  };
   TxItem _q[QSIZE]{}; uint8_t _q_head{0}, _q_tail{0}, _q_count{0};

  // ====== Tarea TX (nueva) ======
  TaskHandle_t  _tx_task{nullptr};
  volatile bool _tx_task_ready{false}; // creada una vez
  volatile bool _tx_run{false};        // habilita bucle (enable/disable)

  // Política de envío
  uint16_t _repeat_tx{1};    // 1 => SINGLE
  uint32_t _min_time_ms{0};  // 0 => desactivado

  // Protocolo por defecto si no se llamó setProtocol/_Custom
  bool _proto_explicit{false};

  // ===== helpers de tarea/notificación =====
  static void tx_task_trampoline_(void* arg);
  void        tx_task_loop_();
  inline void notify_tx_task_();
  bool ensure_tx_task_(); // crea tarea una sola vez
  bool create_tx_channel_on_pin_(int pin);
  bool _tx_enabled{false};  
  static bool txDoneCB(rmt_channel_handle_t, const rmt_tx_done_event_data_t*, void*);
  void buildSymbolsAppend_(uint32_t code, uint8_t nbits, rmt_symbol_word_t* sym, size_t& k) const;
  bool startNextBurst_();
};
#endif

