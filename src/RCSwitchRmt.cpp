#include "RCSwitchRmt.h"

// util cola circular
static inline uint8_t qnext(uint8_t x, uint8_t mod){ return (uint8_t)((x+1u)%mod); }
static inline uint32_t adiffu(uint32_t a, uint32_t b){ return (a>b)?(a-b):(b-a); }

static uint8_t number_simbols_expected(const Protocol& P, uint8_t Nbits){
  if (P.zero.len!=P.one.len){
  #if RMT_OOK_DEBUG
  Serial.println("[ERR] La longitud de zero and one debe ser igual!");
  #endif
  return 25;
  }

  return P.sync.len/2+(P.zero.len/2)*Nbits;
}
static void is_valid_seq_iqual(uint8_t l_zero, uint8_t l_one){
  if (l_zero!=l_one){
     Serial.printf("[ERROR] longitud de zero y one diferente! l_zero:%u, l_one:%u",l_zero,l_one);       
  } 
}

static  bool is_valid_seq_(const PulseSeq& s){
    if (s.len != 2 && s.len != 4) {
     Serial.println("[ERROR] Error: requiere longitud para sync, zero y one en 2 o 4!");          
    return false;     // solo 2 o 4
  }
  for(uint8_t i=0;i<s.len;i++){
    if (s.t[i] == 0) return false;                // sin ceros
  }
  return true;
}


void RCSwitchRmt::update_sync_mask() {
  if (!_has_fixed_proto) return;
  // busca el mayor elemento del sync
  uint8_t max_sync = 0;
  uint8_t max_i = 0;
  for (uint8_t i = 0; i < _cur.sync.len; i++){
        _sync_mask[i] = _cur.sync.t[i]; // inicializar _sync_mask
        if (_cur.sync.t[i] > max_sync) {
          max_sync = _cur.sync.t[i];           
           max_i=i;
          }
       
  } 
  _sync_mask[max_i] = 255; // marca el numero mayor
  // aplica tolerancia del 60%
 // _max_ns_current = (uint32_t)(max_sync * _cur.T_us * 1000UL * 0.6f);
  //    rmt_receive_config_t rxcfg;
  //  rxcfg.signal_range_min_ns = 3000; // ~3us (anti-ruido duro)
  //  rxcfg.signal_range_max_ns = _max_ns_current;
  //  rmt_receive(_rx_ch, _rx_buf, sizeof(_rx_buf), &rxcfg);
 
   #if RMT_OOK_DEBUG
  Serial.printf("[INI] Sync+=%u, max_i=%u, _max_ns_current:%lu", max_sync,max_i, _max_ns_current);
  #endif
}

/**
 * @brief Selects a predefined protocol from the internal table for RX decoding.
 * @param n Protocol index (1..PROTO_COUNT)
 */ 
void RCSwitchRmt::setProtocolRx(int n){
  if(n<1) n=1;
  // copia segura desde PROGMEM
  Protocol tmp{};
  memcpy_P(&tmp, &proto[n-1], sizeof(Protocol));
  _cur = tmp;
  _nReceivedProtocol = (uint8_t)n;

  // por defecto: nSeparationLimit = primer elemento del SYNC * T
  uint8_t first = (_cur.sync.len>0) ? _cur.sync.t[0] : 1;
  //_sep_limit_us = (uint32_t)first * _cur.T_us;

   _has_fixed_proto = true;
   _expected_clusters = 25;  
  refresh_rx_cfg_();
   update_sync_mask(); 
   enableReceive(); 
}



/**
 * @brief Defines a custom RX protocol using user-defined timing sequences.
 * 
 * Each symbol (SYNC, ZERO, ONE) must contain 2 or 4 alternating time elements (HLHL...).
 * @param T_us Base pulse width in microseconds.
 * @param sync Pointer to sync timing array.
 * @param sl Length of sync array (2 or 4).
 * @param zero Pointer to zero timing array.
 * @param zl Length of zero array (2 or 4).
 * @param one Pointer to one timing array.
 * @param ol Length of one array (2 or 4).
 * @param inv True to invert signal polarity.
 */
void RCSwitchRmt::setProtocolCustomRx(uint16_t T_us,
                                    const uint8_t* sync, uint8_t sync_len,
                                    const uint8_t* zero, uint8_t zero_len,
                                    const uint8_t* one,  uint8_t one_len,
                                    bool inverted){
  _cur.T_us     = T_us;
  _cur.inverted = inverted;

  _cur.sync.len = sync_len > 4 ? 4 : sync_len;
  for(uint8_t i=0;i<_cur.sync.len;i++) _cur.sync.t[i] = sync[i];

  _cur.zero.len = zero_len > 4 ? 4 : zero_len;
  for(uint8_t i=0;i<_cur.zero.len;i++) _cur.zero.t[i] = zero[i];

  _cur.one.len  = one_len  > 4 ? 4 : one_len;
  for(uint8_t i=0;i<_cur.one.len;i++) _cur.one.t[i]  = one[i];


  is_valid_seq_iqual(_cur.one.len, _cur.zero.len);


  if(!is_valid_seq_(_cur.sync) || !is_valid_seq_(_cur.zero) || !is_valid_seq_(_cur.one)){
#if RMT_OOK_DEBUG
    Serial.printf("[RX][PROTO][ERR] custom invalido. "
                  "T=%u inv=%u SL=%u ZL=%u OL=%u (solo 2/4 y sin ceros)\n",
                  (unsigned)T_us, (unsigned)inverted,
                  _cur.sync.len, _cur.zero.len, _cur.one.len);
#endif 
  }

  _nReceivedProtocol = 0; // 0 = custom
  uint8_t first = (_cur.sync.len>0) ? _cur.sync.t[0] : 1;
  
     _has_fixed_proto = true;
     _expected_clusters = number_simbols_expected(_cur,_expected_bitlen);
     refresh_rx_cfg_();
     update_sync_mask();     
     enableReceive();
}

/**
 * @brief Enables automatic RX protocol recognition.
 * 
 * The receiver will attempt to detect the most likely protocol from the table.
 * Manual definition (setProtocolRx) is faster and more accurate.
 */
void RCSwitchRmt::setProtocolAutoRx()
{   
    _has_fixed_proto = false;          // escuchar todos los de la tabla    
    _expected_clusters = 25;
    _max_ns_current = _NS_DEFAULT;
     update_sync_mask();
     refresh_rx_cfg_();
    enableReceive();   
   #if RMT_OOK_DEBUG
   Serial.printf("[AUTO] _max_ns_current:%lu",  _max_ns_current);
   #endif       
}

/**
 * @brief Defines the expected bit length for received codes.
 * @param n Number of bits expected per frame.
 */
void RCSwitchRmt::setExpectedBits(uint8_t n){
    _expected_bitlen = (n<8?8:(n>32?32:n));
    _expected_clusters = number_simbols_expected(_cur,_expected_bitlen);
     refresh_rx_cfg_();
     update_sync_mask();     
     enableReceive();
}

// ======================= TX =======================

/**
 * @brief Sets how many times each frame is repeated per send().
 * @param n Number of repetitions (n ≥ 1; n=1 = single frame)
 */
void RCSwitchRmt::setRepeatTransmit(uint16_t n){
  _repeat_tx = (n >= 1) ? n : 1;
#if RMT_OOK_DEBUG
  Serial.printf("[TX][CFG] setRepeatTransmit=%u\n", (unsigned)_repeat_tx);
#endif
}

/**
 * @brief Sets the minimum time duration for each send() operation.
 * @param ms Duration in milliseconds (0 disables time-based transmission)
 */
void RCSwitchRmt::setMinTransmitTime(uint32_t ms){
  _min_time_ms = ms;
#if RMT_OOK_DEBUG
  Serial.printf("[TX][CFG] setMinTransmitTime=%lu ms\n", (unsigned long)_min_time_ms);
#endif
}



/**
 * @brief Enables the transmitter on a given GPIO pin.
 * @param pin Output pin for TX signal.
 * @return True on success, false otherwise.
 */
bool RCSwitchRmt::enableTransmit(int pin)
{
  if(!_tx_proto_set){
    Protocol tmp{};
    memcpy_P(&tmp, &proto[0], sizeof(Protocol));
    // valida por si alguien tocó la tabla:
    if(!is_valid_seq_(tmp.sync) || !is_valid_seq_(tmp.zero) || !is_valid_seq_(tmp.one)){
#if RMT_OOK_DEBUG
      Serial.println(F("[TX][ERR] proto[0] invalido (pares len(2 o 4) y sin ceros requeridos)"));
#endif
      return false;
    }
    _cur_tx = tmp;
#if RMT_OOK_DEBUG
    Serial.println(F("[TX][INI] Protocolo TX por defecto: proto[0]"));
#endif
  }

  if(_tx_ch && _txPin != pin){
#if RMT_OOK_DEBUG
    Serial.printf("[TX][INI] Cambiando pin %d -> %d (recrear canal)\n", _txPin, pin);
#endif
    if(_tx_enabled){
      rmt_disable(_tx_ch);
      rmt_del_channel(_tx_ch); _tx_ch=nullptr;
    }  

     if(_tx_encoder){
    rmt_del_encoder(_tx_encoder); _tx_encoder=nullptr;
  }
  }
 
  if(!create_tx_channel_on_pin_(pin)){
#if RMT_OOK_DEBUG
    Serial.println(F("[TX][ERR] create_tx_channel_on_pin_ fallo"));
#endif
    return false;
  }

  if(!_tx_task_ready){
    _tx_task_ready = (xTaskCreatePinnedToCore(
      tx_task_trampoline_, "RMT-TX", 4096, this, 3, &_tx_task, tskNO_AFFINITY
    ) == pdPASS);
#if RMT_OOK_DEBUG
    Serial.printf("[TX][INI] Tarea TX %s\n", _tx_task_ready? "creada":"NO creada");
#endif
    if(!_tx_task_ready) return false;
  } else {
#if RMT_OOK_DEBUG
    Serial.println(F("[TX][INI] Tarea TX existente (reuse)"));
#endif
  }

  _tx_run     = true; 
  _tx_busy    = false;
  _tx_abort   = false;
  _txPin      = pin;

#if RMT_OOK_DEBUG
  Serial.printf("[TX][INI] enableTransmit pin=%d T=%uus inv=%u SL=%u ZL=%u OL=%u\n",
    pin, (unsigned)_cur_tx.T_us, (unsigned)_cur_tx.inverted,
    _cur_tx.sync.len, _cur_tx.zero.len, _cur_tx.one.len);
#endif
  return true;
}


void RCSwitchRmt::abortTx() {
  _tx_abort = true;         // bloquea relanzar en callbacks/startNextBurst_
  _tx_kick  = false;        // cancela cualquier “siguiente” pendiente

  if (_tx_ch && _tx_enabled && _tx_busy) {
    // IDF5: rmt_disable corta inmediatamente la TX y deja en INIT
    esp_err_t e = rmt_disable(_tx_ch);
    if (e == ESP_OK || e == ESP_ERR_INVALID_STATE) {
      _tx_busy = false;     // ya no hay transmisión en curso
      // Rehabilitar para dejar el canal listo (estado ENABLED)
      e = rmt_enable(_tx_ch);
      if (e == ESP_OK) _tx_enabled = true;
      else             _tx_enabled = false; // si fallara, se evita relanzar
    }
  }

  // suspender todo lo encolado:
  clearQueue();

#if RMT_OOK_DEBUG
  Serial.println(F("[RMT-TX] abortTx: corte inmediato (soft), cola vaciada"));
#endif
}


/**
 * @brief Disables transmitter and releases resources.
 * @return True on success.
 */
bool RCSwitchRmt::disableTransmit()
{
  
  _tx_run = false;
  if (_tx_ch && _tx_enabled){
    rmt_disable(_tx_ch);    
    rmt_del_channel(_tx_ch); _tx_ch=nullptr;
#if RMT_OOK_DEBUG
    Serial.println(F("[TX][INI] Canal TX deshabilitado"));
#endif
  }
  _tx_enabled=false;
  _tx_busy=false;

    if (_tx_encoder) {
    rmt_del_encoder(_tx_encoder);
    _tx_encoder = nullptr;
#if RMT_OOK_DEBUG
    Serial.println(F("[TX][INI] Encoder TX destruido"));
#endif
  }

  return true;
}

/*
bool RCSwitchRmt::create_tx_channel_on_pin_(int pin)
{
  if (_tx_ch && _txPin == pin) {
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-TX] reuse channel on same pin=%d\n", pin);
#endif
    
    if(!_tx_enabled){
      esp_err_t e = rmt_enable(_tx_ch);
      if (e != ESP_OK) {
#if RMT_OOK_DEBUG
        Serial.printf("[RMT-TX][ERR] rmt_enable(reuse)=%d\n", (int)e);
#endif
        return false;
      }
      _tx_enabled = true;
    }
    _txPin = pin;
    return true;
  }


  rmt_tx_channel_config_t tx_cfg = {
    .gpio_num          = (gpio_num_t)pin,
    .clk_src           = RMT_CLK_SRC_DEFAULT,
    .resolution_hz     = 1000000, // 1us
    .mem_block_symbols = 64,
    .trans_queue_depth = 2,
    .flags = { .invert_out = (uint32_t)(_cur.inverted ? 1U : 0U), .with_dma = 0U }
  };
  esp_err_t e = rmt_new_tx_channel(&tx_cfg, &_tx_ch);
  if(e != ESP_OK || !_tx_ch) return false;

  rmt_copy_encoder_config_t copy_cfg = {};
  e = rmt_new_copy_encoder(&copy_cfg, &_tx_encoder);
  if(e != ESP_OK || !_tx_encoder) return false;

  rmt_tx_event_callbacks_t tcbs = {}; tcbs.on_trans_done = &RCSwitchRmt::txDoneCB;
  e = rmt_tx_register_event_callbacks(_tx_ch, &tcbs, this);
  if(e != ESP_OK) return false;

  e = rmt_enable(_tx_ch);
  if(e != ESP_OK) return false;

  _txPin = pin;
  return true;
}
*/
bool RCSwitchRmt::create_tx_channel_on_pin_(int pin)
{
  // Reuse mismo canal/pin
  if (_tx_ch && _txPin == pin) {
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-TX] reuse channel on same pin=%d\n", pin);
#endif
    // Si el encoder fue liberado antes, recrearlo
 //   if(!_tx_encoder){
      rmt_copy_encoder_config_t copy_cfg = {};
      esp_err_t e = rmt_new_copy_encoder(&copy_cfg, &_tx_encoder);
      if(e != ESP_OK || !_tx_encoder){
#if RMT_OOK_DEBUG
        Serial.printf("[RMT-TX][ERR] rmt_new_copy_encoder(reuse)=%d\n", (int)e);
#endif
        return false;
      }
      // (Re)registrar callbacks de TX
      rmt_tx_event_callbacks_t tcbs = {};
      tcbs.on_trans_done = &RCSwitchRmt::txDoneCB;
      e = rmt_tx_register_event_callbacks(_tx_ch, &tcbs, this);
      if(e != ESP_OK){
#if RMT_OOK_DEBUG
        Serial.printf("[RMT-TX][ERR] rmt_tx_register_event_callbacks(reuse)=%d\n", (int)e);
#endif
        return false;
      }
      #if RMT_OOK_DEBUG
    Serial.println(F("[TX][REUSED] _tx_encoder"));
#endif
  //  }

    // Habilitar si estaba en INIT
    if(!_tx_enabled){
      #if RMT_OOK_DEBUG
    Serial.println(F("[TX][REUSED] rmt_enable(_tx_ch)"));
#endif
      esp_err_t e = rmt_enable(_tx_ch);
      if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
#if RMT_OOK_DEBUG
        Serial.printf("[RMT-TX][ERR] rmt_enable(reuse)=%d\n", (int)e);
#endif
        return false;
      }   
      _tx_enabled = true;   
    }
    _txPin = pin;
    return true;
  }

  // Crear canal nuevo
  rmt_tx_channel_config_t tx_cfg = {
    .gpio_num          = (gpio_num_t)pin,
    .clk_src           = RMT_CLK_SRC_DEFAULT,
    .resolution_hz     = 1000000, // 1 us
    .mem_block_symbols = 64,
    .trans_queue_depth = 2,
    .flags = { .invert_out = (uint32_t)(_cur_tx.inverted ? 1U : 0U), .with_dma = 0U } // <-- _cur_tx
  };
  esp_err_t e = rmt_new_tx_channel(&tx_cfg, &_tx_ch);
  if(e != ESP_OK || !_tx_ch) return false;

  rmt_copy_encoder_config_t copy_cfg = {};
  e = rmt_new_copy_encoder(&copy_cfg, &_tx_encoder);
  if(e != ESP_OK || !_tx_encoder) return false;

  rmt_tx_event_callbacks_t tcbs = {};
  tcbs.on_trans_done = &RCSwitchRmt::txDoneCB;
  e = rmt_tx_register_event_callbacks(_tx_ch, &tcbs, this);
  if(e != ESP_OK) return false;

  e = rmt_enable(_tx_ch);
  if(e != ESP_OK && e != ESP_ERR_INVALID_STATE) return false;


 #if RMT_OOK_DEBUG
    Serial.println(F("[TX][INI] rmt_enable(_tx_ch)"));
#endif

  _tx_enabled = true;
  _txPin = pin;
  return true;
}


/**
 * @brief Returns whether the transmitter is active or has queued frames.
 * @return True if busy or in progress.
 */
bool RCSwitchRmt::txActive() const
{
  return _tx_busy || (_q_count > 0);
}

bool RCSwitchRmt::applyTxProto_(const Protocol& np, int pin){
  bool need_recreate = (!_tx_ch) || (np.inverted != _cur_tx.inverted) || (_txPin!=pin);
  _cur_tx = np; _tx_proto_set = true;

  if (need_recreate){
    if(_tx_enabled) rmt_disable(_tx_ch);
    if(_tx_encoder){ rmt_del_encoder(_tx_encoder); _tx_encoder=nullptr; }
    if(_tx_ch){ rmt_del_channel(_tx_ch); _tx_ch=nullptr; }
    if(!create_tx_channel_on_pin_(pin)) return false;
  }
  _tx_abort=false; _tx_busy=false; _tx_run=true;
  return true;
}
/**
 * @brief Selects a predefined protocol from the internal table for transmission.
 * @param n Protocol index (1..PROTO_COUNT)
 * @return True if protocol valid and applied.
 */
bool RCSwitchRmt::setProtocolTx(int n){
  if(n < 1) n = 1;
  if(n > PROTO_COUNT) n = PROTO_COUNT;
  abortTx(); 
  Protocol tmp{};
  memcpy_P(&tmp, &proto[n-1], sizeof(Protocol));

  if(!is_valid_seq_(tmp.sync) || !is_valid_seq_(tmp.zero) || !is_valid_seq_(tmp.one)){
#if RMT_OOK_DEBUG
    Serial.printf("[TX][PROTO][ERR] proto=%d invalido (pares=2/4 y sin ceros). "
                  "SL=%u ZL=%u OL=%u\n",
                  n, tmp.sync.len, tmp.zero.len, tmp.one.len);
#endif
    return false;
  }

  _cur_tx = tmp;
  _tx_proto_set = true;
#if RMT_OOK_DEBUG
  Serial.printf("[TX][PROTO] setProtocolTx=%d T=%u inv=%u SL=%u ZL=%u OL=%u\n",
                n, (unsigned)_cur_tx.T_us, (unsigned)_cur_tx.inverted,
                _cur_tx.sync.len, _cur_tx.zero.len, _cur_tx.one.len);
#endif

  bool result =true;
   if(_txPin>0){ result =applyTxProto_(tmp, _txPin);}
  return result;
}

/**
 * @brief Defines a custom transmission protocol.
 * 
 * Each sequence (SYNC, ZERO, ONE) must contain 2 or 4 timing elements (HLHL pattern).
 * @param T_us Base pulse width in microseconds.
 * @param sync Pointer to sync sequence.
 * @param sync_len Length of sync sequence (2 or 4).
 * @param zero Pointer to zero sequence.
 * @param zero_len Length of zero sequence (2 or 4).
 * @param one Pointer to one sequence.
 * @param one_len Length of one sequence (2 or 4).
 * @param inverted True to invert output levels.
 * @return True if successfully configured.
 */
bool RCSwitchRmt::setProtocolCustomTx(uint16_t T_us,
                                      const uint8_t* sync, uint8_t sync_len,
                                      const uint8_t* zero, uint8_t zero_len,
                                      const uint8_t* one,  uint8_t one_len,
                                      bool inverted)
{
  abortTx(); 
  Protocol tmp{};
  tmp.T_us     = T_us;
  tmp.inverted = inverted;

  tmp.sync.len = (sync_len > 4) ? 4 : sync_len;
  for(uint8_t i=0;i<tmp.sync.len;i++) tmp.sync.t[i] = sync[i];

  tmp.zero.len = (zero_len > 4) ? 4 : zero_len;
  for(uint8_t i=0;i<tmp.zero.len;i++) tmp.zero.t[i] = zero[i];

  tmp.one.len  = (one_len  > 4) ? 4 : one_len;
  for(uint8_t i=0;i<tmp.one.len;i++)  tmp.one.t[i]  = one[i];


  is_valid_seq_iqual(tmp.one.len, tmp.zero.len);

  if(!is_valid_seq_(tmp.sync) || !is_valid_seq_(tmp.zero) || !is_valid_seq_(tmp.one)){
#if RMT_OOK_DEBUG
    Serial.printf("[TX][PROTO][ERR] custom invalido. "
                  "T=%u inv=%u SL=%u ZL=%u OL=%u (solo 2/4 y sin ceros)\n",
                  (unsigned)T_us, (unsigned)inverted,
                  tmp.sync.len, tmp.zero.len, tmp.one.len);
#endif
    return false;
  }

  _cur_tx = tmp;
  _tx_proto_set = true;
#if RMT_OOK_DEBUG
  Serial.printf("[TX][PROTO] custom T=%u inv=%u SL=%u ZL=%u OL=%u\n",
                (unsigned)_cur_tx.T_us, (unsigned)_cur_tx.inverted,
                _cur_tx.sync.len, _cur_tx.zero.len, _cur_tx.one.len);
#endif
    bool result =true;
   if(_txPin>0){ result =applyTxProto_(tmp, _txPin);}
   return result;
}


// ======== Cola y envío (política: n>=2 > tiempo>0 > SINGLE) ========

static inline uint8_t qnext_(uint8_t i, uint8_t cap){ return (uint8_t)((i+1) % cap); }

/**
 * @brief Sends a code with the configured protocol and behavior.
 * 
 * Behavior adapts based on configuration:
 *  - If setRepeatTransmit(n) > 1, sends n frames per call (burst mode)
 *  - If setMinTransmitTime(ms) > 0, sends continuously for ms milliseconds
 *  - If both are set, repetition count has priority
 * @param code Data to transmit.
 * @param bitlen Number of bits (up to 32).
 * @return True if transmission started successfully.
 */
bool RCSwitchRmt::send(uint32_t code, uint8_t bitlen)
{
  if(!_tx_ch || bitlen==0 || bitlen>MAXBITS) {
#if RMT_OOK_DEBUG
    Serial.printf("[TX][ERR] send invalido (ch=%p bits=%u)\n", (void*)_tx_ch, bitlen);
#endif
    return false;
  }
  if(_q_count >= QSIZE) {
#if RMT_OOK_DEBUG
    Serial.println(F("[TX][ERR] Cola llena"));
#endif
    return false;
  }

  TxItem it{};
  it.code   = code;
  it.bitlen = bitlen;
  it.used   = true;

  if(_repeat_tx >= 2){
    it.mode   = TxMode::BY_REPS;
    it.reps   = _repeat_tx;
    it.remain = _repeat_tx;
#if RMT_OOK_DEBUG
    Serial.printf("[TX][ENQ] code=0x%08lX bits=%u mode=BY_REPS reps=%u q=%u\n",
      (unsigned long)code, bitlen, (unsigned)_repeat_tx, (unsigned)_q_count+1);
#endif
  } else if(_min_time_ms > 0){
    it.mode    = TxMode::BY_TIME;
    it.min_ms  = _min_time_ms;
    it.started = false;
#if RMT_OOK_DEBUG
    Serial.printf("[TX][ENQ] code=0x%08lX bits=%u mode=BY_TIME ms=%lu q=%u\n",
      (unsigned long)code, bitlen, (unsigned long)_min_time_ms, (unsigned)_q_count+1);
#endif
  } else {
    it.mode   = TxMode::BY_REPS;
    it.reps   = 1;
    it.remain = 1;
#if RMT_OOK_DEBUG
    Serial.printf("[TX][ENQ] code=0x%08lX bits=%u mode=SINGLE q=%u\n",
      (unsigned long)code, bitlen, (unsigned)_q_count+1);
#endif
  }

  _q[_q_tail] = it;
  _q_tail = qnext_(_q_tail, QSIZE);
  _q_count++;

  if(_tx_task && _tx_run && !_tx_busy){
    xTaskNotifyGive(_tx_task);
#if RMT_OOK_DEBUG
    Serial.println(F("[TX][ENQ] Notificada tarea (inicio inmediato)"));
#endif
  }
  return true;
}

// ======== Tarea TX ========

void RCSwitchRmt::tx_task_trampoline_(void* arg){
  static_cast<RCSwitchRmt*>(arg)->tx_task_loop_();
}

void RCSwitchRmt::tx_task_loop_()
{
#if RMT_OOK_DEBUG
  Serial.println(F("[TX][TASK] Loop iniciado"));
#endif
  for(;;){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(!_tx_run) {
#if RMT_OOK_DEBUG
      Serial.println(F("[TX][TASK] Notificado, pero _tx_run=false (pausa)"));
#endif
      continue;
    }

    if(!_tx_busy && _q_count>0){
      TxItem& cur = _q[_q_head];

      if(cur.mode == TxMode::BY_REPS){
        if(cur.remain == 0){
#if RMT_OOK_DEBUG
          Serial.println(F("[TX][TASK] BY_REPS completado (pop)"));
#endif
          _q[_q_head].used=false;
          _q_head = qnext_(_q_head, QSIZE);
          _q_count--;
          continue;
        }
#if RMT_OOK_DEBUG
        Serial.printf("[TX][TASK] BY_REPS remain=%u -> startNextBurst_\n", (unsigned)cur.remain);
#endif
        startNextBurst_();
      } else {
        const uint32_t now = millis();
        if(!cur.started){ cur.t_start = now; cur.started = true;
#if RMT_OOK_DEBUG
          Serial.printf("[TX][TASK] BY_TIME start t=%lu ms min=%lu ms\n",
                        (unsigned long)cur.t_start, (unsigned long)cur.min_ms);
#endif
        }
        const bool listo = (uint32_t)(now - cur.t_start) >= cur.min_ms;
        if(listo){
#if RMT_OOK_DEBUG
          Serial.println(F("[TX][TASK] BY_TIME cumplido (pop)"));
#endif
          _q[_q_head].used=false;
          _q_head = qnext_(_q_head, QSIZE);
          _q_count--;
          continue;
        }
#if RMT_OOK_DEBUG
        Serial.println(F("[TX][TASK] BY_TIME continua -> startNextBurst_"));
#endif
        startNextBurst_();
      }
    }
  }
}

// ======== Notificación desde ISR / desde código ========

inline void RCSwitchRmt::notify_tx_task_(){
  if(_tx_task){
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(_tx_task, &hpw);
    if(hpw == pdTRUE) portYIELD_FROM_ISR();
  } else {
    _tx_kick = true; // fallback si la tarea aún no existe
  }
}


// cuántos símbolos RMT produce una secuencia (2 duraciones por símbolo)
static inline uint8_t seq_symbols_(const PulseSeq& s){
  return (uint8_t)(s.len / 2); // exige longitudes pares
}

// anexa una secuencia arbitraria HLHL... a sym[] (exige len par)
inline bool appendSeq_(const PulseSeq& s, uint16_t T,
                       uint8_t lvlH, uint8_t lvlL,
                       rmt_symbol_word_t* sym, size_t& k){
  if ((s.len & 1) != 0){
#if RMT_OOK_DEBUG
    Serial.println(F("[RMT-TX][WARN] seq.len impar: ajusta sync/zero/one a pares"));
#endif
    return false; // evita distorsionar el timing
  }
  for(uint8_t i=0;i<s.len;i+=2){
    sym[k].level0    = lvlH;
    sym[k].duration0 = (uint32_t)s.t[i]   * T;
    sym[k].level1    = lvlL;
    sym[k].duration1 = (uint32_t)s.t[i+1] * T;
    ++k;
  }
  return true;
}

void RCSwitchRmt::buildSymbolsAppend_(uint32_t code, uint8_t nbits,
                                      rmt_symbol_word_t* sym, size_t& k) const {
  const uint16_t T = _cur_tx.T_us;
#if RMT_OOK_DEBUG
  Serial.printf("[TX][BUILD] code=0x%08lX bits=%u T=%u\n",
                (unsigned long)code, nbits, (unsigned)T);
#endif
  // Bits MSB→LSB
  for(int i=nbits-1; i>=0; --i){
    const bool bit = (code >> i) & 1U;
    const PulseSeq& seq = bit ? _cur_tx.one : _cur_tx.zero;
    if (!appendSeq_(seq, T, _lvlH, _lvlL, sym, k)) return; // len impar ⇒ aborta
  }
  // SYNC final (delimitador entre repeticiones)
  (void)appendSeq_(_cur_tx.sync, T, _lvlH, _lvlL, sym, k);
}


/**
 * @brief Adds a frame to the TX queue for later transmission.
 * @param code Data to transmit.
 * @param bitlen Number of bits.
 * @param reps Repetitions (default 1).
 * @return True if added to queue successfully.
 */
bool RCSwitchRmt::queue(uint32_t code, uint8_t bitlen, uint16_t reps){
  if(!_tx_ch || bitlen==0 || bitlen>MAXBITS || reps==0) return false;
  if(_q_count>=QSIZE) return false;

  TxItem it{};
  it.code   = code;
  it.bitlen = bitlen;
  it.mode   = TxMode::BY_REPS;
  it.reps   = reps;
  it.remain = reps;
  it.used   = true;

  _q[_q_tail]=it; _q_tail=qnext_(_q_tail,QSIZE); _q_count++;
#if RMT_OOK_DEBUG
  Serial.printf("[TX][Q] enq code=0x%08lX bits=%u reps=%u qlen=%u\n",
                (unsigned long)code, bitlen, reps, (unsigned)_q_count);
#endif
  if(_tx_task && _tx_run && !_tx_busy){
    xTaskNotifyGive(_tx_task);
#if RMT_OOK_DEBUG
    Serial.println(F("[TX][Q] Notificada tarea"));
#endif
  }
  return true;
}

/**
 * @brief Clears the TX queue (only when transmitter is idle).
 */
void RCSwitchRmt::clearQueue(){
  if(_tx_busy) return;
  _q_head=_q_tail=_q_count=0;
  for(auto& e:_q){ e.used=false; e.remain=0; e.started=false; e.min_ms=0; }
#if RMT_OOK_DEBUG
  Serial.println(F("[TX][Q] Cola limpiada"));
#endif
}


bool RCSwitchRmt::startNextBurst_(){
  if(_q_count==0 || _tx_busy){
      #if RMT_OOK_DEBUG
    Serial.println(F("[TX][WRN] _q_count==0 || _tx_busy"));
  #endif
  return false;
  } 

    if(!_tx_ch){
  #if RMT_OOK_DEBUG
    Serial.println(F("[TX][ERR] _tx_ch"));
  #endif
    return false;
  }

    if(!_tx_enabled){
  #if RMT_OOK_DEBUG
    Serial.println(F("[TX][ERR] _tx_enabled"));
  #endif
    return false;
  }

      if(!_tx_encoder){
  #if RMT_OOK_DEBUG
    Serial.println(F("[TX][ERR] _tx_encoder"));
  #endif
    return false;
  }

        if(_tx_abort){
  #if RMT_OOK_DEBUG
    Serial.println(F("[TX] Abortar tx"));
  #endif
    return false;
  }

  // saltar BY_REPS agotados
  while(_q_count>0 && _q[_q_head].mode==TxMode::BY_REPS && _q[_q_head].remain==0){
    _q[_q_head].used=false; _q_head = (_q_head+1)%QSIZE; _q_count--;
  }
  if(_q_count==0){
     #if RMT_OOK_DEBUG
    Serial.println(F("[TX][WRN] _q_count==0"));
  #endif
  return false;
  } 

  TxItem &cur = _q[_q_head];

  // símbolos por bit = len/2 (ya validado 2 o 4)
  const uint8_t bit_syms  = (uint8_t)max(_cur_tx.zero.len, _cur_tx.one.len) / 2;
  const uint8_t sync_syms = (uint8_t)(_cur_tx.sync.len / 2);

  const size_t syms_per_frame = (size_t)bit_syms * cur.bitlen + sync_syms;
  if(syms_per_frame==0){
#if RMT_OOK_DEBUG
    Serial.println(F("[TX][ERR] syms_per_frame=0 (protocolo TX invalido)"));
#endif
    return false;
  }

  size_t reps_fit = _sym_cap / syms_per_frame;
  if(reps_fit==0){
#if RMT_OOK_DEBUG
    Serial.printf("[TX][ERR] frameSyms=%u > cap=%u\n",
                  (unsigned)syms_per_frame, (unsigned)_sym_cap);
#endif
    return false;
  }
  if(cur.mode==TxMode::BY_REPS && reps_fit > cur.remain) reps_fit = cur.remain;

  size_t k=0;
  for(size_t r=0; r<reps_fit; ++r){
    buildSymbolsAppend_(cur.code, cur.bitlen, _sym_buf, k);
  }

  _last_tx_syms_k = k;

#if RMT_OOK_DEBUG
  Serial.printf("[TX][BUILD] symbols=%u T=%uus reps_fit=%u frameSyms=%u\n",
                (unsigned)k, (unsigned)_cur_tx.T_us, (unsigned)reps_fit, (unsigned)syms_per_frame);
#endif

  if(k == 0){
#if RMT_OOK_DEBUG
    Serial.println(F("[TX][ERR] k=0: build fallido (paridad/ceros). No se transmite."));
#endif
    return false;
  }

  rmt_transmit_config_t txcfg = {
    .loop_count = 0,
    .flags = { .eot_level = (uint32_t)(_cur_tx.inverted?1:0) }
  };
  esp_err_t e = rmt_transmit(_tx_ch, _tx_encoder, _sym_buf, k*sizeof(rmt_symbol_word_t), &txcfg);
  if(e!=ESP_OK){
#if RMT_OOK_DEBUG
    Serial.printf("[TX][ERR] rmt_transmit=%d\n", (int)e);
#endif
    return false;
  }
  _tx_busy = true;

  if(cur.mode == TxMode::BY_REPS){
    cur.remain -= (uint16_t)reps_fit;
#if RMT_OOK_DEBUG
    Serial.printf("[TX][STATE] remain=%u\n", (unsigned)cur.remain);
#endif
  }
  return true;
}


void RCSwitchRmt::repeater(){
  // atiende cola TX 
  if (_tx_kick) { 
      _tx_kick = false; startNextBurst_(); // aquí decides si repites o avanzas en la cola 
  }
}


// ======== ISR de fin de TX: marcar libre y despertar tarea ========

IRAM_ATTR bool RCSwitchRmt::txDoneCB(rmt_channel_handle_t /*ch*/,
                                     const rmt_tx_done_event_data_t* /*ed*/,
                                     void* user)
{
  auto* self = static_cast<RCSwitchRmt*>(user);
  if (!self) return false;

  self->_tx_busy  = false;   // canal quedó libre
  if(!self->_tx_enabled || self->_tx_abort || !self->_tx_run){ self->_tx_kick=false; return false; }
  self->notify_tx_task_();   // pedir a la tarea decidir siguiente ráfaga / fin

  return true;
}

// ======================= RX =======================


#if !RMT_OOK_DEBUG
// Bucle de la tarea: espera en la cola y llama service(...)
void RCSwitchRmt::rx_task_loop_() {
  RxPkt pkt;

  // opcional: arrancar “siguiente” buffer
  rmt_symbol_word_t* next   = _rx_buf0;
  size_t             nextSz = sizeof(_rx_buf0);

  for (;;) {
    if (xQueueReceive(_rx_q, &pkt, portMAX_DELAY) == pdTRUE) {

      // 0) Aplicar cfg pendiente (barato, fuera de ISR)
      if (_rx_cfg_dirty) { _rx_cfg = _rx_cfg_next; _rx_cfg_dirty = false; }

      // 1) Elegir el buffer “next” (ping-pong) según el que llegó en pkt.p
      const volatile rmt_symbol_word_t* data = pkt.p;
      if (data== _rx_buf0) { next = _rx_buf1; nextSz = sizeof(_rx_buf1); }
      else                 { next = _rx_buf0; nextSz = sizeof(_rx_buf0); }

      // 2) Rearme YA (fuera de ISR; seguro en S2)
      esp_err_t e = rmt_receive(_rx_ch, next, nextSz, &_rx_cfg);
      if (e == ESP_ERR_INVALID_STATE) {
        rmt_disable(_rx_ch);
        rmt_enable(_rx_ch);
        (void) rmt_receive(_rx_ch, next, nextSz, &_rx_cfg);
      }

      // 3) Filtro aquí (no en ISR)
      if (pkt.n != _expected_clusters) {
        _drop_badlen++;
        continue; // ya rearmado, ignora el frame
      }

      // 4) Procesar el frame válido
      service(data, pkt.n);
    }
  }
}


// Trampolín estático -> llama al método de instancia
void RCSwitchRmt::rx_task_trampoline_(void* arg) {
  static_cast<RCSwitchRmt*>(arg)->rx_task_loop_();
}


// Crea la cola+tarea si aún no existen
bool RCSwitchRmt::ensure_service_task_() {
  if (!_rx_q) {
    _rx_q = xQueueCreate(4, sizeof(RxPkt));
    if (!_rx_q) return false;
  }
  if (!_rx_task) {
    BaseType_t ok = xTaskCreate(&RCSwitchRmt::rx_task_trampoline_,
                                "rmtRXproc",
                                4096,         // stack (ajústalo luego midiendo)
                                this,         // arg: la instancia
                                10,           // prioridad
                                &_rx_task);
    if (ok != pdPASS) return false;
  }
  return true;
}
#endif


/**
 * @brief Enables the receiver on the specified GPIO pin.
 * @param pin Input pin for RX signal.
 * @return True on success, false otherwise.
 */
bool RCSwitchRmt::enableReceive(int pin)
{
#if !RMT_OOK_DEBUG
  if (!ensure_service_task_()) return false; // si tienes tu tarea de servicio RX
#endif

  // Cambió pin -> destruir y recrear
  if (_rx_ch && _rxPin != pin) {
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-RX] switching pin %d -> %d (destroy & recreate)\n", _rxPin, pin);
#endif
    if(_rx_enabled) rmt_disable(_rx_ch);
    rmt_del_channel(_rx_ch);
    _rx_ch = nullptr;
    _rx_enabled = false;
  }

  // Reuse mismo pin -> (re)habilitar
  if (_rx_ch && _rxPin == pin) {
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-RX] reuse existing channel on pin=%d (ensure ready)\n", pin);
#endif
    bool ok = ensure_rx_ready_();
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-RX] reuse ensure_ready=%s\n", ok?"OK":"FAIL");
#endif
    return ok;
  }

  // Crear + habilitar + armar
  if (!create_rx_channel_on_pin_(pin)) {
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-RX][ERR] create_rx_channel_on_pin_(%d) failed\n", pin);
#endif
    return false;
  }
  bool ok = ensure_rx_ready_();
#if RMT_OOK_DEBUG
  Serial.printf("[RMT-RX] enabled pin=%d tol=%u%% -> %s\n", _rxPin, _rx_tol, ok?"OK":"FAIL");
#endif
  return ok;
}



/**
 * @brief Enables RX using the previously configured pin.
 * @return True if receiver started successfully.
 */
bool RCSwitchRmt::enableReceive()
{
  if (_rxPin < 0) {
#if RMT_OOK_DEBUG
    Serial.println("[RMT-RX][ERR] enableReceive(): _rxPin < 0");
#endif
    return false;
  }
#if RMT_OOK_DEBUG
  Serial.printf("[RMT-RX] enableReceive() using stored pin=%d\n", _rxPin);
#endif
  return enableReceive(_rxPin);
}

/**
 * @brief Disables the receiver and frees resources.
 * @return True if successfully stopped.
 */
bool RCSwitchRmt::disableReceive()
{
  if(_rx_ch && _rx_enabled){
    rmt_disable(_rx_ch);
#if RMT_OOK_DEBUG
    Serial.println(F("[RMT-RX] channel disabled"));
#endif
  }
  _rx_enabled = false;
  // Si quieres liberar recursos completamente, descomenta:
  // if(_rx_ch){ rmt_del_channel(_rx_ch); _rx_ch=nullptr; }
  // _rxPin = -1;
  _cc = 0;
  return true;
}


bool RCSwitchRmt::create_rx_channel_on_pin_(int pin)
{
  // Si existe y cambia de pin -> destruir y recrear
  if (_rx_ch && _rxPin != pin) {
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-RX] switching pin %d -> %d (destroy & recreate)\n", _rxPin, pin);
#endif
    if(_rx_enabled) rmt_disable(_rx_ch);
    rmt_del_channel(_rx_ch);
    _rx_ch = nullptr;
    _rx_enabled = false;
  }

  // Reuse: mismo canal y mismo pin -> solo (re)habilitar
  if (_rx_ch && _rxPin == pin) {
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-RX] reuse channel on pin=%d (enable)\n", pin);
#endif
    esp_err_t e = rmt_enable(_rx_ch);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
#if RMT_OOK_DEBUG
      Serial.printf("[RMT-RX][ERR] rmt_enable(reuse)=%d\n", (int)e);
#endif
      return false;
    }
    _rx_enabled = true;
    _cc = 0;
    return true;
  }

  // Crear canal nuevo
#if RMT_OOK_DEBUG
  Serial.printf("[RMT-RX] new channel on pin=%d (res=1us, invert_in=%u)\n",
                pin, (unsigned)(_cur.inverted ? 1U : 0U));
#endif
  rmt_rx_channel_config_t cfg = {};
  cfg.gpio_num          = (gpio_num_t)pin;
  cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
  cfg.resolution_hz     = 1000000;          // 1 us
  cfg.mem_block_symbols = RX_RAW_BUF_SYMS;  // p.ej. 64
  cfg.flags.invert_in   = (uint32_t)(_cur.inverted ? 1U : 0U); // ojo: usa _cur (RX)
  cfg.flags.with_dma    = 0U;

  if (rmt_new_rx_channel(&cfg, &_rx_ch) != ESP_OK || !_rx_ch) {
#if RMT_OOK_DEBUG
    Serial.println("[RMT-RX][ERR] rmt_new_rx_channel");
#endif
    return false;
  }

  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = &RCSwitchRmt::rxDoneCB;
  esp_err_t e = rmt_rx_register_event_callbacks(_rx_ch, &cbs, this);
  if (e != ESP_OK) {
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-RX][ERR] register_cb=%d\n", (int)e);
#endif
    return false;
  }

  e = rmt_enable(_rx_ch);
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-RX][ERR] rmt_enable(new)=%d\n", (int)e);
#endif
    return false;
  }

  _rx_enabled = true;
  _rxPin = pin;
  _cc = 0;
  return true;
}


bool RCSwitchRmt::ensure_rx_ready_()
{
  if (!_rx_ch) {
#if RMT_OOK_DEBUG
    Serial.println("[RMT-RX][ERR] ensure_rx_ready_: no channel");
#endif
    return false;
  }

  // Habilitar si hiciera falta (idempotente)
  esp_err_t e = rmt_enable(_rx_ch);
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-RX][ERR] rmt_enable=%d\n", (int)e);
#endif
    return false;
  }

  refresh_rx_cfg_(); // actualiza _rx_cfg_next y marca _rx_cfg_dirty

  rmt_receive_config_t rxcfg = {};
  rxcfg.signal_range_min_ns = 3100;
  rxcfg.signal_range_max_ns = _max_ns_current;

  // Armar captura (dos intentos si el driver reporta INVALID_STATE)
  for (int attempt = 0; attempt < 2; ++attempt) {
    e = rmt_receive(_rx_ch, _rx_buf, sizeof(_rx_buf), &rxcfg);
    if (e == ESP_OK) {
#if RMT_OOK_DEBUG
      Serial.printf("[RMT-RX] rmt_receive armed (attempt=%d), _expected_clusters:%u, _max_ns_current:%u\n",
                    attempt+1, _expected_clusters, _max_ns_current);
#endif
      _rx_enabled    = true;
      _cc            = 0;
      _rx_cfg        = _rx_cfg_next;
      _rx_cfg_dirty  = false;
      return true;
    }
    if (e == ESP_ERR_INVALID_STATE) {
#if RMT_OOK_DEBUG
      Serial.printf("[RMT-RX][WARN] rmt_receive INVALID_STATE (attempt=%d) -> re-enable\n", attempt+1);
#endif
      rmt_disable(_rx_ch);
      rmt_enable(_rx_ch);
      continue;
    }
#if RMT_OOK_DEBUG
    Serial.printf("[RMT-RX][ERR] rmt_receive=%d (attempt=%d)\n", (int)e, attempt+1);
#endif
    break;
  }
  return false;
}


/**
 * @brief Returns whether a valid frame is available.
 * @return True if data ready to read.
 */
#if RMT_OOK_DEBUG
bool RCSwitchRmt::available() {
  // ¿hay un frame pendiente?
  if (!_dbg_ready) return false;

  // Rearmar ANTES de procesar (ping-pong): elegir el otro buffer
  const volatile rmt_symbol_word_t* cur = _dbg_p;
  rmt_symbol_word_t* next = (cur == _rx_buf0) ? _rx_buf1 : _rx_buf0;
  size_t nextSz = (next == _rx_buf0) ? sizeof(_rx_buf0) : sizeof(_rx_buf1);

  // Rearme fuera de ISR (seguro en S2)
  esp_err_t e = rmt_receive(_rx_ch, next, nextSz, &_rx_cfg);
  if (e == ESP_ERR_INVALID_STATE) {
    rmt_disable(_rx_ch); rmt_enable(_rx_ch);
    (void) rmt_receive(_rx_ch, next, nextSz, &_rx_cfg);
  }

  // (Opcional) filtro aquí mismo
  if (_dbg_n != _expected_clusters) {
    _dbg_ready = false;   // descartado, ya quedó rearmado
    return false;
  }

  // Procesar en contexto de loop (puedes loguear seguro)
  service(cur, _dbg_n);

  // Consumido
  _dbg_ready = false;

  // Semántica legacy: available() verdadero si el “último” está sin reset
  return _nReceivedBitlength != 0;
}
#endif



/**
 * @brief Reads the latest decoded frame into a structure.
 * @param out Pointer to Decoded structure to fill.
 * @return True if valid data was retrieved.
 */
bool RCSwitchRmt::read(Decoded* out) {
  if (!out) return false;
  bool ok = false;

  portENTER_CRITICAL(&_rbMux);
  if (_rb_count > 0) {
    *out = _rb[_rb_tail];
    _rb_tail = (_rb_tail + 1) % MAX_ELEMENTS;
    _rb_count--;
    ok = true;
  }
  portEXIT_CRITICAL(&_rbMux);

  return ok;
}

/**
 * @brief Clears the RX buffer and all pending decoded data.
 */
void RCSwitchRmt::clearBuffer() {
  portENTER_CRITICAL(&_rbMux);
  _rb_head = _rb_tail = _rb_count = 0;
  // Opcional: limpiar memoria para depuración/privacidad
  for (size_t i = 0; i < MAX_ELEMENTS; ++i) _rb[i] = {};
  portEXIT_CRITICAL(&_rbMux);
}



// Construye un vector de "timings" (duraciones entre flancos) a partir de símbolos RMT.
// idle_active_high = ! _p.inverted (reposo LOW → activo HIGH).
// Normaliza cada símbolo a (activo, reposo) y los “aplana” a timings[]
static size_t sym_to_timings_(const rmt_symbol_word_t* s, size_t n,
                              bool active_high, uint32_t* tbuf, size_t tcap){
  size_t k=0; const uint8_t ACTIVE = active_high?1:0;
  for(size_t i=0;i<n && k+1<tcap;++i){
    if(s[i].level0==ACTIVE){ tbuf[k++]=s[i].duration0; tbuf[k++]=s[i].duration1; }
    else                   { tbuf[k++]=s[i].duration1; tbuf[k++]=s[i].duration0; }
  }
  return k;
}


bool IRAM_ATTR RCSwitchRmt::rxDoneCB(rmt_channel_handle_t,
                                     const rmt_rx_done_event_data_t* edata,
                                     void* user_ctx)
{
  auto* self = static_cast<RCSwitchRmt*>(user_ctx);
  BaseType_t hpw = pdFALSE;

  // aplicar cfg precalculada (barato)
  if (self->_rx_cfg_dirty) { self->_rx_cfg = self->_rx_cfg_next; self->_rx_cfg_dirty = false; }

#if RMT_OOK_DEBUG
  // Modo debug: NO rearma, NO cola. Solo deja el frame “pendiente”
  self->_dbg_p = edata->received_symbols;
  self->_dbg_n = edata->num_symbols;
  self->_dbg_ready = true;
  return false; // no hace falta hacer yield
#else
  // Modo producción: encolar para la tarea
  RxPkt pkt{ edata->received_symbols, edata->num_symbols };
  if (self->_rx_q) xQueueSendFromISR(self->_rx_q, &pkt, &hpw);
  return hpw == pdTRUE;
#endif
}


//+++++++++++++++++++++++++++++++++++++++
// compara v[k]*T con t[pos+k] con tolerancia
static inline bool match_scaled_seq_(const uint8_t* v, uint8_t len,
                                     const uint32_t* t, unsigned pos, unsigned cc,
                                     uint16_t T, uint8_t tol_pct){
  if (len==0) return true;
  if (pos + len > cc) return false;
  for (uint8_t k=0;k<len;k++){
    uint32_t ref = (uint32_t)v[k] * (uint32_t)T;
    uint32_t tol = (ref * tol_pct)/100u;
    uint32_t x   = t[pos+k];
    if ( (x>ref? x-ref : ref-x) > tol ) return false;
  }
  return true;
}



struct SyncSeg { uint8_t prefIdx, prefLen, sufIdx, sufLen, imax; };
// devuelve índices y longitudes de prefijo/sufijo en _sync_mask
static inline SyncSeg split_mask_(const uint8_t* m, uint8_t L){
  SyncSeg s{0,0,0,0,0};
  // hallar imax (posición del 255)
  for(uint8_t i=0;i<L;i++) if (m[i]==255){ s.imax=i; break; }
  // prefijo = elementos después de 255
  s.prefIdx = (uint8_t)(s.imax+1);
  s.prefLen = (s.prefIdx<L)? (L - s.prefIdx) : 0;
  // sufijo = elementos antes de 255
  s.sufIdx  = 0;
  s.sufLen  = s.imax;
  return s;
}

// Comparar secuencia con T
bool RCSwitchRmt::match_seq(const PulseSeq& seq, const uint32_t* t, unsigned i, unsigned cc,
                             uint16_t T, uint32_t tol_T){
  if (i + seq.len > cc) return false;
  for (unsigned k=0;k<seq.len;k++){
    uint32_t ref = (uint32_t)seq.t[k] * (uint32_t)T;  
  uint32_t tol_bit = tol_T*seq.t[k];
    if (adiffu(t[i+k], ref) > tol_bit) {        
      return false;}
  }
  return true;    
}


// ---- helpers locales -------------------------------------------------
inline uint32_t RCSwitchRmt::compute_max_ns_for_proto_(const Protocol& P) const {
  // max_ns = 0.6 * (max elemento del SYNC) * T_us  → en nanosegundos
  uint8_t m = 0;
  for (uint8_t i=0; i<P.sync.len; ++i) if (P.sync.t[i] > m) m = P.sync.t[i];
  if (m == 0) m = 1;
  uint32_t us = (uint32_t)m * (uint32_t)P.T_us * 6u / 10u;
  if (us < 1000u) us = 1000u;          // piso 1 ms para evitar ISR demasiado frecuente
  #if RMT_OOK_DEBUG
  Serial.printf("[MAX_N] compute_max_ns=%lu", us * 1000u); 
  Serial.println();
#endif
  return us * 1000u;                    // a ns
}

inline void RCSwitchRmt::init_sync_mask_from_proto_(const Protocol& P) {
  // Copia la secuencia de SYNC al _sync_mask[] y marca el mayor con 255
  _sync_len = P.sync.len;
  for (uint8_t i=0; i<4; ++i) _sync_mask[i] = 0;       // limpia
  uint8_t maxv = 0; uint8_t maxi = 0;
  for (uint8_t i=0; i<_sync_len; ++i) {
    _sync_mask[i] = P.sync.t[i];
    if (P.sync.t[i] > maxv) { maxv = P.sync.t[i]; maxi = i; }
  }
  // Marcamos el mayor (el que RMT "corta") con 255 (sentinela)
  _sync_mask[maxi] = (int8_t)255;
#if RMT_OOK_DEBUG
  Serial.printf("[MASK] sync_len=%u mask={", _sync_len);
  for (uint8_t i=0;i<_sync_len;i++) { Serial.printf("%s%d", i?",":"", (int)_sync_mask[i]); }
  Serial.println("}");
#endif
}


void RCSwitchRmt::refresh_rx_cfg_() {
  rmt_receive_config_t cfg{};
  cfg.signal_range_min_ns = 3100;   // ~3 µs anti-ruido duro

  if (_has_fixed_proto) {
    _max_ns_current = compute_max_ns_for_proto_(_cur);
    cfg.signal_range_max_ns = _max_ns_current;
  } else {
    cfg.signal_range_max_ns = _NS_DEFAULT; // 6.2 ms en ns (auto)
  }

  // Publica como “siguiente” y marca su aplicación
  portENTER_CRITICAL(&_cfgMux);
  _rx_cfg_next  = cfg;
  _rx_cfg_dirty = true;
  portEXIT_CRITICAL(&_cfgMux);
}


// Valida según modo y, si corresponde, publica.
// Devuelve true si quedó publicado (available()=true).
bool RCSwitchRmt::maybe_publish_(uint32_t code, uint8_t bits, uint16_t T_us, uint8_t proto_idx){
  if (bits < _min_bits_publish) { _cand_bits=0; return false; }

  switch(_vMode){
    case ValidationMode::LONG_RANGE:            // 1 sola trama
      return publish_now_(code,bits,T_us,proto_idx);

    case ValidationMode::RELIABLE:              // exige 2 iguales
      if (_cand_bits==bits && _cand_code==code) {
       _cand_code=code; _cand_bits=bits;         // guarda candidato y espera la siguiente
        return publish_now_(code,bits,T_us,proto_idx);
      }
    
      _cand_code=code; _cand_bits=bits;         // guarda candidato y espera la siguiente
   
   #if RMT_OOK_DEBUG
   Serial.printf("[PLB] _cand_code=%lu _cand_bits:%u", code, _cand_bits);
   Serial.println();
  #endif

      return false;

    case ValidationMode::SAFE_MODERATE_RANGE:   // publica 1; si dudas, eleva a RELIABLE
      // (si implementas heurísticas de “duda”, llama primero a RELIABLE; aquí publicamos directo)
      _cand_bits=0;
      return publish_now_(code,bits,T_us,proto_idx);
  }
  return false;
}

// Publica inmediatamente (lo que leerá el usuario con available()/get*)
bool RCSwitchRmt::publish_now_(uint32_t code, uint8_t bits, uint16_t T_us, uint8_t proto_idx){
     #if RMT_OOK_DEBUG
  Serial.printf("[PL] code=%lu", code);
   Serial.println();
  #endif
  _nReceivedValue     = code;
  _nReceivedBitlength = bits;        // <- esto hace que available() sea true
  _nReceivedDelay     = T_us;
  _nReceivedProtocol  = proto_idx;   // 1..N si viene de la tabla; 0 si “custom/auto”
  _t_last_frame_ok=millis();

   Decoded d{ code, bits, T_us, proto_idx, _t_last_frame_ok };

  portENTER_CRITICAL(&_rbMux);
  // Si está lleno, sobreescribe el más antiguo (política overwrite)
  if (_rb_count == MAX_ELEMENTS) {
    _rb_tail = (_rb_tail + 1) % MAX_ELEMENTS;
    _rb_count--;
  }
  _rb[_rb_head] = d;
  _rb_head = (_rb_head + 1) % MAX_ELEMENTS;
  _rb_count++;
  portEXIT_CRITICAL(&_rbMux);

  return true;
}


// Ajusta T usando IIMF sobre t[i..j-1]. T0 puede ser 0 (se autoinicializa).
// tol_pct: tolerancia (p.ej. 60); iters: 2–4 suele bastar.
// Devuelve T (us) refinado o 0 si no hay datos válidos.
uint16_t RCSwitchRmt::refine_T_iimf_(const uint32_t* t, unsigned i, unsigned j,
                                     uint16_t T0, uint8_t tol_pct, uint8_t iters){
  if (!t || i >= j) return 0;

  // --- 1) T inicial ---
  uint32_t T = T0;
  if (T == 0){
    // inicializa con el mínimo (aprox. 1T) del segmento
    uint32_t mn = t[i];
    for (unsigned k=i+1; k<j; ++k) if (t[k] < mn) mn = t[k];
    if (mn == 0) return 0;
    T = mn;
  }

  // --- 2) Iteraciones IIMF ---
  for (uint8_t it=0; it<iters; ++it){
    const uint32_t tol_abs = (T * tol_pct) / 100u;

    // Acumuladores de mínimos cuadrados: T = sum(n*d)/sum(n^2)
    uint64_t sum_nd = 0;
    uint64_t sum_n2 = 0;
    uint16_t accepted = 0;

    for (unsigned k=i; k<j; ++k){
      uint32_t d = t[k];
      if (d == 0) continue;

      // múltiplo entero más cercano de T
      uint32_t n = (uint32_t)((d + T/2) / T);  // round(d/T)
      if (n == 0) n = 1;

      int32_t resid = (int32_t)d - (int32_t)(n * T);
      if ((uint32_t)(resid < 0 ? -resid : resid) > tol_abs) {
        // fuera de tolerancia → descarta
        continue;
      }

      sum_nd += (uint64_t)n * (uint64_t)d;
      sum_n2 += (uint64_t)n * (uint64_t)n;
      ++accepted;
    }

    if (accepted < 3 || sum_n2 == 0) break; // no hay base suficiente

    uint32_t T_new = (uint32_t)(sum_nd / sum_n2);

    // Convergencia rápida
    if (T_new == 0) break;
    if (T_new == T || (T_new > T ? (T_new - T) : (T - T_new)) <= 1) {
      T = T_new; 
      break;
    }
    T = T_new;
  }

  if (T > 0xFFFFu) T = 0xFFFFu;
  return (uint16_t)T;
}


bool RCSwitchRmt::try_decode_with_proto_(const Protocol& P,
                                         uint32_t& code, uint8_t& bits, uint16_t& T_us)
{
  code = 0; bits = 0; T_us = 0;
  if (_cc < 3) return false;
 

  // === 1) Construir prefijo/sufijo esperados a partir de _sync_mask ===
  // Regla: elementos DESPUÉS del “agujero” (255) aparecen al INICIO del buffer (prefijo),
  //        elementos ANTES del agujero aparecen al FINAL del buffer (sufijo).
  uint8_t pref[4]; uint8_t prefLen=0;
  uint8_t suf [4]; uint8_t sufLen =0;

  // Determinar posición del "hole" 255 y longitud real del sync (0..4)
  uint8_t L = 0; while (L<4 && _sync_mask[L]!=0) L++;
  int8_t hole = -1;
  for (uint8_t i=0;i<L;i++) { if (_sync_mask[i]==255){ hole = (uint8_t)i; break; }}
 
   #if RMT_OOK_DEBUG
  Serial.printf("[Dec] hole=%u", hole);
  #endif

  if (hole >= 0){
    if(hole>0){
     // sufijo:  elementos antes del hole
    for (int8_t i=0; i<hole; i++)  { suf[sufLen++]  = _sync_mask[i];}
    }

    if(hole<=3){
    // prefijo: elementos después del hole    
    for (uint8_t i=hole+1; i<L; i++) {pref[prefLen++] = _sync_mask[i]; }
    }
   
  } else {
    // Si no hay “hole” marcado, no imponemos pref/suf (quedarán en 0)
    prefLen = 0; sufLen = 0;
  }

     #if RMT_OOK_DEBUG
  Serial.printf("[Dec] prefLen=%u sufLen=%u", prefLen, sufLen);
  #endif

  // === 2) Estimar T_us usando prefijo/sufijo si existen ===
  auto est_T_from_seg = [&](const uint8_t* seg, uint8_t segLen,
                            const uint32_t* t, unsigned off)->uint16_t {
    // media de razones t[k]/seg[k]
    if (segLen==0) return 0;
    float num = 0; float cnt = 0;
    for (uint8_t k=0;k<segLen;k++){
      float m = seg[k];
      if (m==0) return 0;
   //   Serial.println();
   //   Serial.printf("[Dec] t[off+k]=%lu m=%f ", t[off+k], m);
      num += (t[off+k] / m);      
      cnt = cnt + 1;      
    }
    if (!cnt) return 0;
    float T = num / cnt;
   #if RMT_OOK_DEBUG
  Serial.printf("[Dec] num=%f cnt=%f, T:%f", num, cnt, T);
  #endif

    return (T>0xFFFFu)? 0xFFFFu : (uint16_t)T;
  };
  

  uint16_t Tp = 0, Ts = 0;
  if (prefLen && _cc >= prefLen)           Tp = est_T_from_seg(pref, prefLen, _timings, 0);
  if (sufLen  && _cc >= sufLen)            Ts = est_T_from_seg(suf,  sufLen,  _timings, _cc - sufLen);

   #if RMT_OOK_DEBUG
  Serial.printf("[Dec] Tp=%lu Ts=%lu", Tp, Ts);
  #endif

  if (Tp && Ts) T_us = (uint16_t)((uint32_t)Tp + (uint32_t)Ts)/2u;
  else if (Tp)  T_us = Tp;
  else if (Ts)  T_us = Ts;
  else {
    // Fallback: usa la menor duración como base aproximada de T
    uint32_t mn = _timings[0];
    for (unsigned i=1;i<_cc;i++) if (_timings[i]<mn) mn=_timings[i];
    T_us = (uint16_t)mn;
  }
  unsigned i = prefLen;                 // inicio real de payload
  unsigned j = (_cc > sufLen)? (_cc - sufLen) : 0; // fin real (excluyente)
  uint16_t Tfina =  refine_T_iimf_(_timings, i, j, T_us, _rx_tol, 4);

  #if RMT_OOK_DEBUG
  Serial.printf("[Dec] Tus=%lu, Tfina:%lu", T_us, Tfina);
  #endif

  if (Tfina>0){
     T_us= Tfina;
  }

  if (T_us == 0) return false;
  const uint32_t tol_abs = ((uint32_t)T_us * _rx_tol) / 100u;

  // === 3) Validar que el prefijo/sufijo (si existen) coinciden dentro de tolerancia ===
  // prefijo al inicio
  if (prefLen){
      #if RMT_OOK_DEBUG 
      Serial.println("[Dec] PreOK");
      #endif
    for (uint8_t k=0;k<prefLen;k++){
      uint32_t ref = (uint32_t)pref[k]*T_us;
      if (adiffu(_timings[k], ref) > tol_abs) return false;
    }
  }
  // sufijo al final
  if (sufLen){
          #if RMT_OOK_DEBUG 
      Serial.println("[Dec] SufOK");
      #endif
    for (uint8_t k=0;k<sufLen;k++){
      uint32_t ref = (uint32_t)suf[k]*T_us;
      uint32_t v   = _timings[_cc - sufLen + k];
      if (adiffu(v, ref) > tol_abs) return false;
    }
  }

  // === 4) Decodificar bits entre prefijo y sufijo ===


   #if RMT_OOK_DEBUG
  Serial.printf("[Dec] ini=%u, fin=%u", i,j);
  #endif
  if (j <= i) return false;

  

  code = 0; bits = 0;
  while (i <= j && bits < (_expected_bitlen+1)){
    if (match_seq(P.zero, _timings, i, j, T_us, tol_abs)) { code <<= 1; bits++; i += P.zero.len;     }
    else if (match_seq(P.one,  _timings, i, j, T_us, tol_abs)) { code = (code<<1)|1u; bits++; i += P.one.len;     }
    else {
       // YA NO: if (match_seq(P.sync, ...)) ...
  // → Con RMT “cortado”, puede quedar sólo residuo de SYNC, no SYNC completo.
      
  if (bits == 0) {
    // tolerar basura/residuo previo: avanza 1 duración y reintenta
    i += 1;
    continue;
  } else {
    // ya venías leyendo bits: termina el frame
    break;
    }
  }
 }

   #if RMT_OOK_DEBUG
  Serial.printf("[Dec] bits=%u", bits);
  #endif

  if (bits < 8) return false;

  // === 5) Publicación se hace afuera; aquí sólo confirmamos éxito ===
#if RMT_OOK_DEBUG
  Serial.printf("[TRY] bits=%u T=%u (pref=%u suf=%u cc=%u)\n", bits, T_us, prefLen, sufLen, _cc);
#endif
  return true;
}

/*
bool RCSwitchRmt::better_than_locked_(uint8_t cand_idx, uint16_t cand_T, uint32_t cand_err){
  if (_proto_locked==0) return true;  // no hay lock aún
  // Heurística simple: si el candidato trae error “mucho” menor se permite re-lock
  // Para algo rápido, compara contra un error base derivado de T bloqueado:
  uint32_t base_err = (uint32_t)_locked_T * 4; // placeholder (ajusta si ya calculas score real)
  return (cand_err + (cand_err * RMT_OOK_RELOCK_MARGIN_PCT)/100u) < base_err;
}
*/

void RCSwitchRmt::sym_to_timings_append_(const volatile rmt_symbol_word_t* s, size_t n){  
  for(size_t i=0;i<n;i++){
    uint32_t d0 = s[i].duration0;
    uint32_t d1 = s[i].duration1;
    if (d0 >= _noise_min_us && _cc < TIMINGS_CAP) _timings[_cc++] = d0;
    if (d1 >= _noise_min_us && _cc < TIMINGS_CAP) _timings[_cc++] = d1; // si es 0, se ignora
  }
#if RMT_OOK_DEBUG
  Serial.printf("[TIM] append n=%u cc=%u\n",(unsigned)n,(unsigned)_cc);
#endif
}

// ---- service() -------------------------------------------------------

void RCSwitchRmt::service(const volatile rmt_symbol_word_t* syms, size_t n)
{
  // 1) Reset de acumuladores de timings del decoder
  _cc = 0;               // contador de timings válidos
  // 2) Convertir símbolos RMT -> timings (llena tus buffers internos y setea _cc)
  sym_to_timings_append_(syms, n);


      #if RMT_OOK_DEBUG  
      Serial.printf("[RX] n=%u  cc=%u\n", (unsigned)n, (unsigned)_cc);
  // volcado opcional corto
 
  Serial.println("[RX-RAW] (dur0,dur1)...");
  for (size_t i=0;i<_expected_clusters;i++){
    Serial.printf("%u:%u,%u  ", (unsigned)i, (unsigned)syms[i].duration0, (unsigned)syms[i].duration1);
    if ((i%8)==7) Serial.println();
  }
  Serial.println();
  
#endif



  // 3) Si no hubo timings útiles, salir
  if (_cc == 0) return;

  // 4) Decodificación
  if (_has_fixed_proto) {
    // ===== MODO FIJO =====
    uint32_t code = 0; uint8_t bits = 0; uint16_t T_us = 0;
    if (try_decode_with_proto_(_cur, code, bits, T_us)) {
      // Nota: el índice de protocolo "fijo" puedes guardarlo en _nReceivedProtocol o derivarlo de _cur
      uint8_t proto_idx = _nReceivedProtocol; // o el que corresponda a _cur
      publish_now_(code, bits, T_us, proto_idx);
    }
    _cc = 0;
    return;
  } else {
    // ===== MODO AUTOMÁTICO =====
    for (int idx = 0; idx < PROTO_COUNT; ++idx) {
      Protocol P; memcpy_P(&P, &proto[idx], sizeof(P));
      init_sync_mask_from_proto_(P);

      uint32_t code = 0; uint8_t bits = 0; uint16_t T_us = 0;
      if (try_decode_with_proto_(P, code, bits, T_us)) {
        publish_now_(code, bits, T_us, (uint8_t)(idx + 1));
        _cc = 0;
        return;
      }
    }
    _cc = 0; // no decodificó con ningún proto
  }
}

//++++++++++++++++++++++++++++++++++++++

