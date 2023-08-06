#include "esphome.h"

static const char *const TAG = "Webasto";

class Webasto : public Component, public UARTDevice {

    ESP32ArduinoUARTComponent  *_uart_comp;

    const uint8_t WBUS_CLIENT_ADDR = 0x0F; // address as client
    const uint8_t WBUS_HOST_ADDR   = 0x04; // address as host

    const uint8_t WBUS_CMD_OFF     = 0x10; /* no data */
    const uint8_t WBUS_CMD_ON_PH   = 0x21; /* Parking Heating*/
    const uint8_t WBUS_CMD_ON_VENT = 0x22; /* Ventilation */
    const uint8_t WBUS_CMD_CHK     = 0x44; /* Check current command (0x20,0x21,0x22 or 0x23) */

    uint8_t   message_data[10], message_length;

    const unsigned long send_break_periode = 30000;
    unsigned long     last_ok_rx           = millis() - send_break_periode;

    uint8_t         keep_alive_cmd  = 0;
    unsigned long keep_alive_time   = 0;

  public:
    Webasto(ESP32ArduinoUARTComponent *parent) : UARTDevice(parent) {
      _uart_comp = parent;
    }

    struct state_50_03_t {
      bool heat_request = false;
      bool vent_request = false;
      bool bit3 = false;
      bool bit4 = false;
      bool combustion_fan = false;
      bool glowplug = false;
      bool fuel_pump = false;
      bool nozzle_heating = false;
    } state_50_03;

    struct state_50_04_t {
      float glowplug = 0;
      float fuel_pump = 0;
      float combustion_fan = 0;
    } state_50_04;

    struct state_50_05_t {
      float temperature = 0;
      float voltage = 0;
      float glowplug_resistance = 0;
    } state_50_05;

    struct state_50_06_t {
      float working_hours = 0;
      float operating_hours = 0;
      uint16_t start_counter = 0;
    } state_50_06;

    struct state_50_07_t {
      uint8_t op_state = 0;
    } state_50_07;


    //char state_50_02s[40];
    //char state_50_03s[40];
    //char state_50_04s[40];
    //char state_50_05s[40];
    //char state_50_06s[40];
    //char state_50_07s[40];

    void itob(uint8_t x, char *buf)
    {
      unsigned char *ptr = (unsigned char *)&x;
      int pos = 0;
      for (int i = sizeof(uint8_t) - 1; i >= 0; i--)
        for (int j = CHAR_BIT - 1; j >= 0; j--)
          buf[pos++] = '0' + !!(ptr[i] & 1U << j);
      buf[pos] = '\0';
    }

    void SendBreak() {
      unsigned long now = millis();
      if (now - last_ok_rx >= send_break_periode) {
        ESP_LOGD(TAG, "SendBreak");
        
        // wait for empty tx buffer
        _uart_comp->flush();
        // empty rx buffer
        uint8_t waste;
        while (_uart_comp->available() > 0) _uart_comp->read_byte(&waste);

        // send magic byte with slow baudrate for 25ms LOW level
        HardwareSerial *_hw_serial = _uart_comp->get_hw_serial();
        _hw_serial->updateBaudRate(300);
        _uart_comp->write_byte(0b10000000);
        _uart_comp->flush();

        // empty RX system buffer, wait for 0b10000000 = wait 25ms with HIGH level
        unsigned long rxstr = millis();
        while ((millis() - rxstr) < 50) {
          if (_uart_comp->available()) {
            _uart_comp->read_byte(&waste);
            break;
          }
          else delay(1);
        }

        // restore baudrate
        _hw_serial->updateBaudRate(2400);
      } else {
        ESP_LOGD(TAG, "SendBreak not needed, last good rx before: %lu ms", now - last_ok_rx);
      }
    }


    uint8_t checksum(uint8_t *buf, uint8_t len)
    {
      uint8_t chk = 0;
      for ( ; len != 0; len--) chk ^= *buf++;
      return chk;
    }

    bool tx_msg2(uint8_t* dat, uint8_t len) {
      ESP_LOGD(TAG, "tx_msg start");
      uint8_t txbuf[40];
      txbuf[0] = ((WBUS_CLIENT_ADDR << 4) | WBUS_HOST_ADDR);
      txbuf[1] = len + 1;
      uint8_t txcnt = 2;
      for (uint8_t i = 0; i < len; ) {
        txbuf[txcnt++] = dat[i++];
      }
      // make checksum
      uint8_t chks = 0;
      for (uint8_t i = 0; i < txcnt; i++) chks ^= txbuf[i];
      txbuf[txcnt++] = chks;
      // Log
      char logbuf[130];
      logbuf[0] = '\0';
      for (uint8_t i = 0; i < txcnt; i++) {
        sprintf(logbuf, "%s %02X", logbuf, txbuf[i]);
      }
      ESP_LOGD(TAG, "TX: %s", logbuf);
      // empty RX system buffer
      uint8_t waste;
      while (_uart_comp->available() > 0) _uart_comp->read_byte(&waste);
      // Send message
      _uart_comp->write_array(&txbuf[0], txcnt);
      _uart_comp->flush();
      // Receive own message
      uint8_t rxbuf[40];
      uint8_t rxcnt = 0;
      unsigned long rxstr = millis();
      while (rxcnt < txcnt && (millis() - rxstr) < 100) {
        if (_uart_comp->available()) _uart_comp->read_byte(&rxbuf[rxcnt++]);
        else delay(1);
      }
      logbuf[0] = '\0';
      for (uint8_t i = 0; i < rxcnt; i++) {
        sprintf(logbuf, "%s %02X", logbuf, rxbuf[i]);
      }
      ESP_LOGD(TAG, "RX: %s", logbuf);
      // compare RX with TX
      bool ok = txcnt == rxcnt;
      for (uint8_t i = 0; ok && i < txcnt; i++) {
        ok = txbuf[i] == rxbuf[i];
      }
      if (ok) ESP_LOGD(TAG, "tx_msg done: ok");
      else    ESP_LOGD(TAG, "tx_msg done: error");
      return ok;
    }
 
    bool rx_msg2(uint8_t* dat, uint8_t len) {
      ESP_LOGD(TAG, "rx_msg start");
      // Receive message
      uint8_t rx_len = len + 3;
      uint8_t rxbuf[40];
      uint8_t rxcnt = 0;
      unsigned long rxstr = millis();
      unsigned long timeout = 200;
      long delta = 0;
      while (rxcnt < rx_len && delta < timeout) {
        if (_uart_comp->available()) _uart_comp->read_byte(&rxbuf[rxcnt++]);
        else delay(1);
        delta = millis() - rxstr;
      }
      // check timeout
      bool ok_time = delta < timeout;
      ESP_LOGD(TAG, "Time msg: %03ld, max: %03ld -> %s", delta, timeout, ok_time ? "ok" : "error");
      // check rx_len
      bool ok_rxcnt = rxcnt == rx_len;
      ESP_LOGD(TAG, "RXLEN msg: %02X, cal: %02X -> %s", rxcnt, rx_len, ok_rxcnt ? "ok" : "error");
      // log
      char logbuf[130];
      logbuf[0] = '\0';
      for (uint8_t i = 0; i < rxcnt; i++) {
        sprintf(logbuf, "%s %02X", logbuf, rxbuf[i]);
      }
      ESP_LOGD(TAG, "RX %u bytes: %s", rxcnt, logbuf);
      //ckeck address
      bool ok_addr = rxbuf[0] == ((WBUS_HOST_ADDR << 4) | WBUS_CLIENT_ADDR);
      ESP_LOGD(TAG, "ADDR msg: %02X, cal: %02X -> %s", rxbuf[0], ((WBUS_HOST_ADDR << 4) | WBUS_CLIENT_ADDR), ok_addr ? "ok" : "error");
      //ckeck length byte
      bool ok_len = rxbuf[1] == rxcnt - 2;
      ESP_LOGD(TAG, "LEN msg: %02X, cal: %02X -> %s", rxbuf[1], rxcnt - 2, ok_len ? "ok" : "error");
      // ckeck checksum
      uint8_t chks = 0;
      for (uint8_t i = 0; i < rxcnt - 1; i++) chks ^= rxbuf[i];
      bool ok_chks = rxbuf[rxcnt - 1] == chks;
      ESP_LOGD(TAG, "CHKS msg: %02X, cal: %02X -> %s", rxbuf[rxcnt - 1], chks, ok_chks ? "ok" : "error");

      bool ok = ok_time && ok_rxcnt && ok_addr && ok_len && ok_chks;
      if (ok) memcpy(&dat[0], &rxbuf[2], len);
      if (ok) last_ok_rx = millis();
      ESP_LOGD(TAG, "rx_msg done: %s", ok ? "ok" : "error");
      return ok;
    }

    void VentOn(uint8_t t_on_mins) {
      ESP_LOGD(TAG, "Send VentOn");
      uint8_t tx_dat[] = {WBUS_CMD_ON_VENT, t_on_mins};
      uint8_t rx_dat[sizeof(tx_dat)];
      for (uint8_t i = 0; i < 3; i++) { // tries
        SendBreak();
        if (!tx_msg2(tx_dat, sizeof(tx_dat))) {
          ESP_LOGE(TAG, "VentOn !tx_ok");
          continue;
        }
        if (!rx_msg2(rx_dat, sizeof(rx_dat))) {
          ESP_LOGE(TAG, "VentOn !rx_ok");
          continue;
        }
        //ESP_LOGD(TAG, "CMD tx: %02X, rx: %02X", tx_dat[0], rx_dat[0]);
        if ((tx_dat[0] | 0x80) != rx_dat[0]) {
          ESP_LOGE(TAG, "VentOn !cmd_ok");
          continue;
        }
        //ESP_LOGD(TAG, "SUBCMD tx: %02X, rx: %02X", tx_dat[1], rx_dat[1]);
        if (tx_dat[1] != rx_dat[1]) {
          ESP_LOGE(TAG, "VentOn !subcmd_ok");
          continue;
        }
        keep_alive_cmd  = WBUS_CMD_ON_VENT;
        keep_alive_time = (unsigned long)t_on_mins * 60 * 1000;
        break;
      }
    }

    void HeatOn(uint8_t t_on_mins) {
      ESP_LOGD(TAG, "Send HeatOn");
      uint8_t tx_dat[] = {WBUS_CMD_ON_PH, t_on_mins};
      uint8_t rx_dat[sizeof(tx_dat)];
      for (uint8_t i = 0; i < 3; i++) { // tries
        SendBreak();
        if (!tx_msg2(tx_dat, sizeof(tx_dat))) {
          ESP_LOGE(TAG, "HeatOn !tx_ok");
          continue;
        }
        if (!rx_msg2(rx_dat, sizeof(rx_dat))) {
          ESP_LOGE(TAG, "HeatOn !rx_ok");
          continue;
        }
        //ESP_LOGD(TAG, "CMD tx: %02X, rx: %02X", tx_dat[0], rx_dat[0]);
        if ((tx_dat[0] | 0x80) != rx_dat[0]) {
          ESP_LOGE(TAG, "HeatOn !cmd_ok");
          continue;
        }
        //ESP_LOGD(TAG, "SUBCMD tx: %02X, rx: %02X", tx_dat[1], rx_dat[1]);
        if (tx_dat[1] != rx_dat[1]) {
          ESP_LOGE(TAG, "HeatOn !subcmd_ok");
          continue;
        }
        keep_alive_cmd  = WBUS_CMD_ON_PH;
        keep_alive_time = (unsigned long)t_on_mins * 60 * 1000;
        break;
      }
    }

    void VentOn() {
      VentOn(1);
    }
    void HeatOn() {
      HeatOn(1);
    }

    void Off() {
      ESP_LOGD(TAG, "Send Off");
      uint8_t tx_dat[] = {WBUS_CMD_OFF};
      uint8_t rx_dat[sizeof(tx_dat)];
      for (uint8_t i = 0; i < 3; i++) { // tries
        SendBreak();
        if (!tx_msg2(tx_dat, sizeof(tx_dat))) {
          ESP_LOGE(TAG, "Off !tx_ok");
          continue;
        }
        if (!rx_msg2(rx_dat, sizeof(rx_dat))) {
          ESP_LOGE(TAG, "Off !rx_ok");
          continue;
        }
        //ESP_LOGD(TAG, "CMD tx: %02X, rx: %02X", tx_dat[0], rx_dat[0]);
        if ((tx_dat[0] | 0x80) != rx_dat[0]) {
          ESP_LOGE(TAG, "Off !cmd_ok");
          continue;
        }
        keep_alive_cmd  = 0;
        keep_alive_time = 0;
        break;
      }
    }

    void KeepAlive() {
      const unsigned long periode = 10000;
      unsigned long now = millis();
      static unsigned long last = now - periode;
      if (now - last >= periode) {
        last += periode;
        if (keep_alive_cmd > 0 && keep_alive_time > 0) {
          ESP_LOGD(TAG, "Send KeepAlive");
          uint8_t tx_dat[] = {WBUS_CMD_CHK, keep_alive_cmd, 0};
          uint8_t rx_dat[2];
          for (uint8_t i = 0; i < 3; i++) { // tries
            SendBreak();
            if (!tx_msg2(tx_dat, sizeof(tx_dat))) {
              ESP_LOGE(TAG, "KeepAlive !tx_ok");
              continue;
            }
            if (!rx_msg2(rx_dat, sizeof(rx_dat))) {
              ESP_LOGE(TAG, "KeepAlive !rx_ok");
              continue;
            }
            //ESP_LOGD(TAG, "CMD tx: %02X, rx: %02X", tx_dat[0], rx_dat[0]);
            if ((tx_dat[0] | 0x80) != rx_dat[0]) {
              ESP_LOGE(TAG, "KeepAlive !cmd_ok");
              continue;
            }
            if (keep_alive_time > periode) keep_alive_time -= periode;
            else keep_alive_time = 0;
            break;
          }
        }
        if (keep_alive_cmd > 0 && keep_alive_time < 30 * 1000) {
          ESP_LOGD(TAG, "Send ReNew");
          if (keep_alive_cmd == WBUS_CMD_ON_VENT) VentOn(1);
          if (keep_alive_cmd == WBUS_CMD_ON_PH)   HeatOn(1);
        }
      }
    }

    void get_state_50_03() {
      const unsigned long periode = 5000;
      unsigned long now = millis();
      static unsigned long last = now - periode;
      if (now - last >= periode) {
        last += periode;
        ESP_LOGD(TAG, "get_state_50_03");
        /*
            0x01 Heat request
            0x02 Vent request
            0x04 ?
            0x08 ?
            0x10 Combustion Fan
            0x20 Glowplug
            0x40 Fuel Pump
            0x80 Nozzle heating
            ? Circulation Pump
        */
        uint8_t tx_dat[] = {0x50, 0x03};
        uint8_t rx_dat[sizeof(tx_dat) + 1];
        SendBreak();
        bool tx_ok = tx_msg2(tx_dat, sizeof(tx_dat));
        if (!tx_ok) {
          ESP_LOGE(TAG, "50_03 !tx_ok");
          return;
        }
        bool rx_ok = rx_msg2(rx_dat, sizeof(rx_dat));
        if (!rx_ok) {
          ESP_LOGE(TAG, "50_03 !rx_ok");
          return;
        }
        //ESP_LOGD(TAG, "CMD tx: %02X, rx: %02X", tx_dat[0], rx_dat[0]);
        if ((tx_dat[0] | 0x80) != rx_dat[0]) {
          ESP_LOGE(TAG, "50_03 !cmd_ok");
          return;
        };
        //ESP_LOGD(TAG, "SUBCMD tx: %02X, rx: %02X", tx_dat[1], rx_dat[1]);
        if (tx_dat[1] != rx_dat[1]) {
          ESP_LOGE(TAG, "50_03 !subcmd_ok");
          return;
        };

        //state_50_03s[0] = '\0'; for(uint8_t i=0; i<sizeof(rx_dat); i++) sprintf(state_50_03s, "%s %02X", state_50_03s, rx_dat[i]);

        state_50_03.heat_request     = rx_dat[2] & 0x01;
        state_50_03.vent_request     = rx_dat[2] & 0x02;
        state_50_03.bit3             = rx_dat[2] & 0x04;
        state_50_03.bit4             = rx_dat[2] & 0x08;
        state_50_03.combustion_fan   = rx_dat[2] & 0x10;
        state_50_03.glowplug         = rx_dat[2] & 0x20;
        state_50_03.fuel_pump        = rx_dat[2] & 0x40;
        state_50_03.nozzle_heating   = rx_dat[2] & 0x80;

        ESP_LOGD(TAG, "Heat Request:     %s", state_50_03.heat_request     ? "on" : "off");
        ESP_LOGD(TAG, "Vent Request:     %s", state_50_03.vent_request     ? "on" : "off");
        ESP_LOGD(TAG, "Bit3:             %s", state_50_03.bit3             ? "on" : "off");
        ESP_LOGD(TAG, "Bit4:             %s", state_50_03.bit4             ? "on" : "off");
        ESP_LOGD(TAG, "Combustion Fan:   %s", state_50_03.combustion_fan   ? "on" : "off");
        ESP_LOGD(TAG, "Glowplug:         %s", state_50_03.glowplug         ? "on" : "off");
        ESP_LOGD(TAG, "Fuel Pump:        %s", state_50_03.fuel_pump        ? "on" : "off");
        ESP_LOGD(TAG, "Nozzle Heating:   %s", state_50_03.nozzle_heating   ? "on" : "off");
      }
    }

    void get_state_50_04() {
      const unsigned long periode = 5000;
      unsigned long now = millis();
      static unsigned long last = now - periode;
      if (now - last >= periode) {
        last += periode;
        ESP_LOGD(TAG, "get_state_50_04");
        /*
            byte0: Unknown
            byte1: Unknown
            byte2: Unknown
            byte3: Unknown
            byte4: Glowplug %
            byte5: Fuel Pump Hz
            byte6: Combustion Fan %
            byte7: Unknown
        */
        uint8_t tx_dat[] = {0x50, 0x04};
        uint8_t rx_dat[sizeof(tx_dat) + 8];
        SendBreak();
        bool tx_ok = tx_msg2(tx_dat, sizeof(tx_dat));
        if (!tx_ok) {
          ESP_LOGE(TAG, "50_04 !tx_ok");
          return;
        }
        bool rx_ok = rx_msg2(rx_dat, sizeof(rx_dat));
        if (!rx_ok) {
          ESP_LOGE(TAG, "50_04 !rx_ok");
          return;
        }
        //ESP_LOGD(TAG, "CMD tx: %02X, rx: %02X", tx_dat[0], rx_dat[0]);
        if ((tx_dat[0] | 0x80) != rx_dat[0]) {
          ESP_LOGE(TAG, "50_04 !cmd_ok");
          return;
        };
        //ESP_LOGD(TAG, "SUBCMD tx: %02X, rx: %02X", tx_dat[1], rx_dat[1]);
        if (tx_dat[1] != rx_dat[1]) {
          ESP_LOGE(TAG, "50_04 !subcmd_ok");
          return;
        };

        //state_50_04s[0] = '\0'; for(uint8_t i=0; i<sizeof(rx_dat); i++) sprintf(state_50_04s, "%s %02X", state_50_04s, rx_dat[i]);

        state_50_04.glowplug       = (float)rx_dat[6]; // 0-100 %
        state_50_04.fuel_pump      = (float)rx_dat[7] * 2.0 / 100.0; // 0-5 Hz
        state_50_04.combustion_fan = (float)rx_dat[8]; // 0-200 %

        ESP_LOGD(TAG, "Glowplug:      %03.0f %"  , state_50_04.glowplug);
        ESP_LOGD(TAG, "Fuel Pump:     %04.2f Hz" , state_50_04.fuel_pump);
        ESP_LOGD(TAG, "Combustin Fan: %03.0f %"  , state_50_04.combustion_fan);
      }
    }

    void get_state_50_05() {
      const unsigned long periode = 5000;
      unsigned long now = millis();
      static unsigned long last = now - periode;
      if (now - last >= periode) {
        last += periode;
        ESP_LOGD(TAG, "get_state_50_05");
        /*
            byte0: Temperature
            byte1: Voltage
            byte2: Flame detector resistance
        */
        uint8_t tx_dat[] = {0x50, 0x05};
        uint8_t rx_dat[sizeof(tx_dat) + 3];
        SendBreak();
        bool tx_ok = tx_msg2(tx_dat, sizeof(tx_dat));
        if (!tx_ok) {
          ESP_LOGE(TAG, "50_05 !tx_ok");
          return;
        }
        bool rx_ok = rx_msg2(rx_dat, sizeof(rx_dat));
        if (!rx_ok) {
          ESP_LOGE(TAG, "50_05 !rx_ok");
          return;
        }
        //ESP_LOGD(TAG, "CMD tx: %02X, rx: %02X", tx_dat[0], rx_dat[0]);
        if ((tx_dat[0] | 0x80) != rx_dat[0]) {
          ESP_LOGE(TAG, "50_05 !cmd_ok");
          return;
        };
        //ESP_LOGD(TAG, "SUBCMD tx: %02X, rx: %02X", tx_dat[1], rx_dat[1]);
        if (tx_dat[1] != rx_dat[1]) {
          ESP_LOGE(TAG, "50_05 !subcmd_ok");
          return;
        };

        //state_50_05s[0] = '\0'; for(uint8_t i=0; i<sizeof(rx_dat); i++) sprintf(state_50_05s, "%s %02X", state_50_05s, rx_dat[i]);

        { const float x[] = {186, 71};
          const float y[] = { 18, 72};
          const float m = (y[1] - y[0]) / (x[1] - x[0]);
          const float n = y[0] - m * x[0];
          state_50_05.temperature         = m * (float)rx_dat[2] + n;
        }
        { const float x[] = { 195,  180};
          const float y[] = {13.4, 12.0};
          const float m = (y[1] - y[0]) / (x[1] - x[0]);
          const float n = y[0] - m * x[0];
          state_50_05.voltage             = m * (float)rx_dat[3] + n;
        }
        { const float x[] = { 51, 108};
          const float y[] = {0.8, 1.1};
          const float m = (y[1] - y[0]) / (x[1] - x[0]);
          const float n = y[0] - m * x[0];
          state_50_05.glowplug_resistance = rx_dat[4] > 0 ? m * (float)rx_dat[4] + n : 0;
        }

        ESP_LOGD(TAG, "Temperature:         %+05.1f C", state_50_05.temperature);
        ESP_LOGD(TAG, "Voltage:             %04.1f V" , state_50_05.voltage);
        ESP_LOGD(TAG, "Glowplug Resistance: %05.3f Ω" , state_50_05.glowplug_resistance);
      }
    }

    void get_state_50_06() {
      const unsigned long periode = 30000;
      unsigned long now = millis();
      static unsigned long last = now - periode;
      if (now - last >= periode) {
        last += periode;
        ESP_LOGD(TAG, "get_state_50_06");
        /*
              byte0,1: Working hours
              byte2:   Working minutes
              byte3,4: Operating hours
              byte5:   Operating minutes
              byte6,7: Start counter
        */
        uint8_t tx_dat[] = {0x50, 0x06};
        uint8_t rx_dat[sizeof(tx_dat) + 8];
        SendBreak();
        bool tx_ok = tx_msg2(tx_dat, sizeof(tx_dat));
        if (!tx_ok) {
          ESP_LOGE(TAG, "50_06 !tx_ok");
          return;
        }
        bool rx_ok = rx_msg2(rx_dat, sizeof(rx_dat));
        if (!rx_ok) {
          ESP_LOGE(TAG, "50_06 !rx_ok");
          return;
        }
        //ESP_LOGD(TAG, "CMD tx: %02X, rx: %02X", tx_dat[0], rx_dat[0]);
        if ((tx_dat[0] | 0x80) != rx_dat[0]) {
          ESP_LOGE(TAG, "50_06 !cmd_ok");
          return;
        };
        //ESP_LOGD(TAG, "SUBCMD tx: %02X, rx: %02X", tx_dat[1], rx_dat[1]);
        if (tx_dat[1] != rx_dat[1]) {
          ESP_LOGE(TAG, "50_06 !subcmd_ok");
          return;
        };

        //state_50_06s[0] = '\0'; for(uint8_t i=0; i<sizeof(rx_dat); i++) sprintf(state_50_06s, "%s %02X", state_50_06s, rx_dat[i]);

        state_50_06.working_hours   = 256 * (float)rx_dat[2] + (float)rx_dat[3] + (float)rx_dat[4] / 60;
        state_50_06.operating_hours = 256 * (float)rx_dat[5] + (float)rx_dat[6] + (float)rx_dat[7] / 60;
        state_50_06.start_counter   = 256 * (uint16_t)rx_dat[8] + (uint16_t)rx_dat[9];

        ESP_LOGD(TAG, "Working Hours:   %07.2f h", state_50_06.working_hours);
        ESP_LOGD(TAG, "Operating Hours: %07.2f h", state_50_06.operating_hours);
        ESP_LOGD(TAG, "Start Counter:   %04u"    , state_50_06.start_counter);
      }
    }

    void get_state_50_07() {
      const unsigned long periode = 5000;
      unsigned long now = millis();
      static unsigned long last = now - periode;
      if (now - last >= periode) {
        last += periode;
        ESP_LOGD(TAG, "get_state_50_07");
        /*
            byte1: Operating state
            byte2: Unknown
            byte3: Unknown
            byte4: Unknown
        */
        uint8_t tx_dat[] = {0x50, 0x07};
        uint8_t rx_dat[sizeof(tx_dat) + 4];
        SendBreak();
        bool tx_ok = tx_msg2(tx_dat, sizeof(tx_dat));
        if (!tx_ok) {
          ESP_LOGE(TAG, "50_07 !tx_ok");
          return;
        }
        bool rx_ok = rx_msg2(rx_dat, sizeof(rx_dat));
        if (!rx_ok) {
          ESP_LOGE(TAG, "50_07 !rx_ok");
          return;
        }
        //ESP_LOGD(TAG, "CMD tx: %02X, rx: %02X", tx_dat[0], rx_dat[0]);
        if ((tx_dat[0] | 0x80) != rx_dat[0]) {
          ESP_LOGE(TAG, "50_07 !cmd_ok");
          return;
        };
        //ESP_LOGD(TAG, "SUBCMD tx: %02X, rx: %02X", tx_dat[1], rx_dat[1]);
        if (tx_dat[1] != rx_dat[1]) {
          ESP_LOGE(TAG, "50_07 !subcmd_ok");
          return;
        };

        //state_50_07s[0] = '\0'; for(uint8_t i=0; i<sizeof(rx_dat); i++) sprintf(state_50_07s, "%s %02X", state_50_07s, rx_dat[i]);

        state_50_07.op_state = rx_dat[2];

        ESP_LOGD(TAG, "OP state: %02X" , state_50_07.op_state);
      }
    }



    void setup() override {}


    void loop() override {
      static uint8_t state = 0;

      switch (state++) {
        case 0: KeepAlive(); break;
        case 1: get_state_50_03(); break;
        case 2: get_state_50_04(); break;
        case 3: get_state_50_05(); break;
        case 4: get_state_50_06(); break;
        case 5: get_state_50_07(); break;
        default: state = 0; break;
      }

      char logbuf[130];
      logbuf[0] = '\0';
      while (_uart_comp->available()) {
        uint8_t rx;
        _uart_comp->read_byte(&rx);
        sprintf(logbuf, "%s %02X", logbuf, rx);
      }
      if (logbuf[0] != '\0') ESP_LOGD(TAG, "%010ld, RX: %s", millis(), logbuf);
    }

};
