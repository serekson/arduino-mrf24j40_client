#ifndef _MRF24J40_CLIENT_H_INCLUDED
#define _MRF24J40_CLIENT_H_INCLUDED

#include <Arduino.h>
#include <mrf24j40_driver.h>


class MRFClient : public MRFDriver {
  private:

  public:
  uint8_t _join_stat;
  uint16_t _panid;
  DeviceAddress _parent_addr;
  uint32_t _last_time;
  uint32_t _beacon_timer;
  
  uint8_t _hops;
  
  byte tx_cmd_buffer[64];
  
  MRFClient(int pin_cs, int pin_int);
  
  void init_client(void);
  
  void tx_beacon_req(void);
  void tx_assoc_req(void);
  void tx_data_cmd(DeviceAddress dest_addr, byte data_cmd);
  void tx_data_queue(uint8_t len) ;
  
  void client_loop(void);
  
  void rx_packet(void);
  void rx_beacon(void);
  void rx_mac(void);
  void rx_data(void);
  
  int cmd_count;
  byte cmd_buffer[64];
};
#endif