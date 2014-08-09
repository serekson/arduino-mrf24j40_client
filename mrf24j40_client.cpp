#include <Arduino.h>
#include <mrf24j40_client.h>

MRFClient::MRFClient(int pin_cs, int pin_int)
  : MRFDriver(pin_cs, pin_int)
{
  //nothing here for now
}

void MRFClient::init_client(void) {
  init();
  write_short(MRF_RXMCR, 0x00);
  
  memset(_parent_addr, 0, 8);
  write_pan(0xFFFF);
  write_addr16(0xFFFF);
  _hops = 0xFF;
  _join_stat = 1;
}

void MRFClient::tx_beacon_req(void) {
  int i;
  i=0;

  write_long(i++,7); //header length
  write_long(i++,8); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b01001011);
  write_long(i++,0b01000011);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b00001000);

  write_long(i++,_seq_num); //sequence number
  write_long(i++,0xFF); //broadcast PANID low
  write_long(i++,0xFF); //broadcast PANID high
  write_long(i++,0xFF); //broadcast address low
  write_long(i++,0xFF); //broadcast address high
  write_long(i++,0x07); //MAC Command Frame (beacon request)

  tx_ready();

  //Serial.println("TX BEACON REQ");
}

void MRFClient::tx_assoc_req(void) {
  int i;
  int j;

  i=0;
  write_long(i++,23); //header length
  write_long(i++,25); //frame lengt

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b00101011);
  write_long(i++,0b00100011);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,mrf_panid[j]);
  }

  //write dest addr
  for(j=0;j<8;j++) {
	  write_long(i++,_parent_addr[j]);
  }

  write_long(i++,0xFF); //src PANID low (broadcast)
  write_long(i++,0xFF); //src PANID high (broadcast)

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }
  write_long(i++,0x01); //MAC Command Frame (association request)

  // alloc addr | sec cap | 00 | rx idle | pwr src | dev type | PC
  write_long(i++,0b00000000); //ignored

  tx_ready();

  //Serial.println("TX ASSOC REQ");
}

void MRFClient::tx_data_cmd(DeviceAddress dest_addr, byte data_cmd) {
  int i;
  int j;

  i=0;
  write_long(i++,21); //header length
  write_long(i++,22); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b01101001);
  write_long(i++,0b01100001);
  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,mrf_panid[j]);
  }

  //write dest addr
  for(j=0;j<8;j++){
	  write_long(i++,dest_addr[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }
  //end of header (21 bytes)

  write_long(i++,data_cmd); //cmd id

  tx_ready();

  //Serial.print("TX DATA CMD: ");
  //Serial.println(data_cmd, HEX);
}

void MRFClient::tx_data_queue(uint8_t len) {
  int i;
  int j;

  i=0;
  write_long(i++,21); //header length
  write_long(i++,22+len); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  if(tx_cmd_buffer[0] == 102) {
    write_long(i++,0b01000001);
  } else {
    write_long(i++,0b01100001);
  }
  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,mrf_panid[j]);
  }

  //write dest addr
 for(j=0;j<8;j++) {
    write_long(i++,_parent_addr[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }
  //end of header (21 bytes)

  write_long(i++,0x09); //mrf to udp command

  for(j=0;j<len;j++) {
    write_long(i++,tx_cmd_buffer[j]);
  }

  tx_ready();

  //Serial.println("TX DATA QUEUE");
}

void MRFClient::client_loop(void) {
  uint32_t current_time = millis();

  switch(_join_stat) {
    case 1:  //transmitting beacon requests every 10 seconds
      if(current_time - _last_time > 10000) {
        //Serial.println("beacon req");
        tx_beacon_req();
        _last_time = current_time;
      }
      break;
    case 2: //received a beacon, tx assoc req
      tx_assoc_req();
      _join_stat = 3;
    case 3: //waiting for assoc resp
      if(current_time - _last_time > 20000) {
        init_client();
      }
      break;
    case 4: //node is active tx heartbeat
      if(current_time - _beacon_timer > 95000) {
        tx_data_cmd(_parent_addr, 0x06);
		_join_stat = 5;
      }
      break;
    case 5: //waiting for heartbeat resp
      if(current_time - _beacon_timer > 105000) {
        Serial.println("lost PC");
        init_client();
      }
      break;
    default:
      init_client();
  }
  
  if(int_mrf) {
    proc_interrupt();
  }
  
  if(_rx_count>0) {
    rx_packet();
    _rx_count--;
  }
}

void MRFClient::rx_packet(void) {
  switch(packet.frm_ctrl1 & 0x07) {
    case 0x00:
	  //Serial.println("rx beacon");
      rx_beacon();
      break;
    case 0x01:
	  //Serial.println("rx data");
      rx_data();
      break;
    case 0x02:
      //Serial.println("ack frame");
	  break;
    case 0x03:
	  //Serial.println("rx mac");
      rx_mac();
      break;
    default:
      Serial.print("invalid frame type: ");
      Serial.println(packet.frm_ctrl1, HEX);
  }

}

void MRFClient::rx_beacon(void) {
  uint8_t superframe1;
  uint8_t superframe2;
  uint8_t hops;
  uint8_t i;

  i=0;

  superframe1 = packet.data[i++]; //ignore this byte
  superframe2 = packet.data[i++];
  hops = packet.data[i++];
  
  if(_join_stat == 1) {
    if(superframe2 & 0x80) {
      memcpy(_parent_addr,packet.src_addr,8);
      _panid = packet.src_pan;
	  _join_stat = 2;
    }
  }

  _beacon_timer = millis();
}

void MRFClient::rx_mac(void) {
  uint16_t addr;

  switch(packet.data[0]) {
    case 0x01:  //assoc req
      //Serial.println("ERR: RX ASSOC REQ");
      break;
    case 0x02:  //assoc resp
      //Serial.println("RX ASSOC RESP");
      addr = packet.data[1];
      addr |= packet.data[2]<<8;
      if(!memcmp(packet.src_addr, _parent_addr, 8) && (packet.src_pan == _panid)) {
        write_addr16(addr);
        write_pan(packet.src_pan);
        _join_stat = 4;
        //Serial.print("ADDR SET TO: ");
        //Serial.println(addr, HEX);
        //Serial.print("PANID SET TO: ");
        //Serial.println(src_pan, HEX);
      }
      break;
    //case 0x03:  //dissoc not
    //case 0x04:  //data req
    //case 0x05:  //PAN ID Conflict
    //case 0x06:  //orphan not
    case 0x07:  //beacon req
      Serial.println("ERR: RX BEACON REQ");
      break;
    //case 0x08:  //coord realign
    //case 0x09:  //GTS req
    default:
      Serial.print("MAC COMMAND ");
      Serial.print(packet.data[0], HEX);
      Serial.println(" NOT PROGRAMMED");
  }
}

void MRFClient::rx_data(void) {
  uint8_t i;
  uint8_t j;

  switch(packet.data[0]) {
    case 0x05:  //heartbeat
      Serial.print("heartbeat: ");
      printAddress(packet.src_addr);
      if(!memcmp(packet.src_addr, _parent_addr, 8)) {
        //Serial.println("valid");
        _join_stat = 4;
      } else {
        Serial.println("not valid");
      }
      break;
    case 0x06:  //heartbeat request
      Serial.print("heartbeat REQ: ");
      printAddress(packet.src_addr);
      tx_data_cmd(packet.src_addr, 0x05);
      break;
   case 0x10:  //rims command
      Serial.println("rims command");
      for(i=1;i<packet.data_len;i++) {
		cmd_buffer[i-1] = packet.data[i];
      }
	  cmd_count++;
      break;
    default:
      Serial.print("Data Command ");
      Serial.print(packet.data[0], HEX);
      Serial.println(" is not valid");
  }
}