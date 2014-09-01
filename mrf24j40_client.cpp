#include <Arduino.h>
#include <mrf24j40_client.h>

MRFClient::MRFClient(int pin_cs, int pin_int)
  : MRFDriver(pin_cs, pin_int)
{
  //nothing here for now
}

void MRFClient::init(void){
  write_short(MRF_GPIO, 0x00);
  write_short(MRF_TRISGPIO, 0x00);
  write_long(MRF_TESTMODE, 0x0F);
  
  write_short(MRF_PACON0, 0x29);
  write_short(MRF_PACON1, 0x02);
  write_short(MRF_PACON2, 0x98); // Initialize FIFOEN = 1 and TXONTS = 0x6.
  write_short(MRF_TXSTBL, 0x95); // Initialize RFSTBL = 0x9.

  write_long(MRF_RFCON0, 0x03); // Initialize RFOPT = 0x03.
  write_long(MRF_RFCON1, 0x02); // Initialize VCOOPT = 0x02.
  write_long(MRF_RFCON2, 0x80); // Enable PLL (PLLEN = 1).
  write_long(MRF_RFCON6, 0x90); // Initialize TXFIL = 1 and 20MRECVR = 1.
  write_long(MRF_RFCON7, 0x80); // Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
  write_long(MRF_RFCON8, 0x10); // Initialize RFVCO = 1.
  write_long(MRF_SLPCON1, 0x21); // Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.

  // Configuration for nonbeacon-enabled devices (see Section 3.8 Beacon-Enabled and
  // Nonbeacon-Enabled Networks)
  write_short(MRF_TXMCR, 0x1C);  //Clear Slotted mode
  write_short(MRF_RXMCR, 0x00);

  //Security
  set_AES_key(); //install the tx and rx security key
  //write_short(MRF_SECCON0,0x12); //enable AES-CCM-128 on the TXFIFO and RXFIFO
  //write_short(MRF_SECCON0,0x09); //AES-CTR
  write_short(MRF_SECCON0,0); //NONE

  write_short(MRF_BBREG2, 0x80); // Set CCA mode to ED
  write_short(MRF_CCAEDTH, 0x60); // Set CCA ED threshold.
  write_short(MRF_BBREG6, 0x40); // Set appended RSSI value to RXFIFO.

  // Initialize interrupts
  write_long(MRF_SLPCON0, 0x01); //Interrupt on falling edge and disable sleep clock
  write_short(MRF_INTCON, 0xE6); //Enable SEC, RX, and TX Interrupts

  set_channel(7);
  write_long(MRF_RFCON3, 0x40); //Select TX Power
  write_short(MRF_RFCTL, 0x04); //Reset RF state machine.
  write_short(MRF_RFCTL, 0x00);

  delay(1); //delay at least 192usec

  writeAddress();

  init_client();
}

void MRFClient::init_client(void) {
  zeroAddress(parent.addr);
  zeroPAN(parent.pan);
  local.pan[0]=0xFF;
  local.pan[1]=0xFF;
  writePAN(local.pan);
  local.addr16[0]=0xFF;
  local.addr16[1]=0xFF;
  write_addr16(local.addr16);
  _hops = 0xFF;
  _join_stat = 1;
  
  _rx_count = 0;
  cmd_count = 0;
}

void MRFClient::client_loop(void) {
  uint32_t current_time = millis();

  switch(_join_stat) {
    case 1:  //transmitting beacon requests every 10 seconds
      if(current_time - local.time > 10000) {
        //Serial.println("beacon req");
        tx_beacon_req();
        local.time = current_time;
      }
      break;
    case 2: //received a beacon, tx assoc req
      tx_assoc_req();
      _join_stat = 3;
    case 3: //waiting for assoc resp
      if(current_time - local.time > 20000) {
        init_client();
      }
      break;
    case 4: //haven't seen PC in awhile, request beacon
	  //if((current_time - parent.time) > 40000) {
	  //  _join_stat = 5;
	  //  tx_beacon_req();
      //}
	  break;
    case 5: //haven't heard back from PC, re-init
      if((current_time - parent.time) > 50000) {
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
  int ptr;
  int i;

  ptr=0;
  write_long(ptr++,23); //header length
  write_long(ptr++,25); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b00101011);
  write_long(ptr++,0b00100011);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(ptr++,0b11001100);

  write_long(ptr++,_seq_num); //sequence number

  //write panid
  for(i=0;i<2;i++) {
    write_long(ptr++,parent.pan[i]);
  }

  //write dest addr
  for(i=0;i<8;i++) {
	  write_long(ptr++,parent.addr[i]);
  }
  
  write_long(ptr++,0xFF); //src PANID low (broadcast)
  write_long(ptr++,0xFF); //src PANID high (broadcast)

  //write src addr
  for(i=0;i<8;i++) {
    write_long(ptr++,local.addr[i]);
  }
  write_long(ptr++,0x01); //MAC Command Frame (association request)

  // alloc addr | sec cap | 00 | rx idle | pwr src | dev type | PC
  write_long(ptr++,0b00000000); //ignored

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
    write_long(i++,local.pan[j]);
  }

  //write dest addr
  for(j=0;j<8;j++){
	  write_long(i++,dest_addr[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,local.addr[j]);
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
    write_long(i++,0b01100001);
  } else {
    write_long(i++,0b01100001);
  }
  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,local.pan[j]);
  }

  //write dest addr
 for(j=0;j<8;j++) {
    write_long(i++,parent.addr[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,local.addr[j]);
  }
  //end of header (21 bytes)

  write_long(i++,0x09); //mrf to udp command

  for(j=0;j<len;j++) {
    write_long(i++,tx_cmd_buffer[j]);
  }

  tx_ready();

  //Serial.println("TX DATA QUEUE");
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

  //update parent timeout
  if(compareAddress(packet.src_addr,parent.addr)) {
    parent.time = millis();
	parent.rssi = packet.rssi;
	parent.lqi = packet.lqi;
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
	  setAddress(parent.addr,packet.src_addr);
	  setPAN(parent.pan,packet.src_pan);
	  _join_stat = 2;
    }
  }

  parent.time = millis();
}

void MRFClient::rx_mac(void) {
  switch(packet.data[0]) {
    case 0x01:  //assoc req
      Serial.println("ERR: RX ASSOC REQ");
      break;
    case 0x02:  //assoc resp
      //Serial.println("RX ASSOC RESP");
      local.addr16[0] = packet.data[1];
      local.addr16[1] = packet.data[2];
	  if(compareAddress(packet.src_addr, parent.addr) && comparePAN(packet.src_pan, parent.pan)) {
        write_addr16(local.addr16);
		setPAN(local.pan, packet.src_pan);
        writePAN(local.pan);
        _join_stat = 4;
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

  switch(packet.data[0]) {
    case 0x06:  //rx heartbeat request
      Serial.print("heartbeat REQ: ");
      printAddress(packet.src_addr);
      tx_data_cmd(packet.src_addr, 0x05);
      break;
   case 0x10:  //rx rims command
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