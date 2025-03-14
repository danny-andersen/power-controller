#include <multi_channel_relay.h>
#include <Wire.h>

  /** 
   *  channle: 4 3 2 1
   *  state: 0b0000 -> 0x00  (all off)
   *  state: 0b1111 -> 0x0f   (all on)
  */ 

Multi_Channel_Relay relay;

void setup()
{
  DEBUG_PRINT.begin(9600);  
  while(!DEBUG_PRINT); 
  // Serial.println("Initialising Wire library for I2C");
  // Wire.begin(2,0);
  
  //Scan I2c
  // uint8_t old_address =                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             relay.scanI2CDevice();
  // Serial.print("Found Relay device at address: ");  // Serial.println("Initialising Wire library for I2C");
  // Wire.begin(2,0);

  // Serial.println(old_address);
  // Set Relay I2C address and start relay
  Serial.println("Initialising Relay library");
  relay.begin(0x11); 

/* Read firmware  version */
  Serial.print("Relay firmware version: ");
  Serial.print("0x");
  Serial.print(relay.getFirmwareVersion(), HEX);
  Serial.println();

}

void loop()
{
  /* Begin Controlling Relay */ 
  DEBUG_PRINT.println("Channel 1 on");
  relay.turn_on_channel(1);  
  delay(500);
  DEBUG_PRINT.println("Channel 2 on");
  relay.turn_off_channel(1);
  relay.turn_on_channel(2);
  delay(500);
  DEBUG_PRINT.println("Channel 3 on");
  relay.turn_off_channel(2);
  relay.turn_on_channel(3);  
  delay(500);
  DEBUG_PRINT.println("Channel 4 on");
  relay.turn_off_channel(3);
  relay.turn_on_channel(4);  
  delay(500);
  relay.turn_off_channel(4);

  relay.channelCtrl(CHANNLE1_BIT | 
                    CHANNLE2_BIT | 
                    CHANNLE3_BIT | 
                    CHANNLE4_BIT);
  DEBUG_PRINT.print("Turn all channels on, State: ");
  DEBUG_PRINT.println(relay.getChannelState(), BIN);
  
  delay(2000);

  relay.channelCtrl(CHANNLE1_BIT |                   
                    CHANNLE3_BIT);
  DEBUG_PRINT.print("Turn 1 3 channels on, State: ");
  DEBUG_PRINT.println(relay.getChannelState(), BIN);

  delay(2000);

  relay.channelCtrl(CHANNLE2_BIT | 
                    CHANNLE4_BIT);
  DEBUG_PRINT.print("Turn 2 4 channels on, State: ");
  DEBUG_PRINT.println(relay.getChannelState(), BIN);
  
  delay(2000);


  relay.channelCtrl(0);
  DEBUG_PRINT.print("Turn off all channels, State: ");
  DEBUG_PRINT.println(relay.getChannelState(), BIN);
  
  delay(2000);
  
}
