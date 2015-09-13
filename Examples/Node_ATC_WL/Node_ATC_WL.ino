// Sample RFM69 sender/node sketch, with ACK and optional encryption
// Sends periodic messages of increasing length to gateway (id=1)
// It also looks for an onboard FLASH chip, if present.
// Updated to use the RFM69_ATC derived class to demonstrate how to use the extension.
// This code is based on:
// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/
//==============================================================================================================
//==============================================================================================================
#include <RFM69_ATC.h>  //<=== NEW: Need to reference derived class library
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <LowPower.h> // to get sleep code

#define NODEID        2    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
//==============================================================================================================
//==============================================================================================================
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W! <=== MOST useful if you have RFM69HW
#define ACK_TIME      30 // max # of ms to wait for an ack

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define SERIAL_BAUD   115200

char payload[] = "123 ABCDEFGHIJKLMNOPQRSTUVWXYZ";
char buff[20];
byte listenBuffer[64];

byte sendSize=0;
boolean requestACK = false;
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

//==============================================================================================================
//==============================================================================================================
RFM69_ATC radio;   //<=== NOTE New class used instead of RFM69.

void setup() 
{
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
  radio.setPowerLevel(51);     // start with power at MAX
#endif
  radio.enableAutoPower(-69);  //<=== NEW method! Enables auto transmit power control with a targetRSSI of -69dB
  //==============================================================================================================
  // If the far end node also uses this library, it will automatically Ack with this node's received RSSI and
  // this node will automatically adjust its transmit power to provide this target level.
  //==============================================================================================================
  // THAT'S IT!  Nothing else needs to be done!
  //==============================================================================================================
  
  
  radio.encrypt(ENCRYPTKEY);
  //radio.setFrequency(919000000); //set frequency to some custom frequency
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  
  if (flash.initialize())
  {
    Serial.print("SPI Flash Init OK ... UniqueID (MAC): ");
    flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      Serial.print(flash.UNIQUEID[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
  else
    Serial.println("SPI Flash Init FAIL! (is chip present?)");
}

void loop() 
{
  // first thing we do is send a packet to the gateway...
    
  //send FLASH id every so often
  if(sendSize==0)
  {
    sprintf(buff, "FLASH_MEM_ID:0x%X", flash.readDeviceId());
    byte buffLen=strlen(buff);
    if (radio.sendWithRetry(GATEWAYID, buff, buffLen))
      Serial.print(" ok!");
    else Serial.print(" nothing...");
  }
  else
  {
    Serial.print("Sending[");
    Serial.print(sendSize);
    Serial.print("]: ");
    for(byte i = 0; i < sendSize; i++)
      Serial.print((char)payload[i]);

    if (radio.sendWithRetry(GATEWAYID, payload, sendSize))
     Serial.print(" ok!");
    else Serial.print(" nothing...");
  }
  sendSize = (sendSize + 1) % 31;
  Serial.println();
  
  // now we sleep until we get something from the gateway
  uint8_t len = sleepUntilPacket();
  
  //check for any received packets
  if (len)
  {
    for (byte i = 0; i < len; i++)
      Serial.print((char)listenBuffer[i]);
    Blink(LED,3);

    Serial.println();
  }

}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

uint8_t sleepUntilPacket(void)
{
  uint8_t len = 0;
  
  radio.clearListenBuffer();              // we can clear the buffer now since we HAVE to be done with it!
  radio.startListening(listenBuffer,64);  // put radio into listen mode

  Serial.flush();                           // flush any residual stuff in Serial port
 
  // DON'T PUT RADIO TO SLEEP!  IT'S IN LISTENING LOW POWER MODE!
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);     //  go to sleep until we get woken up by a burst from Gateway
  
  // WOKEN UP!
  len = radio.listenReceivedBytes(); // see what we have (if anything)
 
  // we've woken up!  Must mean we've got a command to wake up and resume!
  radio.endListening();

  radio.restoreInit(FREQUENCY,NODEID,NETWORKID);  // restore radio to normal settings
  radio.sleep();  // put radio to sleep while we wait for our time slot

#define BURST_TIME    3500ul   // 3 seconds plus 1/2 second buffer
#define MY_TIME_SLOT  (NODEID * 500ul)  // this gives 1/2 second response window to each node ID

  // time slot is (BURST_TIME - timeOffset) + MY_TIME_SLOT
  uint32_t 
    timeOffset,
    myTimeSlot;
    
    timeOffset = radio.listenReceivedOffset();  
    myTimeSlot = (BURST_TIME - timeOffset) + MY_TIME_SLOT;
    
  while (myTimeSlot >= 8192)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  // this really IS 8192mS DAMHIKT!
    myTimeSlot -= 8192;
  }
  
  return len;
}