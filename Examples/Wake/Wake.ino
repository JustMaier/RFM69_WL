#include <RFM69_WL.h>

// Uncomment the appropriate frequency for your hardware
#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ

#define ENCRYPT_KEY "sampleEncryptKey"
#define ADDRESS 1
#define NETWORK_ID 1
#define REMOTE_ADDRESS 2

RFM69_WL radio;

void setup() {
  Serial.begin(9600);
  Serial.println("Wake Example");
  Serial.println("Press 'b' to send a burst, 'm' for a normal message ");

  radio.initialize(RF69_433MHZ, ADDRESS, NETWORK_ID);
  radio.encrypt(ENCRYPT_KEY);
}

char payload[] = "0123456789";

void loop() {
  if (!Serial.available()) {
    return;
  }

  byte cmd = Serial.read(); 
  if (cmd == 'b') {
    Serial.println("Sending wakeup burst...");
    radio.sendBurst(REMOTE_ADDRESS, payload, sizeof(payload));
    Serial.println("Done");
  }

  if (cmd == 'm') {
    Serial.println("Sending normal message...");
    radio.send(REMOTE_ADDRESS, payload, sizeof(payload));
    Serial.println("Done");
  }
}
