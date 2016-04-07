#include <RFM69_WL.h>
#include <LowPower.h>

// Uncomment the appropriate frequency for your hardware
#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ

#define ENCRYPT_KEY "sampleEncryptKey"
#define ADDRESS 2
#define NETWORK_ID 1

RFM69_WL radio;

void setup() {
  Serial.begin(9600);
  Serial.println("Listen Mode Example");
  Serial.println("Press 'l' to enter listen mode");

  radio.initialize(RF69_433MHZ, ADDRESS, NETWORK_ID);
  radio.encrypt(ENCRYPT_KEY);
}

void loop() {
  if (radio.receiveDone()) {
    if (radio.ACKRequested()) {
      radio.sendACK();
    }
    Serial.println("Received a normal message");
    Serial.println((char*)radio.DATA);
    Serial.flush();
  }

  if (Serial.read() == 'l') {
    Serial.println("Entering low-power listen mode...");
    Serial.flush();

    radio.startListening();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // Woke up, check for a message
    Serial.println("Woke up!");
    uint8_t from = 0;
    long burstRemaining = 0;
    if (radio.DATALEN > 0) {
      Serial.println("Received a message in listen mode");
      Serial.println((char*)radio.DATA);
      Serial.flush();
      from = radio.SENDERID;
      burstRemaining = radio.LISTEN_BURST_REMAINING_MS;
    }

    // Radio goes back to standby, ready for normal operations
    radio.endListening();

    if (from) {
      while (burstRemaining > 0) {
        LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
        burstRemaining -= 60;
      }
      radio.send(from, "woke", 4);
    }
  }
}
