#include <RFM69_WL.h>

// Uncomment the appropriate frequency for your hardware
#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ

#define ENCRYPT_KEY "sampleEncryptKey"
#define ADDRESS 1
#define NETWORK_ID 1
#define REMOTE_ADDRESS 2
#define BURST_REPLY_TIMEOUT_MS 250
#define PAYLOAD_SIZE 12

RFM69_WL radio;

void setup() {
  Serial.begin(9600);
  Serial.println("Wake Example");
  Serial.println("Press 'b' to send a burst, 'm' for a normal message ");

  radio.initialize(RF69_433MHZ, ADDRESS, NETWORK_ID);
  radio.encrypt(ENCRYPT_KEY);
}

char* buildPayload(void)
{
  static char payload[PAYLOAD_SIZE];
  static uint8_t num;

  snprintf(payload, PAYLOAD_SIZE, "Message %u", ++num);
  return payload;
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  byte cmd = Serial.read();
  if (cmd == 'b') {
    Serial.print("Sending wakeup burst...");
    radio.sendBurst(REMOTE_ADDRESS, buildPayload(), PAYLOAD_SIZE);

    bool replied = false;
    long start = millis();
    while (millis() - start < BURST_REPLY_TIMEOUT_MS) {
      if (radio.receiveDone()) {
        Serial.println("Success");
        replied = true;
        break;
      }
    }

    if (!replied) {
      Serial.println("Failed");
    }
  }

  if (cmd == 'm') {
    Serial.print("Sending normal message...");
    if (radio.sendWithRetry(REMOTE_ADDRESS, buildPayload(), PAYLOAD_SIZE)) {
      Serial.println("Success");
    } else {
      Serial.println("Failed");
    }
  }
}