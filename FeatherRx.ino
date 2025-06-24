#include <SPI.h>
#include <RH_RF95.h>
#include <HX711.h>

// for Feather M0 Lora:
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 434.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Singleton instance of the radio driver

// for HX711
#define DAT 13
#define CLK 12
HX711 scale;

// for pressure transducers
#define pt1 A0
#define pt2 A1
#define pt3 A2
#define pt4 A3
#define pt5 A4
#define pt6 A6
// #define pt1 5

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("Feather LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1)
      ;
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);

  // ADC Setup
  scale.begin(DAT, CLK);
  Serial.println("Remove all weight from the scale...");
  delay(2000);
  scale.tare();  // Reset scale to 0 with no load
  Serial.println("Scale tared.");
  scale.set_scale(2280.0);  // raw data / known weight
}
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
uint8_t weightaddress = 0x01;
float weight;
const uint8_t numSensors = 6;
const uint8_t ptPins[numSensors] = { pt1, pt2, pt3, pt4, pt5, pt6};
uint16_t rawpressures[numSensors];
uint8_t pressureaddress = 0x02;

void loop() {
  ReadPackSendWeight();
  ReadPackSendPressure();

  if (rf95.available()) {
    if (rf95.recv(buf, &len)) {
      //RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print((char*)buf);
      Serial.println(" RECEIVED");
      command_decoder();
      sendACK();
    }
  }
}

// ACK can be a colored light(s) or something if we can't use serial because were printing data.
void sendACK() {
  Serial.println("Sending ACK");
  uint8_t radioack[17] = "Message Received";
  radioack[16] = '\0';
  rf95.send(radioack, sizeof(radioack));
}

void command_decoder() {
  if (len != 16) {  // checking for size of message (should be 16 bytes)
    Serial.println("Wrong Message Length");
    uint8_t radioerror[21] = "WRONG RECIPIENT-JCJL";
    radioerror[20] = '\0';
    rf95.send(radioerror, sizeof(radioerror));
  }

  // housekeeping for our received message (for commands)
  if (len == 16) {           // checking for size of message (should be 16 bytes)
    char main[len + 1];      // size of "radiopacket" full message (from tx)
    memcpy(main, buf, len);  // copying all of (char*)buf into an array we can manipulate
    main[len] = '\0';        // knowing size of our message is always 16, the 15th (last) index is manually set to null
    Serial.println(strlen(main));
    Serial.println(main);

    char password[13];            // size of password HERCULESJCJL +null
    strncpy(password, main, 12);  // copying only the first 12 chars into password
    password[12] = '\0';
    Serial.println(password);

    char launchcode[3];                               // size of launchcode [XX] +null
    strncpy(launchcode, main + strlen(main) - 2, 2);  // copies the last 2 chars from main
    launchcode[2] = '\0';
    Serial.println(launchcode);
    int test = sizeof(launchcode);
    Serial.println(test);

    // functions for if the message is what we're expecting
    if (strcmp(password, "HERCULESJCJL") == 0) {  // addressing
      Serial.println("Good Password");

      if (strcmp(launchcode, "01") == 0) {  // launch code 01
        Serial.println("launch code 01 good");
        servocommand1();
      }
      if (strcmp(launchcode, "02") == 0) {  // launch code 01
        Serial.println("launch code 02 good");
        servocommand2();
      }
    } else {
      Serial.println("Bad Password or Launch Code");
    }
  }
}

// Reads, packs, and sends weight data
void ReadPackSendWeight() {
  if (scale.is_ready()) {
    weight = scale.get_units(10);  // Get weight, average of 10 readings
    Serial.print("Weight: ");
    Serial.print(weight, 1);
    Serial.println(" g");
  } else {
    Serial.println("HX711 not ready");
  }
  delay(500);

  uint8_t weightbuffer[6];  //address + 4 bytes (float) + crc8 (null implied / not needed for binary data)
  memcpy(weightbuffer + 1, &weight, 4);
  weightbuffer[0] = weightaddress;
  uint8_t weightcrc = computeCRC8(weightbuffer, 5);  // calculating crc over the first 5 bytes only
  weightbuffer[5] = weightcrc;
  rf95.send(weightbuffer, sizeof(weightbuffer));
  rf95.waitPacketSent();
  delay(200);
}

// Pressure related functions

// Reads the analogpins, packs all the pressure data into 1 buffer, and sends the whole message where it will decoded on the other side.
void ReadPackSendPressure() {
  for (uint8_t i = 0; i < numSensors; i++) {
    rawpressures[i] = (uint16_t)analogRead(ptPins[i]);
  }

  uint8_t pressurebuffer[14];  // address + 6x2bytes (uint16_t) + CRC
  pressurebuffer[0] = pressureaddress;

  for (uint8_t i = 0; i < numSensors; i++) {
    memcpy(pressurebuffer + 1 + i * sizeof(uint16_t), &rawpressures[i], sizeof(uint16_t));
  }
  uint8_t pressurecrc = computeCRC8(pressurebuffer, 13);
  pressurebuffer[13] = pressurecrc;
  rf95.send(pressurebuffer, sizeof(pressurebuffer));
  rf95.waitPacketSent();
  delay(200);
}

void servocommand1() {
  // SERVO COMMANDS HERE
  Serial.println("Sending ACK");
  uint8_t radioack[21] = "SERVOCOMMAND 01 GOOD";
  radioack[20] = '\0';
  rf95.send(radioack, sizeof(radioack));
}

void servocommand2() {
  // SERVO COMMANDS HERE
  Serial.println("Sending ACK");
  uint8_t radioack[21] = "SERVOCOMMAND 02 GOOD";
  radioack[20] = '\0';
  rf95.send(radioack, sizeof(radioack));
}

uint8_t computeCRC8(const uint8_t* data, size_t datalen) {
  uint8_t crc = 0;
  for (size_t i = 0; i < datalen; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;  // Polynomial
      else
        crc <<= 1;
    }
  }
  return crc;
}