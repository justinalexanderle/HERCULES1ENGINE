#include <SPI.h>
#include <RH_RF95.h>
#include <HX711.h>
#include <Adafruit_SleepyDog.h>

// macro for debugging, 0 for OFF, 1 for ON
// macro for debugging
#define DEBUG 1

#if DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT_FLOAT(x, d) Serial.print(x, d)
#define DEBUG_PRINTLN_FLOAT(x, d) Serial.println(x, d)
#define DEBUG_PRINT_HEX(x) Serial.print(x, HEX)
#define DEBUG_PRINTLN_HEX(x) Serial.println(x, HEX)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_FLOAT(x, d)
#define DEBUG_PRINTLN_FLOAT(x, d)
#define DEBUG_PRINT_HEX(x)
#define DEBUG_PRINTLN_HEX(x)
#endif

// for Feather M0 Lora:
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Singleton instance of the radio driver

// for HX711
#define DAT 5
#define CLK 21  //SCL
HX711 scale;

// for pressure transducers
#define pt1 A0
#define pt2 A1
#define pt3 A2
#define pt4 A3
#define pt5 A4
#define pt6 A5
#define pt7 0  // Rx

unsigned long lastRadioMessageTime;

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

uint8_t dataACKaddress = 0x90;

uint8_t servocommandaddress = 0xBA;

uint8_t servoACKbuffer[3];
uint8_t servoACKaddress = 0xA3;

uint8_t weightaddress = 0x84;
float calibration_factor = (6300 - 5300) / 211.4;  // (raw weight data - raw zero data) / known weight
float weight;

uint8_t pressureaddress = 0xCE;
const uint8_t numSensors = 7;
uint8_t pressurepins[numSensors] = { pt1, pt2, pt3, pt4, pt5, pt6, pt7 };
uint16_t rawpressures16[numSensors];
uint8_t rawpressures8[numSensors * 2];

void setup() {
  setServoDefault();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  DEBUG_PRINTLN("Feather LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    DEBUG_PRINTLN("LoRa radio init failed");
    DEBUG_PRINTLN("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1)
      ;
  }
  DEBUG_PRINTLN("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    DEBUG_PRINTLN("setFrequency failed");
    while (1)
      ;
  }
  DEBUG_PRINT("Set Freq to: ");
  DEBUG_PRINTLN(RF95_FREQ);
  rf95.setTxPower(23, false);

  // ADC Setup
  scale.begin(DAT, CLK);
  DEBUG_PRINTLN("Remove all weight from the scale...");
  delay(10000);  // to ensure the scale is initialized fully - I noticed that if this delay is too quick, then the scale is still "ramping up" to its full equilibrium value.
  scale.tare();  // Reset scale to 0 with no load
  DEBUG_PRINTLN("Scale tared.");
  scale.set_scale(calibration_factor);  // raw data / known weight
  delay(200);                           // delay to make sure scale.is_ready is actually ready after calibration

  // set ADC resolution (for pressure transducers)
  analogReadResolution(12);

  int watchdogTimeout = Watchdog.enable(2000);  // 2 second hardware watchdog
  lastRadioMessageTime = millis();
}

void loop() {

  ReadPackSendWeight();
  ReadPackSendPressure();


  if (millis() - lastRadioMessageTime > 2000) {  // 2 second radio watchdog
    setServoDefault();
    DEBUG_PRINTLN("RADIO WATCH");
    NVIC_SystemReset();
  }

  Watchdog.reset();

  if (rf95.waitAvailableTimeout(100)) {
    // Should be a reply message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      for (int i = 0; i < len; i++) {
        DEBUG_PRINT("Byte ");
        DEBUG_PRINT(i);
        DEBUG_PRINT(": ");
        DEBUG_PRINTLN_HEX(buf[i]);
      }
      DEBUG_PRINTLN(len);
      command_decoder(buf, len);
      rec_dataACK(buf, len);
    } else {
      DEBUG_PRINTLN("Receive failed");
    }
  } else {
    DEBUG_PRINTLN("No reply received (timeout)");
  }
}

// byte commands
void command_decoder(uint8_t* buf, uint8_t len) {
  DEBUG_PRINTLN("ENTERING DECODER");
  DEBUG_PRINTLN(len);
  if (len == 3) {
    DEBUG_PRINTLN("good length command");
    // Comparing addresses
    uint8_t address = buf[0];
    if (address != servocommandaddress) {
      DEBUG_PRINT("unknown command address: 0x");
      DEBUG_PRINTLN_HEX(address);
      return;
    }
    // Comparing CRC
    uint8_t recCRC = buf[2];
    uint8_t calcCRC = computeCRC8(buf, 2);
    if (recCRC != calcCRC) {
      DEBUG_PRINTLN("bad command crc");
      return;
    }
    lastRadioMessageTime = millis();
    DEBUG_PRINTLN("RESET SERVO RADIODOG");
    if (buf[1] == 1) {
      servocommand1();
    }
    if (buf[1] == 2) {
      servocommand2();
    }
  }
  DEBUG_PRINTLN("EXITING DECODER");
}

// Reads, packs, and sends weight data
void ReadPackSendWeight() {
  if (scale.is_ready()) {
    weight = scale.get_units(10);  // Get weight, average of 10 readings

    // for testing
    DEBUG_PRINT("Weight: ");
    DEBUG_PRINTLN_FLOAT(weight, 2);
    DEBUG_PRINTLN(" g");
    //
  } else {
    DEBUG_PRINTLN("HX711 not ready");
  }

  uint8_t weightbuffer[6];  //address + 4 bytes (float) + crc8 (null implied / not needed for binary data)
  memcpy(weightbuffer + 1, &weight, 4);
  weightbuffer[0] = weightaddress;
  uint8_t weightcrc = computeCRC8(weightbuffer, 5);  // calculating crc over the first 5 bytes only
  weightbuffer[5] = weightcrc;
  rf95.send(weightbuffer, sizeof(weightbuffer));
  rf95.waitPacketSent();
}

// Pressure related functions

// Reads the analogpins, packs all the pressure data into 1 buffer, and sends the whole message where it will decoded on the other side.
void ReadPackSendPressure() {

  for (uint8_t i = 0; i < numSensors; i++) {
    rawpressures16[i] = analogRead(pressurepins[i]);
  }
  uint8_t pressurebuffer[16];  // address + 7x2bytes (uint16_t) + CRC
  pressurebuffer[0] = pressureaddress;

  for (int i = 0; i < numSensors; i++) {                     // big endian
    rawpressures8[2 * i] = (rawpressures16[i] >> 8) & 0xFF;  // High byte first
    rawpressures8[2 * i + 1] = rawpressures16[i] & 0xFF;     // Low byte second
  }
  for (uint8_t i = 0; i < numSensors * 2; i++) {
    pressurebuffer[1 + i] = rawpressures8[i];
  }
  uint8_t pressurecrc = computeCRC8(pressurebuffer, 15);
  pressurebuffer[15] = pressurecrc;
  DEBUG_PRINT("pressurebuffer: ");
  for (uint8_t i = 0; i < numSensors * 2; i++) {
    DEBUG_PRINT(pressurebuffer[i]);
    DEBUG_PRINT(",");
  }
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("test2");
  DEBUG_PRINT("buffer size: ");
  DEBUG_PRINTLN(sizeof(pressurebuffer));
  rf95.send(pressurebuffer, sizeof(pressurebuffer));
  DEBUG_PRINTLN("test3");
  rf95.waitPacketSent();
  DEBUG_PRINTLN("test4");
}

//local command and send servo ACK
void servocommand1() {
  // SERVO COMMANDS HERE
  digitalWrite(LED_BUILTIN, HIGH); // test LED standing in for servo response
  DEBUG_PRINTLN("Sending servo 1 ACK");
  uint8_t servoACKbuffer[3];
  servoACKbuffer[0] = servoACKaddress;
  servoACKbuffer[1] = 1;
  servoACKbuffer[2] = computeCRC8(servoACKbuffer, 2);

  // sending multiple acks so it actually catches
  for (int i = 0; i < 10; i++){  
    rf95.send(servoACKbuffer, sizeof(servoACKbuffer));
    rf95.waitPacketSent();
    delay(10);
  }
}

void servocommand2() {
  // SERVO COMMANDS HERE
  DEBUG_PRINTLN("Sending servo 2 ACK");
  uint8_t servoACKbuffer[3];
  servoACKbuffer[0] = servoACKaddress;
  servoACKbuffer[1] = 2;
  servoACKbuffer[2] = computeCRC8(servoACKbuffer, 2);
  rf95.send(servoACKbuffer, sizeof(servoACKbuffer));
  rf95.waitPacketSent();
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

void setServoDefault() {
  // this is critical because when the watchdog resets, this will get the servo valves to "abort" position.
  // this will run everytime setup is called (and only once for a watchdog reset)
}

void rec_dataACK(uint8_t* buf, uint8_t len) {
  DEBUG_PRINTLN(len);
  if (len == 2) {
    DEBUG_PRINTLN("GOOD LEN DATAACK");
    uint8_t address = buf[0];
    if (address != dataACKaddress) {
      DEBUG_PRINT("unknown address ACK: 0x");
      DEBUG_PRINTLN_HEX(address);
      return;
    }
    uint8_t recCRC = buf[1];
    uint8_t calcCRC = computeCRC8(buf, 1);
    if (recCRC != calcCRC) {
      DEBUG_PRINTLN("Bad ACK CRC");
      return;
    }
    DEBUG_PRINTLN("reset radio watchdog");
    lastRadioMessageTime = millis();
    delay(30);
  }
}