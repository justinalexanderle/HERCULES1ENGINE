#include <SPI.h>
#include <RH_RF95.h>
#include <HX711.h>

// macro for debugging, 0 for OFF, 1 for ON
//macro for debugging
#define DEBUG 1

#if DEBUG
  #define DEBUG_PRINT(x)         Serial.print(x)
  #define DEBUG_PRINTLN(x)       Serial.println(x)
  #define DEBUG_PRINT_FLOAT(x,d) Serial.print(x, d)
  #define DEBUG_PRINTLN_FLOAT(x,d) Serial.println(x, d)
  #define DEBUG_PRINT_HEX(x)       Serial.print(x, HEX)
  #define DEBUG_PRINTLN_HEX(x)     Serial.println(x, HEX)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT_FLOAT(x,d)
  #define DEBUG_PRINTLN_FLOAT(x,d)
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
#define DAT 12
#define CLK 11
HX711 scale;

// for pressure transducers
#define pt1 A0
#define pt2 A1
#define pt3 A2
#define pt4 A3
#define pt5 A4
#define pt6 A5

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

uint8_t weightaddress = 0x84;
float calibration_factor = 4257.0;  // raw data / known weight
float weight;

uint8_t pressureaddress = 0xCE;
const uint8_t numSensors = 6;
uint8_t pressurepins[numSensors] = {pt1, pt2, pt3, pt4, pt5, pt6};
uint16_t rawpressures16[numSensors];
uint8_t rawpressures8[12];

void setup() {
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
  delay(200);    // delay to make sure scale.is_ready is actually ready after calibration

  // set ADC resolution (for pressure transducers)
  analogReadResolution(12);
}

void loop() {
  ReadPackSendWeight();
  ReadPackSendPressure();
  // testcommand();
  
  if (rf95.available()) {
    if (rf95.recv(buf, &len)) {
      //RH_RF95::printBuffer("Received: ", buf, len);
      DEBUG_PRINT((char*)buf);
      DEBUG_PRINTLN(" RECEIVED");
      command_decoder();
      sendACK();
    }
  }
}

// ACK can be a colored light(s) or something if we can't use serial because were printing data.
void sendACK() {
  DEBUG_PRINTLN("Sending ACK");
  uint8_t radioack[17] = "Message Received";
  radioack[16] = '\0';
  rf95.send(radioack, sizeof(radioack));
}

void command_decoder() {
  if (len != 16) {  // checking for size of message (should be 16 bytes)
    DEBUG_PRINTLN("Wrong Message Length");
    uint8_t radioerror[21] = "WRONG RECIPIENT-JCJL";
    radioerror[20] = '\0';
    rf95.send(radioerror, sizeof(radioerror));
  }

  // housekeeping for our received message (for commands)
  if (len == 16) {           // checking for size of message (should be 16 bytes)
    char main[len + 1];      // size of "radiopacket" full message (from tx)
    memcpy(main, buf, len);  // copying all of (char*)buf into an array we can manipulate
    main[len] = '\0';        // knowing size of our message is always 16, the 15th (last) index is manually set to null
    DEBUG_PRINTLN(strlen(main));
    DEBUG_PRINTLN(main);

    char password[13];            // size of password HERCULESJCJL +null
    strncpy(password, main, 12);  // copying only the first 12 chars into password
    password[12] = '\0';
    DEBUG_PRINTLN(password);

    char launchcode[3];                               // size of launchcode [XX] +null
    strncpy(launchcode, main + strlen(main) - 2, 2);  // copies the last 2 chars from main
    launchcode[2] = '\0';
    DEBUG_PRINTLN(launchcode);
    int test = sizeof(launchcode);
    DEBUG_PRINTLN(test);

    // functions for if the message is what we're expecting
    if (strcmp(password, "HERCULESJCJL") == 0) {  // addressing
      DEBUG_PRINTLN("Good Password");

      if (strcmp(launchcode, "01") == 0) {  // launch code 01
        DEBUG_PRINTLN("launch code 01 good");
        servocommand1();
      }
      if (strcmp(launchcode, "02") == 0) {  // launch code 01
        DEBUG_PRINTLN("launch code 02 good");
        servocommand2();
      }
    } else {
      DEBUG_PRINTLN("Bad Password or Launch Code");
    }
  }
}

// Reads, packs, and sends weight data
void ReadPackSendWeight() {
  if (scale.is_ready()) {
    weight = scale.get_units(10);  // Get weight, average of 10 readings

    // for testing
    // DEBUG_PRINT("Weight: ");
    DEBUG_PRINTLN_FLOAT(weight, 2);
    // DEBUG_PRINTLN(" g");
    //
  } else {
    DEBUG_PRINTLN("HX711 not ready");
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
    rawpressures16[i] = analogRead(pressurepins[i]);
  }
  // DEBUG_PRINTLN("raw16: ");
  // for (uint8_t i = 0; i < numSensors; i++) {
  //   DEBUG_PRINT(rawpressures16[i]);
  //   DEBUG_PRINT(",");
  // }
  // DEBUG_PRINTLN();
  
  uint8_t pressurebuffer[14];  // address + 6x2bytes (uint16_t) + CRC
  pressurebuffer[0] = pressureaddress;

  for (int i = 0; i < numSensors; i++) { // big endian
  rawpressures8[2 * i]     = (rawpressures16[i] >> 8) & 0xFF;  // High byte first
  rawpressures8[2 * i + 1] = rawpressures16[i] & 0xFF;         // Low byte second
  }
  for (uint8_t i = 0; i < numSensors*2; i++) {
    pressurebuffer[1 + i] = rawpressures8[i];
  }
  uint8_t pressurecrc = computeCRC8(pressurebuffer, 13);
  // DEBUG_PRINT("crc: ");
  // DEBUG_PRINTLN(pressurecrc);
  pressurebuffer[13] = pressurecrc;
  DEBUG_PRINT("pressurebuffer: ");
  for (uint8_t i = 0; i < 14; i++){
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
  delay(200);
}

void servocommand1() {
  // SERVO COMMANDS HERE
  DEBUG_PRINTLN("Sending ACK");
  uint8_t radioack[21] = "SERVOCOMMAND 01 GOOD";
  radioack[20] = '\0';
  rf95.send(radioack, sizeof(radioack));
}

void servocommand2() {
  // SERVO COMMANDS HERE
  DEBUG_PRINTLN("Sending ACK");
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

void testcommand (){
  uint8_t dataToSend[14] = {0,2, 104, 0, 100, 0, 98, 0, 100, 0, 101, 0, 93,0};
  dataToSend[0] = pressureaddress;
  uint8_t datacrc = computeCRC8(dataToSend, 13);
  dataToSend[13] = datacrc;
  DEBUG_PRINTLN("Sending packet...");
    for (uint8_t i = 0; i < 14; i++){
    DEBUG_PRINT(dataToSend[i]);
    DEBUG_PRINT(",");
  }
  rf95.send(dataToSend, sizeof(dataToSend));
  
  rf95.waitPacketSent();
  DEBUG_PRINTLN("Packet sent!");
}
