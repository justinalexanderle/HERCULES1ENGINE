
#include <SPI.h>
#include <RH_RF95.h>
#include <HX711.h>
#include <Adafruit_SleepyDog.h>

//macro for debugging
#define DEBUG 0

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

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Setting up button(s)
#define testpin A1
#define testpin2 A5

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

uint8_t dataACKbuffer[2];
uint8_t dataACKaddress = 0x90;

uint8_t servocommandbuffer[3];
uint8_t servocommandaddress = 0xBA;

uint8_t servoACKaddress = 0xA3;

uint8_t weightaddress = 0x84;
float weight;

const uint8_t numSensors = 7;
float pressures[numSensors];
uint8_t rawpressures8[numSensors * 2];
uint16_t rawpressures16[numSensors];
uint8_t pressureaddress = 0xCE;

bool hasWeight;
bool hasPressure;

bool testpinstate = true;

void setup() {
  // creating data ACK buffer
  dataACKbuffer[0] = dataACKaddress;  // address
  dataACKbuffer[1] = computeCRC8(dataACKbuffer, 1);

  pinMode(testpin, INPUT);
  pinMode(testpin2, INPUT);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  DEBUG_PRINTLN("Feather LoRa TX Test!");

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

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  int watchdogTimeout = Watchdog.enable(2000);  // 2 second hardware watchdog
}

void loop() {
  Watchdog.reset();

  if (rf95.waitAvailableTimeout(50)) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      rec_servoACK(buf, len);
      weightPacketHandler(buf, len);
      pressurePacketHandler(buf, len);
      CSV_Serial_and_senddataACK();
    }
  }  

  if (digitalRead(testpin) == HIGH && testpinstate == true) {
    delay(20);
    DEBUG_PRINTLN("TESTPIN 1 HIGH");
    servocommandbuffer[0] = servocommandaddress;
    servocommandbuffer[1] = 1;
    servocommandbuffer[2] = computeCRC8(servocommandbuffer, 2);
    DEBUG_PRINTLN_HEX(servocommandbuffer[0]);
    DEBUG_PRINTLN(servocommandbuffer[1]);
    DEBUG_PRINTLN(servocommandbuffer[2]);

    for (int i = 0; i < 10; i++){ // sending multiple commands so it actually catches
      rf95.send(servocommandbuffer, sizeof(servocommandbuffer));
      rf95.waitPacketSent();
      delay(10);
    }
    testpinstate = false;
  }
  // if (digitalRead(testpin) == LOW) {
  //   digitalWrite(LED_BUILTIN, LOW);
  // }

  if (digitalRead(testpin2) == HIGH) {
    delay(20);
    DEBUG_PRINTLN("TESTPIN 2 HIGH");
    servocommandbuffer[0] = servocommandaddress;
    servocommandbuffer[1] = 2;
    servocommandbuffer[2] = computeCRC8(servocommandbuffer, 2);
    DEBUG_PRINTLN_HEX(servocommandbuffer[0]);
    DEBUG_PRINTLN(servocommandbuffer[1]);
    DEBUG_PRINTLN(servocommandbuffer[2]);
    rf95.send(servocommandbuffer, sizeof(servocommandbuffer));
    rf95.waitPacketSent();
  }
}

float RawtoFloatPressure(uint16_t raw) {
  return (((raw * (3.33 / 4095.00)) - 0.33) * (1600.00 / 2.37));
}

void pressurePacketHandler(uint8_t* buf, uint8_t len) {
  // PACKET HANDLER: PRESSURE DATA
  // Checking the buf length is 16 (address + 12x uint8_t (1 bytes) + CRC)
  if (len == 16) {
    DEBUG_PRINTLN("good length 16");
    // Comparing addresses
    uint8_t address = buf[0];
    if (address != pressureaddress) {
      DEBUG_PRINT("unknown address 16: 0x");
      DEBUG_PRINTLN_HEX(address);
      return;
    }

    // Comparing CRC
    uint8_t recCRC = buf[15];
    uint8_t calcCRC = computeCRC8(buf, 15);
    if (recCRC != calcCRC) {
      memset(rawpressures8, 0, sizeof(rawpressures8));
      memset(rawpressures16, 0, sizeof(rawpressures16));
      hasPressure = false;
      DEBUG_PRINTLN("bad crc");
      return;
    }
    DEBUG_PRINTLN("printing");
    // Extracting uint8_t data
    for (uint8_t i = 0; i < (numSensors * 2); i++) {
      memcpy(&rawpressures8[i], buf + 1 + i, 1);
    }

    DEBUG_PRINT("raw8: ");
    for (uint8_t i = 0; i < 12; i++) {
      DEBUG_PRINT(rawpressures8[i]);
      DEBUG_PRINT(",");
    }
    DEBUG_PRINTLN();

    //need to convert 8bit to 16bit, big endian
    for (uint8_t i = 0; i < numSensors; i++) {
      rawpressures16[i] = ((uint16_t)rawpressures8[2 * i] << 8) | rawpressures8[2 * i + 1];
    }
    DEBUG_PRINT("raw16: ");
    for (uint8_t i = 0; i < 6; i++) {
      DEBUG_PRINT(rawpressures16[i]);
      DEBUG_PRINT(",");
    }
    DEBUG_PRINTLN();
    // Converting 6x uint16_t to 6x floats (for printability later) and voltage to pressure conversion
    for (uint8_t i = 0; i < numSensors; i++) {
      pressures[i] = RawtoFloatPressure(rawpressures16[i]);
    }
    hasPressure = true;
    //for testing
    for (uint8_t i = 0; i < numSensors; i++) {
      DEBUG_PRINT_FLOAT(pressures[i], 2);
      if (i < numSensors - 1) {
        DEBUG_PRINT(",");
      }
    }
    DEBUG_PRINTLN();
    //
  }
}

void weightPacketHandler(uint8_t* buf, uint8_t len) {
  // PACKET HANDLER: WEIGHT DATA
  // Checking the buf length is size we expect (address + float (4 bytes) + CRC)
  if (len == 6) {
    // Comparing addresses
    uint8_t address = buf[0];
    if (address != weightaddress) {
      DEBUG_PRINT("unknown address 6: 0x");
      DEBUG_PRINTLN_HEX(address);
      return;
    }
    // Comparing CRC
    uint8_t recCRC = buf[5];
    uint8_t calcCRC = computeCRC8(buf, 5);
    if (recCRC != calcCRC) {
      weight = 0.0;
      hasWeight = false;
      return;
    }

    // Extracting buffer float data into weight
    memcpy(&weight, buf + 1, sizeof(float));

    hasWeight = true;
    //for testing
    DEBUG_PRINT("Weight: ");
    DEBUG_PRINT_FLOAT(weight, 2);
    DEBUG_PRINTLN(" kg");
    //
  }
}

void CSV_Serial_and_senddataACK() {
  // CSV Printing to Serial
  if (hasWeight && hasPressure) {
    DEBUG_PRINTLN("Synched Reading:");
    Serial.print(weight, 2);
    Serial.print(",");

    for (uint8_t i = 0; i < numSensors; i++) {
      Serial.print(pressures[i], 2);
      if (i < numSensors - 1) {
        Serial.print(",");
      }
    }
    Serial.println();
    // reset on print
    hasWeight = false;
    hasPressure = false;
    DEBUG_PRINTLN("GOOD SERIAL");
    senddataACK();  // sending ACK to Rx to kick watchdog
  }
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

void senddataACK() {
  delay(20);  //critical for letting the radio switch from receive to send mode, between 20-50 ms (50 is safer)
  DEBUG_PRINTLN("GOOD BUFFER");
  rf95.send(dataACKbuffer, sizeof(dataACKbuffer));
  rf95.waitPacketSent();
  DEBUG_PRINTLN("GOOD SEND");
  DEBUG_PRINTLN(dataACKbuffer[0]);
  DEBUG_PRINTLN(dataACKbuffer[1]);
}

void rec_servoACK(uint8_t* buf, uint8_t len) {
  DEBUG_PRINTLN("ENTERING SERVO ACK");
  DEBUG_PRINTLN(len);
  if (len == 3) {
    // Comparing addresses
    uint8_t address = buf[0];
    if (address != servoACKaddress) {
      DEBUG_PRINT("unknown address servo: 0x");
      DEBUG_PRINTLN_HEX(address);
      return;
    }

    // Comparing CRC
    uint8_t recCRC = buf[2];
    uint8_t calcCRC = computeCRC8(buf, 2);
    if (recCRC != calcCRC) {
      DEBUG_PRINT("bad servoACK crc");
      return;
    }

    if (buf[1] == 1) {
      // turn on SERVO 1 LED ("ABORT" or something)
      DEBUG_PRINTLN("SERVO 1 ACK GOOD");
      digitalWrite(LED_BUILTIN, HIGH); //test led, write it to our actual LED pin
    }
    if (buf[1] == 2) {
      // turn on SERVO 2 LED ("PRIME" or something)
      DEBUG_PRINTLN("SERVO 2 ACK GOOD");
    }
  }
}