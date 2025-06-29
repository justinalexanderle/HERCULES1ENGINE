
#include <SPI.h>
#include <RH_RF95.h>
#include <HX711.h>

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

#define RFM95_CS   8
#define RFM95_RST  4
#define RFM95_INT  3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Setting up button(s)
#define testpin 5
#define testpin2 6

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
  
uint8_t buttonstate = 0;
uint8_t weightaddress = 0x84;
float weight;

const uint8_t numSensors = 6;
float pressures[numSensors];
uint8_t rawpressures8[numSensors*2];
uint16_t rawpressures16[numSensors];
uint8_t pressureaddress = 0xCE;

bool hasWeight;
bool hasPressure;


void setup() {
  pinMode(testpin, INPUT);

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
    while (1);
  }
  DEBUG_PRINTLN("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    DEBUG_PRINTLN("setFrequency failed");
    while (1);
  }
  DEBUG_PRINT("Set Freq to: "); DEBUG_PRINTLN(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop() {
  
  
  // if (digitalRead(testpin) == HIGH && buttonstate == 0){        // test pin
  //   delay(500);
  //   buttonstate = 1;                                            // state checking variable (edge detection)
  //   DEBUG_PRINTLN("button 1 pressed");
  //   uint8_t radiopacket[18] = "HERCULESJCJL 01";
  //   radiopacket[17] = 0;
  //   rf95.send(radiopacket, sizeof(radiopacket));
  //   DEBUG_PRINTLN("Sending... button");
  // }
  // if (digitalRead(testpin) == LOW & buttonstate == 1){          // edge detection
  //   delay(500);
  //   buttonstate = 0;  
  //   DEBUG_PRINTLN("Edge detected");
  // }

  // just making it not run all the checks below if there is no data available
  if (!rf95.available()) {
    //DEBUG_PRINTLN("no data");
    return;  
  }
  len = sizeof(buf); // critical for processing to reinitialize the size of len based on the newest buf. I think the code moves too fast and it keeps track of the len = 6 from weight and doesn't change                                   
  // if the receive fails, it returns. It also passes the buf and len if the receive succeeds (supposedly according to chatgpt)
  // if we need to just make another rf95.recv and fold everything below into an if, we can do that easily
  if (!rf95.recv(buf, &len)) {                         
    DEBUG_PRINTLN("Receive failed");
    return;
  }

  weightPacketHandler();
  pressurePacketHandler();
  CSV_Serial();

  // for ACK and error codes
  if (len == 21){
    DEBUG_PRINT((char*)buf);                                   // Should be printing ACK ("servo command good" or something) or "Wrong recipient"
  } 

              
}

float RawtoFloatPressure(uint16_t raw){
  return (((raw * (3.33 / 4095.00)) - 0.33) * (1600.00 / 2.37));
}

void pressurePacketHandler(){
  // PACKET HANDLER: PRESSURE DATA
  // Checking the buf length is 14 (address + 12x uint8_t (1 bytes) + CRC)
  if (len == 14){ 
    DEBUG_PRINTLN("good length 14");
    // Comparing addresses
    uint8_t address = buf[0];
    if (address != pressureaddress){
      DEBUG_PRINT("unknown address 14: 0x");
      DEBUG_PRINTLN_HEX(address);
      return;
    }   

    // Comparing CRC
    uint8_t recCRC = buf[13];
    uint8_t calcCRC = computeCRC8(buf,13);
    if (recCRC != calcCRC){                             
      memset(rawpressures8, 0, sizeof(rawpressures8));   
      memset(rawpressures16, 0, sizeof(rawpressures16));    
      hasPressure = false;
      DEBUG_PRINTLN("bad crc");
      return;
    }   
    DEBUG_PRINTLN("printing");
    // Extracting uint8_t data
    for (uint8_t i = 0; i < (numSensors*2); i++){
      memcpy(&rawpressures8[i], buf+1+i, 1); 
    }

      DEBUG_PRINT("raw8: ");
    for (uint8_t i = 0;i < 12; i++){
      DEBUG_PRINT(rawpressures8[i]);
      DEBUG_PRINT(",");
    }
    DEBUG_PRINTLN();

    //need to convert 8bit to 16bit, big endian
    for (uint8_t i = 0; i < numSensors; i++) {
      rawpressures16[i] = ((uint16_t)rawpressures8[2 * i] << 8) | rawpressures8[2 * i + 1];
    }
      DEBUG_PRINT("raw16: ");
    for (uint8_t i = 0;i < 6; i++){
      DEBUG_PRINT(rawpressures16[i]);
      DEBUG_PRINT(",");
    }
    DEBUG_PRINTLN();
    // Converting 6x uint16_t to 6x floats (for printability later) and voltage to pressure conversion
    for (uint8_t i = 0; i < numSensors; i++){
      pressures[i] = RawtoFloatPressure(rawpressures16[i]);
    }
    hasPressure = true;
    //for testing
    for (uint8_t i = 0; i < numSensors; i++){
      DEBUG_PRINT_FLOAT(pressures[i],2);
      if (i < numSensors - 1){
        DEBUG_PRINT(",");
      }
    }             
    DEBUG_PRINTLN();     
    //

  }
}

void weightPacketHandler(){
  // PACKET HANDLER: WEIGHT DATA
  // Checking the buf length is size we expect (address + float (4 bytes) + CRC)
  if (len == 6){                                      
    // Comparing addresses
    uint8_t address = buf[0];
    if (address != weightaddress){
      DEBUG_PRINT("unknown address 6: 0x");
      DEBUG_PRINTLN_HEX(address);
      return;
    }  
    // Comparing CRC
    uint8_t recCRC = buf[5];
    uint8_t calcCRC = computeCRC8(buf,5);
    if (recCRC != calcCRC){                              
      weight = 0.0;               
      hasWeight = false;
      return;
    }

    // Extracting buffer float data into weight
    memcpy(&weight, buf + 1, sizeof(float));

    hasWeight = true;     
    //for testing
    DEBUG_PRINT("Weight: ");
    DEBUG_PRINT_FLOAT(weight,2); 
    DEBUG_PRINTLN(" kg"); 
    //         
  }
}

void CSV_Serial(){
  // CSV Printing to Serial
  if (hasWeight && hasPressure){
    Serial.println("Synched Reading:");
    Serial.print(weight,2);
    Serial.print(","); 

    for (uint8_t i = 0; i < numSensors; i++){
      Serial.print(pressures[i],2);
      if (i < numSensors - 1){
        Serial.print(",");
      }
    }             
    Serial.println();  
    // reset on print
    hasWeight = false;                                        
    hasPressure = false;
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

  
  
