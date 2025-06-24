
#include <SPI.h>
#include <RH_RF95.h>
#include <HX711.h>

#define RFM95_CS   8
#define RFM95_RST  4
#define RFM95_INT  3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Setting up button(s)
#define testpin 5
#define testpin2 6

void setup() {
  pinMode(testpin, INPUT);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

uint8_t buttonstate = 0;
uint8_t weightaddress = 0x01;
float weight;


const uint8_t numSensors = 6;
float pressures[numSensors];
float rawpressures[numSensors];
uint8_t pressureaddress = 0x02;

bool hasWeight;
bool hasPressure;

void loop() {
  if (digitalRead(testpin) == HIGH && buttonstate == 0){        // test pin
    delay(500);
    buttonstate = 1;                                            // state checking variable (edge detection)
    Serial.println("button 1 pressed");
    uint8_t radiopacket[18] = "HERCULESJCJL 01";
    radiopacket[17] = 0;
    rf95.send(radiopacket, sizeof(radiopacket));
    Serial.println("Sending... button");
  }
  if (digitalRead(testpin) == LOW & buttonstate == 1){          // edge detection
    delay(500);
    buttonstate = 0;  
    Serial.println("Edge detected");
  }

  // just making it not run all the checks below if there is no data available
  if (!rf95.available()) return;                                       
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // if the receive fails, it returns. It also passes the buf and len if the receive succeeds (supposedly according to chatgpt)
  // if we need to just make another rf95.recv and fold everything below into an if, we can do that easily
  if (!rf95.recv(buf, &len)) {                         
    Serial.println("Receive failed");
    return;
  }


  // PACKET HANDLER: WEIGHT DATA
  // Checking the buf length is size we expect (address + float (4 bytes) + CRC)
  if (len = 6){                                      
    
    // Comparing addresses
    uint8_t address = buf[0];
    if (address != pressureaddress){
      Serial.print("unknown address: 0x");
      Serial.println(address, HEX);
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
    
  }

  // PACKET HANDLER: PRESSURE DATA
  // Checking the buf length is 14 (address + 6x uint16_t (2 bytes) + CRC)
  if (len == 14){     

    // Comparing addresses
    uint8_t address = buf[0];
    if (address != pressureaddress){
      return;
    }   

    // Comparing CRC
    uint8_t recCRC = buf[13];
    uint8_t calcCRC = computeCRC8(buf,13);
    if (recCRC != calcCRC){                              
      memset(rawpressures, 0, sizeof(rawpressures));    
      hasPressure = false;
      return;
    }   

    // Extracting uint16_t data
    for (uint8_t i = 0; i < numSensors; i++){
      memcpy(&rawpressures[i], buf+1+i*sizeof(uint16_t), sizeof(uint16_t)); 
    }

    // Converting 6x uint16_t to 6x floats (for printability later) and voltage to pressure conversion
    for (uint8_t i = 0; i < numSensors; i++){
      pressures[i] = RawtoFloatPressure(rawpressures[i]);
    }
    hasPressure = true;
  }

  // for ACK and error codes
  if (len == 21){
    Serial.print((char*)buf);                                   // Should be printing ACK ("servo command good" or something) or "Wrong recipient"
  } 

  // CSV Printing to Graphs
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

    hasWeight = false;                                        // reset on print
    hasPressure = false;
  }                  
}

float RawtoFloatPressure(uint16_t raw){
  return raw * (3.3 / 4095.0) * (1600.0 / 4.5);
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

  
  