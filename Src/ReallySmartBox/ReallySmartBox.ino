// Really Smart Box
// Measures the weight of the contents of a really smart box
// Made by two sheets of acrylic with 2 load cells between them 
// placed in a really smart box.
// Also includes a BME280 to measure temperature and pressure inside the box.
// Author: Stephen Harrison
// License: MIT

#include <Adafruit_BME280.h>
#include <HX711.h>
#include <SigFox.h>
#include <SigFox.h>
#include <ArduinoLowPower.h>

// --------------------------------------
// BME280 on the I2C port.
Adafruit_BME280 bme; 

// --------------------------------------
// HX711 load cell amplifier.
// 0: D0 - DOUT
// 1: D1 - CLK
// initial gain of 128.
HX711 scales(0, 1, 128);

// Arrays for load cells. Index 0 == Channel A, Index 1 == Channel B.
float gain[] = {128,32};

// Calibration factors.
// we use y = mx + c (c = offset, m = scaleFactor).
// to convert the measured value into a weight.
// Set this to the offset reported by the load cells.
// with no weight on them.
float offset[] = {0,54940}; 

// Set this to the factor computed when a weight is placed on the scale.
// Set the offset first, re-flash the arduiono for this to take effect
// place a weight on the scale and divide the raw measured value by the weight.
// using scaleFactor = measured value / weight.
float scaleFactor[] = {378.f,260.9f};

// --------------------------------------
// Sigfox

// This is the data structure we publish to Sigfox.
// Split out the bits as bool flags from the first status byte but the byte still needs to be 
// included otherwise humidity becomes the status
// firstRun::bool:7 hx711Fault::bool:6 bmeFault::bool:5 temperatureAlarm::bool:4 humidityAlarm::bool:3 weightAlarm::bool:2 lowStock::bool:1 b0::bool:0
// status::int:8 humidity::int:8 temperature::int:8 zeroWeight::int:16:little-endian weight::int:16:little-endian itemCount::int:16:little-endian
typedef struct __attribute__ ((packed)) sigfox_message {
  uint8_t status;       // status::uint:8 -> Split to 8 bits // B7 - First run, B6 - HX711 fault, B5 BME280 fault, B4 Temperature alarm, B3 - Humidity alarm, B2 - weight alarm, B1 - Low stock, B0 - spare
  int8_t humidity;      // humidity::int:8 (yes some sensors (HTU21D read -ve humidity)
  int8_t temperature;   // temperature::int:8 (no decimal places).
  int16_t zeroWeight;   // zeroWeight::int:16:little-endian 
  int16_t weight;       // weight::int:16:little-endian
  int16_t itemCount;    // itemCount::int:16:little-endian (100x actual item count to allow for 2.01 (as weight won't match exactly)
  int8_t driftCorrection;    // Drift Correction for changes in zero weight applied to the scales.
  int8_t filler;
  int8_t lastStatus;    // Last sigfox status
} SigfoxMessage;

// Time the last Sigfox message was published at
long lastPublish = 0;

// Time the last Sigfox downlink was requested.
// Allowed max 4 per day of these.
long lastDownlink = 0;

uint8_t lastSigfoxStatus = 0;

// --------------------------------------
// Application/state

// If the fist cycle (after a reset) for the measure/publish
// cycle (this is used to request a downlink message from Sigfox).
// Note that only 4 of them are allowed per day so becareful
// when deploying code.
bool isFirstCycle = true;

// Application mode
// 0: Normal
// 1: Calibration
int mode = 0;

// Which channel should be read during calibration.
int calibrate_channel = 0;

// The last average value measured for each channel.
float lastAverage[] = {0,0};

// The current weight of the contents of the box
float currentWeight = 0;

// The weight of the units the box will hold.
// Updatable via Sigfox downlink message.
float unitWeight = 238;

// Different to tare as it would be a manual
// zero'd at a set reading from scales
// This will most likely change with drift (time/temperature/etc)
// and should be set once the scale is in place but not loaded.
// Updatable via Sigfox downlink message.
float zeroWeight = 0;

bool bmeOk = true;
bool hx711Ok = true;

// Alarms and alarm ranges
float minTemperature = 5.f;
float maxTemperature = 60.f;
float minHumidity = 0.f;
float maxHumidity = 60.f;
float maxWeight = 10000; // 10kg

bool temperatureAlarm = false;
bool humidityAlarm = false;
bool weightAlarm = false;

float currentTemperature = 0;
float currentHumidity = 0;

float stockLevel = 0;
bool lowStock = false;
float minStock = 5;

// Setup the Arduino.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialize serial:
  Serial.begin(9600);

  // NB: The sensor I'm using (from random eBay seller)
  // does not use the default address.
  bmeOk = bme.begin(0x76);  
  if (!bmeOk) {
    Serial.println("Could not find a valid BME280 sensor!");
  } 

  // Delay for USB Serial connect and for the BME's first reading.
  delay(5000);
  Serial.println("Really Smart Box...");
  
  printHeader();
}

int delayCounter = 0;

void loop() {
  switch (mode) {
    case 0:
      measureAndPublish();
      //Sleep for 1 minutes
      // Causing problems with USB connected.
      //LowPower.sleep(1 * 60 * 1000);
      delay(60 * 1000);
      break;
    case 1:
      calibrate();
      delay(1000);
      break;
  }

  // Check for user input via the serial port.
  checkSerial();

  // measure is done on RTC timer tick (once per minute)
  delay(100);
}

void measureAndPublish() { 
  // turn the LED on to indicate measuring.
  digitalWrite(LED_BUILTIN, HIGH);   
  printBmeValues();
  measureTemperatureAndHumidity();
  measureWeight(true);

  // Weight, temperature and humidity are read every minute
  // however we only publish occasionally.
  if (shouldPublish()) {
    publishMeasurements();
  } 
  
  digitalWrite(LED_BUILTIN, LOW);  
}

// Main measurement loop. Reads the weight from the load cells
// and stores if no noise from the previous read.
void measureWeight(bool printDetails) {
   
  scales.power_up();
  delay(500);

  float delta = readDelta(printDetails);
  if (printDetails) {
    Serial.print("\t");
    Serial.print(delta, 2);
  }

  // If the delta is between -1 and 1 (i.e. no noise)
  // update the change in overall weight and units contained
  // otherwise ignore and try again later on.
  // This ensures we use only stable readings when both channels have not changed for 
  // two sets of measurements.
  if (delta < 1.f && delta > -1.f) {
    // Remember the previous measured weight so we can get a delta.
    float lastWeight = currentWeight;
    
    // Compute the weight. Take the weight of both load cells 
    // added together then subtract the zero'd weight.
    currentWeight = lastAverage[0] + lastAverage[1] - zeroWeight;

    // Compute the difference in weight of the items in the box
    // compated to the last time we had a stable reading.
    float itemsWeightDelta = currentWeight - lastWeight;

    updateStockLevels();
    
    if (printDetails) {
      Serial.print("\t");
      Serial.print("\t");
      Serial.print(currentWeight, 2);

      Serial.print("\t");
      // divide by unit weight to estimate the stock level in the box
      Serial.print(currentWeight / unitWeight, 2);

      Serial.print("\t");
      // the change in weight, (i.e. the weight if the items added)
      Serial.print(itemsWeightDelta, 2);

      Serial.print("\t");
      // divide by unit weight to estimate the units removed/added
      Serial.print(itemsWeightDelta / unitWeight, 2);
    }
  }

  checkWeightLimits();

  if (printDetails) {
    Serial.println();
  }

  // put the ADC in sleep mode and switch 
  // off the LED now we're done measuring.
  scales.power_down();                
}

void updateStockLevels() {
  stockLevel = currentWeight / unitWeight;

  // Unlike other alarms the low stock level
  // is reset if the stock is re-stocked.
  lowStock = stockLevel < minStock;
}

// Check if the current total weight
// or a single load cell weight is out of range.
void checkWeightLimits() {
  if (currentWeight > maxWeight ) {
    weightAlarm = true;
  }

  if (lastAverage[0] > (maxWeight /2)) {
    weightAlarm = true;
  }
  
  if (lastAverage[1]> (maxWeight /2)) {
    weightAlarm = true;
  }
}

// Read the difference in weight from the last 
// average to this time across both load cells.
// average value is stored in the lastAverage array.
float readDelta(bool printDetails) {
  float aDelta = readChannel(0, true);
  if (printDetails) {
    Serial.print("\t");
  }
  float bDelta = readChannel(1, true);

  return aDelta + bDelta;
}

// Read the weight from a channel. Stores the measured value in 
// the lastAverage array and retuns the delta of the measured value
// from the previous lastAverage. This allows us to know if the weight
// has changed.
// channel 0 = A
// channel 1 = B
float readChannel(int channel, bool printDetails) {
  
  // Gain:
  // Channel A supports 128 or 64. Default 128
  // Channel B supports 32
  // Select Channel B by using gain of 32.
  scales.set_gain(gain[channel]); 
  
  // HX711 library only has one set of offset/scale factors
  // which won't work for use as we use both channels and they 
  // have different gains, so each needs to have it's offset/scale set 
  // before reading the value.
  scales.set_offset(offset[channel]);
  scales.set_scale(scaleFactor[channel]);
  
  // force read to switch to gain.
  scales.read();
  scales.read();

  float singleRead = scales.get_units();
  float average = scales.get_units(10);
  float delta = average - lastAverage[channel];
  
  if (printDetails) {
    Serial.print(singleRead, 1);
    Serial.print("\t");
    Serial.print(average, 1);
    Serial.print("\t");
    Serial.print(delta, 1);
    Serial.print("\t");
  }
  lastAverage[channel] = average;
  return delta;
}

// print the header for the debug data pushed out when measuring.
void printHeader() {
  Serial.print("BME280\t\t\t\t\t");
  Serial.print("Channel A\t\t\t");
  Serial.print("Channel B\t\t\t");
  Serial.print("\t\t");
  Serial.print("Totals \t\t\t");
  Serial.println("");

  Serial.print("Temp\t");
  Serial.print("Pressure\t");
  Serial.print("Humidity\t");
  
  Serial.print("read\t");
  Serial.print("average\t");
  Serial.print("delta\t");

  Serial.print("\t");
  
  Serial.print("read\t");
  Serial.print("average\t");
  Serial.print("delta\t");

  Serial.print("\t");

  Serial.print("sum\t");
  
  Serial.print("\t");

  Serial.print("weight\t");
  Serial.print("items\t");
  Serial.print("change\t");
  Serial.print("items added");
  Serial.println("");
}

// Calibration - reads/prints selected channel values.
void calibrate() { 
  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  scales.set_gain(gain[calibrate_channel]); 
  scales.set_offset(offset[calibrate_channel]);
  scales.set_scale(scaleFactor[calibrate_channel]);
  
  // force read to switch to gain
  Serial.print("\t|CH:\t");
  Serial.print(calibrate_channel,1);
  Serial.print("\traw:\t");
  Serial.print(scales.read(),1);
  Serial.print("\t| raw:\t");
  Serial.print(scales.read(),1);
  Serial.print("\t| units:\t");
  Serial.print(scales.get_units(), 1);
  Serial.print("\t| gain:\t");
  Serial.print(gain[calibrate_channel], 1);
  Serial.print("\t| factor:\t");
  Serial.println(scaleFactor[calibrate_channel], 1);

  digitalWrite(LED_BUILTIN, LOW);
}

// check the serial port for input from a console to allow us to alter 
// the device mode etc.
void checkSerial() {
  if(Serial.available())
  {
    char instruction = Serial.read();

    switch (instruction) {
      case '0':
        calibrate_channel = 0;
        Serial.println("Channel 0 (A) Selected");
        break;
      case '1':
        calibrate_channel = 1;
        Serial.println("Channel 1 (B) Selected");
        break;
      case 'm':
        // Measurement mode
        mode = 0;
        Serial.println("Measurement Mode");
        printHeader();
        break;
      case 'c':
        // Calibration mode
        mode = 1;
        Serial.println("Calibration Mode");
        break;
      case 't':
        // Tare. Teset the scale to 0
        Serial.println("Taring");
        scales.power_up();
        delay(500);
        scales.tare(5);
        // Need to do this for each channel
        // and update our stored offset.
        Serial.println("Not properly Tared!");
        break;
      case 'h':
        printHeader();
        break;
      case 'z':
        zeroScales();
        break;  
      case 's':
        printSigfoxModelDetails();
        break;
      default:
        Serial.println("Unknown instruction. Select: 0, 1, m, c, t, h, z, or s");
        Serial.println("m - measurement mode");
        Serial.println("c - Calibration mode");
        Serial.println("0 - Channel 0 (A) Calibration");
        Serial.println("1 - Channel 1 (B) Calibration");
        Serial.println("t - Tare (scale)");
        Serial.println("z - Zero (Weight)");
        Serial.println("h - print Header");
        Serial.println("s - print Sigfox model details");
        break;
    }
  }
}

// Measure (and record) the temperature and humidity levels
// Sets alarms if out of rage (we can't use limits on Internet service side
// as the messages may only be sent a few times a day and a brief (maybe hours)
// out of range temperature/humidity could easily be missed between
// message publishing.
void measureTemperatureAndHumidity() {
  if (!bmeOk) {
    return;
  }

  currentTemperature = bme.readTemperature();
  if (currentTemperature < minTemperature) {
    temperatureAlarm = true;
  }

  if (currentTemperature > maxTemperature) {
    temperatureAlarm = true;
  }

  currentHumidity = bme.readHumidity();
  if (currentHumidity < minHumidity) {
    humidityAlarm = true;
  }

  if (currentHumidity > maxHumidity) {
    humidityAlarm = true;
  }
}

// Print the values read from the BME280 sensor
void printBmeValues() {
    //Serial.print("Temperature = ");
    Serial.print(bme.readTemperature(), 1);
    Serial.print("\t");

    Serial.print(bme.readPressure() / 100.0F, 0);
    Serial.print("\t\t");

    Serial.print(bme.readHumidity(),1);
    Serial.print("\t\t");
}

// =============================================================
// Sigfox handing
// =============================================================

// Determine if we should publish the Sigfox message.
// We may also wish to publish if the stock level has
// changed (or a significant weight level has changed)
// but we would need to be careful of exceeding the 
// 140 messages per day for a noisy system.
bool shouldPublish() {
  // Publish every 15 minutes
  // this doesn't really need to be this often
  // but whilst developing it helps keep an eye on the system.
  int messageIntervalMinutes = 15;
  
  // On first run after reset 
  // allow a 2 minute delay for the platform to be placed into 
  // the box and stabalise before doing first publish
  // which is also expected to include a check for zeroing the platform.
  if (isFirstCycle) {
    messageIntervalMinutes = 2;
    Serial.println("First cycle");
  }

  // How long ago we last publish a Sigfox message
  long millisAgo = millis() - lastPublish;

  return millisAgo > (messageIntervalMinutes * 60 * 1000);
}

// Publish our measurements (weight, temperature, humidity etc)
// to Sigfox.
void publishMeasurements() {
  Serial.println("Sending via Sigfox...");  
  bool useDownlink = shouldUseDownlink();
  if (useDownlink) {
    Serial.println("Using Sigfox downlink...");    
  }

  // stub for message which will be sent
  SigfoxMessage msg = buildMessage();

  SigFox.begin();
  SigFox.debug();
  // Wait at least 30mS after first configuration (100mS before)
  delay(100);
  // Clears all pending interrupts
  SigFox.status();
  delay(1);

  SigFox.beginPacket();
  SigFox.write((uint8_t*)&msg, 12);
  // endPacket actually sends the data.
  uint8_t statusCode = SigFox.endPacket(useDownlink);

  printSigfoxStatus(statusCode);

  // Status = 0 for a successful send, otherwise indicates
  // a failure.
  // Store when we last published a Sigfox message
  // to allow for timed message sending.
  if (statusCode == 0) { 
    resetAlarms();
  }

  // Update the last publish/downlink times
  // even if an error resonse was received to prevent
  // repeated publishing
  lastPublish = millis();
  isFirstCycle = false;

  if (useDownlink) {
      parseDownlinkData(statusCode);
      lastDownlink = lastPublish;
  }

  // Store the status value
  lastSigfoxStatus = statusCode;
  SigFox.end();
}

void printSigfoxStatus(uint8_t statusCode) {
  Serial.print("Response status code : 0x");    
  Serial.println(statusCode, HEX);    

  if (statusCode != 0) {
    Serial.print("Sigfox Status:");    
    Serial1.println(SigFox.status(SIGFOX));
    Serial1.println();
  
    Serial.print("Atmel Status:");    
    Serial1.println(SigFox.status(ATMEL));
    Serial1.println();
  }
}

// Create the message to be publish to Sigfox.
SigfoxMessage buildMessage() {
  SigfoxMessage message;

  message.status = getStatusFlags();
  message.humidity = (int8_t )currentHumidity;
  message.temperature = (int8_t)currentTemperature;
  message.zeroWeight = (int16_t)zeroWeight;   
  message.weight = (int16_t)currentWeight;       
  message.itemCount = (int16_t)(stockLevel * 100);      
  message.driftCorrection = 0; // TODO
  message.filler = 0;
  message.lastStatus = lastSigfoxStatus;

  return message;
}

// Get the status flags for the Sigfox message.
byte getStatusFlags() {
  byte status = 0;

  // B7 - First run,
  // B6 - HX711 fault 
  // B5 - BME280 fault
  // B4 - Temperature alarm
  // B3 - Humidity alarm
  // B2 - weight alarm
  // B1 - Low stock
  // B0 - spare
  
  // Upper Nibble (Charging/Battery)
  // Battery flat
  if (isFirstCycle) {
    status |= 0x80; // 1000 0000
  }

  // HX711 fault.
  // we don't have a way to check this yet.
  if (!hx711Ok) {
    status |= 0x40; // 0100 0000
  }

  // BME280 fault
  if (!bmeOk) {
    status |= 0x20; // 0010 0000
  }

  // Over/Under temperature alarm
  if (temperatureAlarm > 0) {
    status |= 0x10; // 0001 0000
  }

  // Over/Under humidity alarm
  if (humidityAlarm) {
    status |= 0x08; // 0000 1000
  }

  // Over/under? weight alarm
  if (weightAlarm) {
    status |= 0x04; // 0000 0100
  }

  // if computed stock level low.
  if (lowStock) {
    status |= 0x02; // 0000 0010
  }

  return status;
}

// Determine if we are requesting a downlink message.
bool shouldUseDownlink() {
  // When debugging uncomment this so as to not keep requesting
  // downlink 
  //return false;
  
  // On first run we want to request a downlink 
  // message to help with zero'ing and setup.
  if (isFirstCycle) {
    return true;
  }

  // How long ago we last did a downlink message.
  long millisAgo = millis() - lastDownlink;

  // try every 12 hours, this keeps us under the 
  // maximum 4 per day.
  return millisAgo > (12 * 60 * 60 * 1000);
}

// Parse downlinked data.
void parseDownlinkData(uint8_t statusMessage) {
 
  if (statusMessage > 0) {
    Serial.println("No transmission. Status: " + String(statusMessage));
    return;
  }

  // Max response size is 8 bytes
  // set-up a empty buffer to store this. (0x00 == no action for us.)
  uint8_t response[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  // Expect...
  // Byte 0: Flags
  // B7: Zero scales
  // B6: Set Temperature range (ignore min/max temp if 0)
  // B5: Set Humidity range (ignore min/max humidity if 0)
  // B4: Set tolerance?
  // B3: Set ???
  // B2: Update unit weight (ignore update if 0)
  // B1: 
  // B0:
  // Byte 1: Min T
  // Byte 2: Max T
  // Byte 3: Min Humidity
  // byte 4: Max Humidity
  // byte 5: Read tolerence??? (+/- x)
  // byte 6 & 7: Unit weight

  // Parse the response packet from Sigfox
  if (SigFox.parsePacket()) {
    
    Serial.println("Response from server:");
    // Move the response into  local buffer.
    int i = 0;
    while (SigFox.available()) {
      Serial.print("0x");
      int readValue = SigFox.read();
      Serial.println(readValue, HEX);
      response[i] = (uint8_t)readValue;
      i++;
    }

    // byte 0 - flags.
    // 0b 1000 0000
    if (response[0] & 0x80 == 0x80) {
      zeroScales();
    }

    // 0b 0100 0000
    if (response[0] & 0x40 == 0x40) {
      updateTemperatureAlarm(response[1], response[2]);
    }

    // 0b 0010 0000
    if (response[0] & 0x20 == 0x20) {
      updateHumidityAlarm(response[3], response[4]);
    }

    // 0b 0000 0100
    if (response[0] & 0x04 == 0x04) {
      // Little Endian format. (ff dd -> 0xddff
      uint16_t weight = response[7] << 8 & response[6];
      updateUnitWeight(weight);
    }
  } else {
    Serial.println("No response from server");
  }
  Serial.println();
}

void printSigfoxModelDetails() {
  if (!SigFox.begin()) {
    Serial.println("Shield error or not present!");
    return;
  }
  
  // Output the ID and PAC needed to register the 
  // device at the Sigfox backend.
  String version = SigFox.SigVersion();
  String ID = SigFox.ID();
  String PAC = SigFox.PAC();

  // Display module informations
  Serial.println("MKRFox1200 Sigfox configuration");
  Serial.println("SigFox FW version " + version);
  Serial.println("ID  = " + ID);
  Serial.println("PAC = " + PAC);

  Serial.println("");

  Serial.print("Module temperature: ");
  Serial.println(SigFox.internalTemperature());

  Serial.println("Register your board on https://backend.sigfox.com/activate with provided ID and PAC");

  delay(100);

  // Send the module to the deepest sleep
  SigFox.end();
}

// =============================================================
// General helper methods
// =============================================================

// Reset the alarms after they have been published.
void resetAlarms() {
  temperatureAlarm = false;
  humidityAlarm = false;
  weightAlarm = false;
}

void zeroScales() {
  zeroWeight = lastAverage[0] + lastAverage[1];
  Serial.print("Zero'd: ");
  Serial.print(zeroWeight, 1);
  Serial.println();
}


void updateTemperatureAlarm(int8_t lower, int8_t upper) {
  Serial.print("Setting temperature alarm. Min: ");
  Serial.print(lower);
  Serial.print(", Max: ");
  Serial.println(upper);
  
  minTemperature = lower;
  maxTemperature = upper;
}

void updateHumidityAlarm(int8_t lower, int8_t upper) {
  Serial.print("Setting humidity alarm. Min: ");
  Serial.print(lower);
  Serial.print(", Max: ");
  Serial.println(upper);
  
  minHumidity = lower;
  maxHumidity = upper;
}

void updateUnitWeight(uint16_t weight) {
  Serial.print("Setting unit weight: ");
  Serial.println(weight);
  unitWeight = weight;
}
