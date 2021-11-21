/**
 * Pulse counting using 3 GPIOs with BLE advertisments every 30 seconds.
 *
 * We don't use the Pulse Counter peripherals as they run on the APB clock at 80MHz
 * and only have a 10-bit filter for short pulses; 1023 cycles - still less than 1ms.
 *
 * Instead we use interrupts.  We would check on just one edge, but that does not
 * seem reliable on the ESP32, at least using the Arduino functions, so we interrupt
 * on every edge and will divide our count by two.
 *
 * To filter out bounce we only count if there wasn't a previous edge within the last
 * FILTER_MS ms.
 */

#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEBeacon.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// So we can flash the blue LED from time to time
#define LED_BUILTIN   2

// Filter out edge detections if the previous were in this number of milliseconds
// Photo-resistor pulses are to pick up very brief pulses from an LED on an electricity
// meter, so our filter needs to be much shorter than for the movement of a magnet
// past a reed switch as on gas/water meters.
#define FILTER_MS_REED  300
#define FILTER_MS_PHOTO 25

// Our bluetooth driver instantiation
BLEAdvertising *pAdvertising;

// 3 pulse counts initially 0
static volatile uint32_t edgeCounter[3] = {0, 0, 0};
static volatile uint32_t lastMillis[3] = {0, 0, 0};
// Set the filters for our 3 pulse counters
static const uint32_t FILTER_MS[3] = {FILTER_MS_PHOTO, FILTER_MS_REED, FILTER_MS_REED};
static unsigned int advertCount = 0;
static BLEAdvertisementData oAdvertisementData;

// Button press - serial output
void printCounters()
{
  String out = "";
  for (int i = 0; i < 3; i++)
  {
    // Our edgeCounters are divided by 2 (right-shifted by 1) as interrupts don't seem
    // to be able to tell the difference between rising and falling edges on the ESP32.
    out += "Counter " + String(i) + ": " + String(edgeCounter[i] >> 1) + "\t";
  }
  Serial.println(out);
}

// Send a BLE advertisement
void bleAdvertise()
{
  // LED on
  digitalWrite(LED_BUILTIN, HIGH);

  // Update data as needed and briefly advertise
  updateAdvertData();
  advertCount++;
  pAdvertising->start();
  delay(1000);
  // FIXME: Advertising seems not to stop when requested!!
  pAdvertising->stop();

  // LED off
  digitalWrite(LED_BUILTIN, LOW);
}

// Button GPIO interrupt service routine
void IRAM_ATTR buttonISR()
{
  // Better for this not to be in an interrupt routine as it will hold up other interrupts!
	//printCounters();
}

// Counter GPIO interrupt service routine
void IRAM_ATTR counterISR(void* arg)
{
  int counterNum = (int)arg;
  // Only increment the edgeCounter if the last detected edge was more than FILTER_MS ms ago
  uint32_t curMillis = millis();
  if (curMillis - lastMillis[counterNum] > FILTER_MS[counterNum])
  {
      edgeCounter[counterNum]++;
  }
  lastMillis[counterNum] = curMillis;
}

void setup()
{
  // GPIO setup for blue LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Serial debug setup
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  // Print out clocks
  // Doesn't find these in the libs for some reason
  //Serial.printf("CPU clock: %d MHz", esp_clk_cpu_freq() / 1000000);
  //Serial.printf("APB clock: %d MHz", esp_clk_apb_freq() / 1000000);

  // GPIO setup for Boot button
  pinMode(0, INPUT_PULLUP);
  //attachInterrupt(0, buttonISR, RISING);

  // GPIO setup for pulse edgeCounter inputs
  pinMode(34, INPUT);
  attachInterruptArg(34, counterISR, (void*)0, CHANGE);

  pinMode(35, INPUT);
  attachInterruptArg(35, counterISR, (void*)1, CHANGE);

  pinMode(36, INPUT);
  attachInterruptArg(36, counterISR, (void*)2, CHANGE);

  // BLE init
  BLEDevice::init("");
  pAdvertising = BLEDevice::getAdvertising();
  setupInitialAdvertAndScanData();
}

// Advertising service data structure excluding length and UUID fields
struct AdvertData
{
/* uint8_t length;
  uint8_t uuidLengthInBits; // 16 for us
  uint16_t uuid;*/
  uint8_t macAddress[6];  // Why do we need the MAC address in the data both forwards and backwards?  Remove?
  uint8_t framePacketCounter;
  uint8_t counterMarker0;
  uint32_t counter0;
  uint8_t counterMarker1;
  uint32_t counter1;
  uint8_t counterMarker2;
  uint32_t counter2;
} __attribute__((packed));

#define HOMEBREW_UUID         ((uint16_t)0x191a)

/**
 * Update the counter values in the advertising data
 */
void updateAdvertData()
{
  static AdvertData* advertData = initialAdvertData();
  static BLEUUID homebrewCounterUUID = BLEUUID(HOMEBREW_UUID);
  advertData->counter0 = (edgeCounter[0] >> 1);
  advertData->counter1 = (edgeCounter[1] >> 1);
  advertData->counter2 = (edgeCounter[2] >> 1);
  advertData->framePacketCounter = advertCount;

  std::string advertDataStr = std::string((char*)advertData, sizeof(struct AdvertData));

  oAdvertisementData = BLEAdvertisementData();
  //oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04

  Serial.println("Adding service data...");
  oAdvertisementData.setServiceData(homebrewCounterUUID, advertDataStr);

  std::string debug = oAdvertisementData.getPayload();
  Serial.printf("Advertising payload is %d bytes\n", debug.length());
  printStringAsHex(debug);

  Serial.println("Adding name...");
  oAdvertisementData.setName("PulseCounter");

  debug = oAdvertisementData.getPayload();
  Serial.printf("Advertising payload is %d bytes\n", debug.length());
  printStringAsHex(debug);

  pAdvertising->setAdvertisementData(oAdvertisementData);
}

void printStringAsHex(std::string s)
{
  String hex;
  for (int i = 0; i < s.length(); i++)
  {
    char hexVal[5];
    sprintf(hexVal, "0x%02x", s[i]);
    hex += String(hexVal) + " ";
  }
  Serial.println(hex);
}

struct AdvertData* initialAdvertData() {
  /*
   *  From https://github.com/atc1441/ATC_MiThermometer/blob/master/README.md, modified for 3x counter values
   *
   *  Advertising format of the custom firmware:
   *  The custom firmware sends advertising data on the UUID 0x191A with counter data.
   *
   *  The format of the advertising data is as follows:
   *  Byte 0 Length
   *  Byte 1 UUID length (bits)
   *  Byte 2-3 UUID
   *  Byte 4-9 MAC in correct order
   *  Byte 10Frame packet counter
   *  Byte 11 Counter data tag
   *  Byte 12-15 Counter 1 value (int32)
   *  Byte 16 Counter data tag
   *  Byte 17-20 Counter 2 value (int32)
   *  Byte 21 Counter data tag
   *  Byte 22-25 Counter 3 value (int32)
   *  Example: 0x0e,
   *           0x16,
   *           0x1a, 0x18,
   *           0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   *           0xee
   *           0x01,
   *           0xaa, 0xaa, 0xaa, 0xaa,
   *           0x01,
   *           0xbb, 0xbb, 0xbb, 0xbb,
   *           0x01,
   *           0xcc, 0xcc, 0xcc, 0xcc,
   */

  // We start at the MAC address as setServiceData() will put in the length and UUID fields
  AdvertData* data = new AdvertData();
  memcpy(data->macAddress, BLEDevice::getAddress().getNative(), 6);
  Serial.printf("MAC address is: %02x:%02x:%02x:%02x:%02x:%02x\n",
                data->macAddress[0], data->macAddress[1], data->macAddress[2],
                data->macAddress[3], data->macAddress[4], data->macAddress[5]
               );
  data->framePacketCounter = 0;
  data->counterMarker0 = 0x1;
  data->counterMarker1 = 0x1;
  data->counterMarker2 = 0x1;
  data->counter0 = 0;
  data->counter1 = 0;
  data->counter2 = 0;

  return data;
}

void setupInitialAdvertAndScanData()
{
  BLEAdvertisementData oScanResponseData = BLEAdvertisementData();

  oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04

  BLEUUID homebrewCounterUUID = BLEUUID(HOMEBREW_UUID);
  std::string strServiceData = std::string((char*)initialAdvertData(), sizeof(AdvertData));

  oAdvertisementData.setServiceData(homebrewCounterUUID, strServiceData);

  oAdvertisementData.setName("PulseCounter");

  pAdvertising->setAdvertisementData(oAdvertisementData);

  // Nothing in the scan response data
  pAdvertising->setScanResponseData(oScanResponseData);
}

void loop()
{
  // Handle Boot button presses
  static uint8_t lastPinState = 1;
  uint8_t pinState = digitalRead(0);
  if(!pinState && lastPinState){
	  printCounters();
  }
  lastPinState = pinState;
  // Advertise every 30 seconds - we should use a timer for this so the
  // button state is polled more regularly or use an interrupt for the
  // button.
  bleAdvertise();
  delay(30 * 1000);
}
