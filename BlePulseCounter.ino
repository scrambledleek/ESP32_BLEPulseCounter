/**
 * Pulse counting using 3 GPIOs with BLE advertisments every 30 seconds.
 * HA BLE format is used for Bluetooth advertisements.
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

#include "NimBLEDevice.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

// So we can flash the blue LED from time to time
#define GPIO_LED_BUILTIN   2

// Filter out edge detections if the previous were in this number of milliseconds
// Photo-resistor pulses are to pick up very brief pulses from an LED on an electricity
// meter, so our filter needs to be much shorter than for the movement of a magnet
// passing a reed switch as on gas/water meters.
#define FILTER_MS_REED  300
#define FILTER_MS_PHOTO 25

// Our bluetooth driver instantiation
BLEAdvertising *pAdvertising;

// 3 pulse counts (initially 0)
static volatile uint32_t edgeCounter[3] = {0, 0, 0};
static volatile uint32_t lastMillis[3] = {0, 0, 0};
// Set the filters for our 3 pulse counters
static const uint32_t FILTER_MS[3] = {FILTER_MS_PHOTO, FILTER_MS_REED, FILTER_MS_REED};
// Stores the counter value to put in adverts
static unsigned int advertCount = 0;
// Data for adverts
static BLEAdvertisementData oAdvertisementData;
// Request flag for printing of counter values
static volatile bool reqPrintCounters = false;

// Print counters via serial port (e.g. for debug, on button press)
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

void ledOff(NimBLEAdvertising* unused)
{
  digitalWrite(GPIO_LED_BUILTIN, LOW);
}

// Send a BLE advertisement
void bleAdvertise()
{
  // LED on
  digitalWrite(GPIO_LED_BUILTIN, HIGH);

  // Update data as needed and briefly advertise
  updateAdvertData();
  advertCount++;
  pAdvertising->start(3, &ledOff); // Advertise for 3 seconds and turn off LED when advertising complete (callback)
}

// Button GPIO interrupt service routine
void IRAM_ATTR buttonISR()
{
  // Better for actual printing not to be in an interrupt routine as it would hold up other interrupts.
  // Instead we flag that a print of the counters has been requested for the next loop iteration.
  reqPrintCounters = true;
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
  pinMode(GPIO_LED_BUILTIN, OUTPUT);

  // Serial debug setup
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  // Print out clocks
  // Doesn't find these in the libs for some reason
  //Serial.printf("CPU clock: %d MHz", esp_clk_cpu_freq() / 1000000);
  //Serial.printf("APB clock: %d MHz", esp_clk_apb_freq() / 1000000);

  // GPIO setup for Boot button
  pinMode(0, INPUT_PULLUP);
  attachInterrupt(0, buttonISR, RISING);

  // GPIO setup for pulse edgeCounter inputs
  pinMode(34, INPUT);
  attachInterruptArg(34, counterISR, (void*)0, CHANGE);

  pinMode(35, INPUT);
  attachInterruptArg(35, counterISR, (void*)1, CHANGE);

  pinMode(36, INPUT);
  attachInterruptArg(36, counterISR, (void*)2, CHANGE);

  // BLE init
  BLEDevice::init("");

  const uint8_t* macAddress = BLEDevice::getAddress().getNative();
  Serial.printf("MAC address is: %02x:%02x:%02x:%02x:%02x:%02x\n",
                macAddress[0], macAddress[1], macAddress[2],
                macAddress[3], macAddress[4], macAddress[5]
               );

  pAdvertising = BLEDevice::getAdvertising();
  setupInitialAdvertAndScanData();
}

// Advertising service data structure excluding length and UUID fields
struct AdvertData
{
/* uint8_t length;
  uint8_t uuidLengthInBits; // 16 for us
  uint16_t uuid;*/
  uint16_t packetIDMarker;
  uint8_t packetID;
  uint16_t counterMarker0;
  uint32_t counter0;
  uint16_t counterMarker1;
  uint32_t counter1;
  uint16_t counterMarker2;
  uint32_t counter2;
} __attribute__((packed));

#define HA_BLE_UNENCRYPTED_UUID  ((uint16_t)0x181c)

/**
 * Update the counter values in the advertising data
 */
void updateAdvertData()
{
  static AdvertData* advertData = initialAdvertData();
  static BLEUUID haBLEUnencryptedUUID = BLEUUID(HA_BLE_UNENCRYPTED_UUID);
  advertData->counter0 = (edgeCounter[0] >> 1);
  advertData->counter1 = (edgeCounter[1] >> 1);
  advertData->counter2 = (edgeCounter[2] >> 1);
  advertData->packetID = advertCount;

  std::string advertDataStr = std::string((char*)advertData, sizeof(struct AdvertData));

  oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x06); // LE_GENERAL_DISCOVERABLE_MODE 0x02 || BR_EDR_NOT_SUPPORTED 0x04

  Serial.println("Adding service data...");
  oAdvertisementData.setServiceData(haBLEUnencryptedUUID, advertDataStr);

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
   * HA BLE advertising data format as described here:
   *   https://custom-components.github.io/ble_monitor/ha_ble#diy-sensors-ha-ble
   *
   * Using 1x packedID and 3x 32-bit counters in the payload.
   * The 'markers' are in fact a byte length and date type, hard-coded as numbers here for now.
   */

  // We start at the MAC address as setServiceData() will put in the length and UUID fields
  AdvertData* data = new AdvertData();
  data->packetIDMarker = 0x0002;
  data->packetID = 0;
  data->counterMarker0 = 0x0905;
  data->counterMarker1 = 0x0905;
  data->counterMarker2 = 0x0905;
  data->counter0 = 0;
  data->counter1 = 0;
  data->counter2 = 0;

  return data;
}

void setupInitialAdvertAndScanData()
{
  BLEAdvertisementData oScanResponseData = BLEAdvertisementData();

  oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x06); // BLE_HS_ADV_F_DISC_GEN || BLE_HS_ADV_F_BREDR_UNSUP

  BLEUUID haBLEUnencryptedUUID = BLEUUID(HA_BLE_UNENCRYPTED_UUID);
  std::string strServiceData = std::string((char*)initialAdvertData(), sizeof(AdvertData));

  oAdvertisementData.setServiceData(haBLEUnencryptedUUID, strServiceData);

  oAdvertisementData.setName("PulseCounter");

  pAdvertising->setAdvertisementData(oAdvertisementData);

  // Nothing in the scan response data
  pAdvertising->setScanResponseData(oScanResponseData);
}

void loop()
{
  // Handle request to print counters
  if (reqPrintCounters)
  {
    reqPrintCounters = false;
	  printCounters();
  }

  // Advertise every 30 seconds - we should use a timer for this so there aren't such long
  // delays between printing counters if there are requests to do so, but given that's just
  // for debug it's not too important.
  bleAdvertise();
  delay(30 * 1000);
}
