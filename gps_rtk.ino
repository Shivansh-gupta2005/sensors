#include <SparkFun_u-blox_Arduino_Library.h>

// Define the serial port for communication with the NEO-M8P-2
SFE_UBLOX_GPS myGPS;

void setup() {
  Serial.begin(115200); // Serial monitor for debugging
  Serial1.begin(9600);  // Default baud rate for NEO-M8P-2

  // Initialize the NEO-M8P-2 module
  if (myGPS.begin(Serial1) == false) {
    Serial.println(F("u-blox GPS not detected. Please check wiring."));
    while (1);
  }

  // Enable RTCM messages for RTK
  myGPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART1, 1); // Example RTCM message
  myGPS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART1, 1); // Another common RTCM message

  // Configure the module to receive RTCM data
  myGPS.setPortInput(COM_PORT_UART1, true); // Enable UART1 for RTCM input

  // Set the update rate for RTK data
  myGPS.setNavigationFrequency(5); // Set to 5Hz for RTK

  // Set the NEO-M8P-2 to RTK mode
  uint8_t setRTK[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  myGPS.sendCommand(setRTK, sizeof(setRTK));

  // Save configuration to flash
  uint8_t saveConfig[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  myGPS.sendCommand(saveConfig, sizeof(saveConfig));
}

void loop() {
  // Read and print RTK data
  if (myGPS.available()) {
    if (myGPS.newNMEAreceived()) {
      if (!myGPS.parse(Serial1)) {
        Serial.println(F("Failed to parse GPS"));
      } else {
        Serial.print(F("Latitude: "));
        Serial.print(myGPS.getLatitude(), 5);
        Serial.print(F(", Longitude: "));
        Serial.print(myGPS.getLongitude(), 5);
        Serial.print(F(", Altitude: "));
        Serial.print(myGPS.getAltitude());
        Serial.println();
      }
    }
  }

  delay(100); // Adjust based on your update rate
}
