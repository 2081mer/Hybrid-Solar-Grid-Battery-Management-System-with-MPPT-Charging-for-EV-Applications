// Reference copy of what the Arduino Cloud generates for the new variables.
// The Cloud regenerates this file when you change variables in the Thing.
// If yours differs from this, trust the one shown in the Thing's Sketch tab.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "arduino_secrets.h"

const char DEVICE_LOGIN_NAME[]  = "YOUR_SSID";

const char SSID[]               = SECRET_SSID;             // Network SSID (name)
const char PASS[]               = SECRET_OPTIONAL_PASS;    // Network password
const char DEVICE_KEY[]         = SECRET_DEVICE_KEY;       // Secret device password

void onAutoChargeChange();

float soc;
float temp;
float iout;
float vpack;
bool autoCharge;

void initProperties(){

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(soc,   READ, 5 * SECONDS, NULL);
  ArduinoCloud.addProperty(temp,  READ, 5 * SECONDS, NULL);
  ArduinoCloud.addProperty(iout,  READ, 5 * SECONDS, NULL);
  ArduinoCloud.addProperty(vpack, READ, 5 * SECONDS, NULL);
  ArduinoCloud.addProperty(autoCharge, READWRITE, ON_CHANGE, onAutoChargeChange);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SECRET_SSID, SECRET_OPTIONAL_PASS);
