#include <Adafruit_TinyUSB.h>
#include "TLx493D_inc.hpp"
#include <SimpleKalmanFilter.h>
#include <OneButton.h>

//--------------------------------------------------------------------+
// MSC RAM Disk Config
//--------------------------------------------------------------------+

Adafruit_USBD_MSC usb_msc;

enum {
  RID_KEYBOARD = 1,
  RID_MOUSE,
};

uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(RID_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE   (HID_REPORT_ID(RID_MOUSE))
};

// USB HID object
Adafruit_USBD_HID usb_hid;

using namespace ifx::tlx493d;

/* Definition of the power pin and sensor objects for Kit2Go XMC1100 boards. */
const uint8_t POWER_PIN = 15; // XMC1100 : LED2

// some board swith multiple I2C need Wire --> Wire1
TLx493D_A1B6 mag(Wire1, TLx493D_IIC_ADDR_A0_e);

//Tlv493d mag = Tlv493d();
SimpleKalmanFilter xFilter(1, 1, 0.2), yFilter(1, 1, 0.2), zFilter(1, 1, 0.2);

// Setup buttons
OneButton button1(27, true);
OneButton button2(24, true);

float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;

int calSamples = 50;
int sensivity = 8;
int magRange = 3;
int outRange = 127;      // Max allowed in HID report
float xyThreshold = 0.4; // Center threshold

int inRange = magRange * sensivity;
float zThreshold = xyThreshold * 2.5;

bool isOrbit = false;
bool middle = false;

uint8_t key_none[6] = {HID_KEY_NONE};
uint8_t key_h[6] = {HID_KEY_H};

/** Definition of a counter variable. */
uint8_t count = 0;

void calibrate() {
  double x, y, z;

  Wire1.begin();

  // mag sensor init
  mag.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 0, 250000);
  mag.begin();

  for (int i = 1; i <= calSamples; i++)
  {
    mag.getMagneticField(&x, &y, &z);

    xOffset += x;
    yOffset += y;
    zOffset += z;
  }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;

}

// the setup routine runs once when you press reset:
void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  usb_msc.begin();

  // Set up HID
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.setBootProtocol(HID_ITF_PROTOCOL_NONE);
  usb_hid.setPollInterval(2);
  usb_hid.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  Serial.begin(115200);
  calibrate();

  button1.attachClick(btn1);
  button2.attachClick(btn2);
}

void process_hid(int x, int y) {
  /*------------- Mouse -------------*/
  if (usb_hid.ready()) {
    usb_hid.mouseButtonPress(RID_MOUSE, MOUSE_BUTTON_MIDDLE);
    middle = true;
    delay(20);
    usb_hid.mouseReport(RID_MOUSE, MOUSE_BUTTON_MIDDLE, x, y, 0, 0 );
  }
}

void getMagnet() {
  double x, y, z;
  mag.getMagneticField(&x, &y, &z);

  xCurrent = xFilter.updateEstimate(x - xOffset);
  yCurrent = yFilter.updateEstimate(y - yOffset);
  zCurrent = zFilter.updateEstimate(z - zOffset);
  
  static uint32_t ms = 0;
  static uint32_t ms2 = 0;

  // check the center threshold
  if (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold)
  {

    int xMove = 0;
    int yMove = 0;

    // map the magnetometer xy to the allowed 127 range in HID repports
    xMove = map(xCurrent, -inRange, inRange, -outRange, outRange);
    yMove = map(yCurrent, -inRange, inRange, -outRange, outRange);

    // press shift to orbit in Fusion 360 if the pan threshold is not corssed (zAxis)
    if (abs(zCurrent) > zThreshold)
    {
      usb_hid.keyboardReport(RID_KEYBOARD, KEYBOARD_MODIFIER_LEFTSHIFT, key_none);
      isOrbit = true;
      ms2 = millis();
    }

    // pan or orbit by holding the middle mouse button and moving propotionaly to the xy axis
    ms = millis();
    process_hid(xMove, -yMove);
  }
  else
  {
    // release the mouse and keyboard if within the center threshold
    if (millis() - ms > 200 && middle ) {
      usb_hid.mouseButtonRelease(RID_MOUSE);
      middle = false;
      delay(10);
    }
    if (millis() - ms2  > 200 && isOrbit) {
      usb_hid.keyboardRelease(RID_KEYBOARD);
      isOrbit = false;
      delay(10);
    }
  }

  Serial.print(xCurrent);
  Serial.print(",");
  Serial.print(yCurrent);
  Serial.print(",");
  Serial.print(zCurrent);
  Serial.println();
} 

// the loop routine runs over and over again forever:
void loop() {
  // keep watching the push buttons
  button1.tick();
  button2.tick();
  
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }
  getMagnet();
}

void btn1()
{
  usb_hid.keyboardReport(RID_KEYBOARD, KEYBOARD_MODIFIER_LEFTSHIFT+KEYBOARD_MODIFIER_LEFTGUI, key_h);
  delay(10);
  usb_hid.keyboardRelease(RID_KEYBOARD);
}

void btn2()
{
  usb_hid.mouseButtonPress(RID_MOUSE, MOUSE_BUTTON_MIDDLE);
  delay(10);
  usb_hid.mouseButtonRelease(RID_MOUSE);
  delay(10);
  usb_hid.mouseButtonPress(RID_MOUSE, MOUSE_BUTTON_MIDDLE);
  delay(10);
  usb_hid.mouseButtonRelease(RID_MOUSE);
}
