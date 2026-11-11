#pragma once
// ===================== SapulMRF Kiosk – settings =====================

// --- Network (same idea as the SapulMRF stream: the kiosk is the server,
//     the PC script connects to the IP shown on the welcome screen) ---
// The kiosk tries these networks in order: your Wi-Fi, then a phone hotspot.
// Leave an entry as "" to skip it.
#define WIFI1_SSID     "YOUR_WIFI"
#define WIFI1_PASS     "YOUR_PASSWORD"
#define WIFI2_SSID     "YOUR_PHONE_HOTSPOT"
#define WIFI2_PASS     "HOTSPOT_PASSWORD"
#define WIFI_TRY_MS    10000          // per network

// If neither is found, the kiosk makes its OWN hotspot. Connect the PC to it;
// the kiosk is then always 192.168.4.1.
#define AP_SSID        "SapulMRF-Kiosk"
#define AP_PASS        "sapulmrf123"  // at least 8 characters

#define STREAM_PORT    5000           // TCP, PC connects here (as in sapul stream)
#define BEACON_PORT    5001           // UDP "I'm here" so the PC finds the kiosk by itself
#define MAX_PACKET     65536          // biggest JPEG/JSON the kiosk accepts

// --- Battery indicator (INA219, wired as in the Alangan translator) ---
#define BATTERY_USE_INA219     1      // 0 = no battery, hides the icon
#define INA219_ADDR            0x40   // 0x41/0x44/0x45 if A0/A1 are bridged
#define INA219_SHUNT_OHM       0.1f   // R100 shunt on common modules
#define BATTERY_INTERNAL_MOHM  150    // cell + wiring; corrects sag under load
#define BATTERY_EMPTY_MV       3300
#define BATTERY_FULL_MV        4150
#define BATTERY_CHARGE_MA      -40    // current below this = charging (VIN+ on battery)
#define BATTERY_LOW_PCT        15     // turns red below this

// --- Video box (must match FRAME_W/FRAME_H in kiosk_bridge.py) ---
#define FRAME_W 480
#define FRAME_H 360

// --- Kiosk behaviour ---
#define RECEIPT_RETURN_S   10     // receipt screen -> welcome
#define IDLE_RETURN_MS     90000  // any screen idle this long -> welcome
#define NUM_PUROKS         7
#define KIOSK_PIN          "1234"  // operator PIN used when the camera PC is offline
                                   // (when the PC is connected, its OPERATOR_PINS decide)

// --- Optional load-cell scale (HX711). 0 = weight entered on the keypad ---
#define USE_HX711          0
#define HX711_DOUT         17     // check which pins your board breaks out (P2/P3 headers)
#define HX711_SCK          18
#define HX711_SCALE        -7050.0f  // calibrate with a known weight

// --- Sunton ESP32-8048S070 (7" 800x480 RGB, GT911 capacitive touch) ---
// Confirm against the label on the back of your board; most 7" ESP32-S3 CYDs use these.
#define TFT_BL   2
#define TFT_DE   41
#define TFT_VSYNC 40
#define TFT_HSYNC 39
#define TFT_PCLK 42
#define TFT_R0 14
#define TFT_R1 21
#define TFT_R2 47
#define TFT_R3 48
#define TFT_R4 45
#define TFT_G0 9
#define TFT_G1 46
#define TFT_G2 3
#define TFT_G3 8
#define TFT_G4 16
#define TFT_G5 1
#define TFT_B0 15
#define TFT_B1 7
#define TFT_B2 6
#define TFT_B3 5
#define TFT_B4 4

#define TOUCH_SDA 19   // shared I2C bus: GT911 touch + INA219 (P3 connector)
#define TOUCH_SCL 20
#define TOUCH_INT -1
#define TOUCH_RST 38

// Screen brightness, 5-100 %. Lower = dimmer, cooler and longer battery life.
#define BACKLIGHT_PCT 80
#define BACKLIGHT_PWM 1      // 0 = backlight fully on (no dimming), use if the screen stays dark
#define DISPLAY_TEST  0      // 1 = flash red/green/blue/white at boot to check the panel

// Lower this (e.g. 12000000) if the screen flickers while Wi-Fi is streaming
#define RGB_PCLK_HZ 14000000
