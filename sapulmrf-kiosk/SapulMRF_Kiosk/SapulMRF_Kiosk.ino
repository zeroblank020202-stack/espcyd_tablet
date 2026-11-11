/*
  SapulMRF Kiosk – ESP32-S3 7" (800x480) touch board
  Resident drop-off:  Welcome -> Camera scan -> Weigh & confirm -> Receipt
  Operator mode:      Lock icon -> PIN -> Operator station (recent intake)

  The PC runs YOLO on the USB camera (pc_bridge/kiosk_bridge.py). Like the
  SapulMRF stream, the KIOSK is the TCP server (port 5000) and the PC connects
  to the IP shown on the welcome screen. The kiosk joins your Wi-Fi or a phone
  hotspot; if neither is found it makes its own hotspot (192.168.4.1).

  Packets, both directions:  [1 byte type][4 byte little-endian length][payload]
    'J' PC->kiosk  JPEG frame          'A' kiosk->PC  ACK, frame shown (send next)
    'M' either way JSON message (det, receipt, stats, auth, recent / mode, intake, pin...)

  Battery: INA219 on the shared I2C bus (GPIO19/20), same wiring as the
  Alangan translator. Icon + % top-right on every screen.

  Arduino IDE settings (Tools menu):
    Board: ESP32S3 Dev Module · PSRAM: OPI PSRAM · Flash: 16MB (QIO)
    Partition: Huge APP (3MB No OTA) · USB CDC On Boot: Enabled

  Libraries (Library Manager):
    GFX Library for Arduino (Moon On Our Nation) 1.6.x, with ESP32 core 3.x
      (on ESP32 core 2.0.x use GFX Library 1.4.9 instead)
    lvgl 8.3.x            (copy lv_conf.h next to the lvgl folder; see README)
    TAMC_GT911
    JPEGDEC (bitbank2)
    ArduinoJson 7.x
    (Wi-Fi, TCP, UDP and the INA219 use only built-in ESP32 libraries)
    HX711 (bogde)         only if USE_HX711 = 1
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <JPEGDEC.h>
#include <Arduino_GFX_Library.h>
#include <TAMC_GT911.h>
#include <lvgl.h>
#include "config.h"
#if USE_HX711
#include <HX711.h>
HX711 scale;
#endif

// ------------------------------------------------------------ colours (match the design)
#define C_GREEN_DARK 0x1E5233
#define C_GREEN      0x2E7D4F
#define C_GREEN_SOFT 0xDCEFE2
#define C_GREEN_TXT  0xC9DDD0
#define C_GREEN_OK   0x9FE0B5
#define C_BG         0xF4F5F2
#define C_CARD       0xFFFFFF
#define C_LINE       0xD5DBD3
#define C_MUTED_BG   0xE4E8E2
#define C_TEXT       0x1A1F1C
#define C_TEXT2      0x4A544D
#define C_AMBER_BG   0xFFF3E0
#define C_AMBER_TXT  0x8A4B08
#define C_RED        0xC62828

// ------------------------------------------------------------ display + touch
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK,
  TFT_R0, TFT_R1, TFT_R2, TFT_R3, TFT_R4,
  TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5,
  TFT_B0, TFT_B1, TFT_B2, TFT_B3, TFT_B4,
  0 /*hsync pol*/, 180 /*hfp*/, 30 /*hpw*/, 16 /*hbp*/,
  0 /*vsync pol*/, 12 /*vfp*/, 13 /*vpw*/, 10 /*vbp*/,
  1 /*pclk active neg*/, RGB_PCLK_HZ);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, rgbpanel, 0, true);
TAMC_GT911 tp(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, 800, 480);

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *lv_buf1, *lv_buf2;
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;

void my_flush(lv_disp_drv_t *d, const lv_area_t *a, lv_color_t *p) {
  uint32_t w = a->x2 - a->x1 + 1, h = a->y2 - a->y1 + 1;
  gfx->draw16bitRGBBitmap(a->x1, a->y1, (uint16_t *)&p->full, w, h);
  lv_disp_flush_ready(d);
}

void my_touch(lv_indev_drv_t *, lv_indev_data_t *data) {
  tp.read();
  if (tp.isTouched) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = tp.points[0].x;
    data->point.y = tp.points[0].y;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

// ------------------------------------------------------------ app state
WiFiServer server(STREAM_PORT);
WiFiClient pc;
WiFiUDP beacon;
JPEGDEC jpeg;
bool pcUp = false;
bool apMode = false;
String netName = "", netIp = "";

// incoming packet state machine (never blocks the UI)
uint8_t rxHdr[5]; int rxHdrGot = 0;
uint8_t *rxBuf = nullptr; uint32_t rxLen = 0, rxGot = 0; char rxType = 0;

// battery
bool batFound = false;
float batPct = -1, batMv = 0, batMa = 0;
lv_obj_t *batPill, *lblBat, *lblNet, *lblIp;

uint16_t *frameBuf = nullptr;            // RGB565 video frame in PSRAM
lv_img_dsc_t frameDsc;

int detRecyc = 0, detResid = 0, detBio = 0, detFps = 0;
bool detStable = false;
String category = "Mixed household";
int purok = 0;                           // 0 = not chosen
float weightKg = 0;
bool weightManual = USE_HX711 ? false : true;
String pinEntry = "";
uint32_t pinAskedAt = 0;   // waiting for the PC's answer to a PIN
String weightEntry = "";
int receiptCountdown = 0;

// screens
lv_obj_t *scrHome, *scrScan, *scrWeigh, *scrReceipt, *scrPin, *scrOp;
// widgets that change
lv_obj_t *lblTodayKg, *lblTodayDrops, *lblCamStatus;
lv_obj_t *imgVideo, *lblVideoWait, *lblFps, *lblRecyc, *lblResid, *lblBio, *lblHint, *btnNextScan;
lv_obj_t *lblWeight, *lblWeightState, *btnCat[4], *btnPurok[NUM_PUROKS], *btnConfirm, *kpWeight;
lv_obj_t *lblCode, *lblRcWeight, *lblRcCat, *lblRcPurok, *lblRcRecyc, *lblCountdown;
lv_obj_t *pinDots[4], *lblPinMsg;
lv_obj_t *tblRecent;
lv_timer_t *receiptTimer = nullptr;

const char *CATS[4] = {"Mixed household", "Recyclable", "Biodegradable", "Residual"};

// ------------------------------------------------------------ small UI helpers
static lv_obj_t *box(lv_obj_t *parent, int w, int h, uint32_t bg) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_remove_style_all(o);
  lv_obj_set_size(o, w, h);
  lv_obj_set_style_bg_color(o, lv_color_hex(bg), 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}

static lv_obj_t *label(lv_obj_t *parent, const char *txt, const lv_font_t *f, uint32_t color) {
  lv_obj_t *l = lv_label_create(parent);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, f, 0);
  lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
  return l;
}

static lv_obj_t *button(lv_obj_t *parent, const char *txt, int w, int h, uint32_t bg, uint32_t fg,
                        const lv_font_t *f, lv_event_cb_t cb, void *user = nullptr) {
  lv_obj_t *b = lv_btn_create(parent);
  lv_obj_set_size(b, w, h);
  lv_obj_set_style_radius(b, 12, 0);
  lv_obj_set_style_shadow_width(b, 0, 0);
  lv_obj_set_style_bg_color(b, lv_color_hex(bg), 0);
  lv_obj_set_style_bg_color(b, lv_color_hex(bg), LV_STATE_DISABLED);
  lv_obj_set_style_bg_opa(b, LV_OPA_40, LV_STATE_DISABLED);
  lv_obj_t *l = label(b, txt, f, fg);
  lv_obj_center(l);
  if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, user);
  return b;
}

static void setChoice(lv_obj_t *b, bool on) {     // outlined "chip" buttons
  lv_obj_set_style_bg_color(b, lv_color_hex(on ? C_GREEN_SOFT : C_CARD), 0);
  lv_obj_set_style_border_color(b, lv_color_hex(on ? C_GREEN : C_LINE), 0);
  lv_obj_set_style_border_width(b, on ? 2 : 1, 0);
  lv_obj_set_style_text_color(lv_obj_get_child(b, 0), lv_color_hex(on ? C_GREEN_DARK : C_TEXT), 0);
}

static lv_obj_t *countRow(lv_obj_t *parent, const char *name, uint32_t dot, lv_obj_t **valOut) {
  lv_obj_t *r = box(parent, 236, 48, C_CARD);
  lv_obj_set_style_radius(r, 10, 0);
  lv_obj_set_style_border_width(r, 1, 0);
  lv_obj_set_style_border_color(r, lv_color_hex(C_LINE), 0);
  lv_obj_t *d = box(r, 12, 12, dot);
  lv_obj_set_style_radius(d, 6, 0);
  lv_obj_align(d, LV_ALIGN_LEFT_MID, 14, 0);
  lv_obj_t *n = label(r, name, &lv_font_montserrat_16, C_TEXT);
  lv_obj_align(n, LV_ALIGN_LEFT_MID, 36, 0);
  *valOut = label(r, "0", &lv_font_montserrat_22, C_TEXT);
  lv_obj_align(*valOut, LV_ALIGN_RIGHT_MID, -14, 0);
  return r;
}

void goTo(lv_obj_t *scr);

void sendPacket(char type, const uint8_t *data, uint32_t len) {
  if (!pc || !pc.connected()) return;
  uint8_t h[5] = {(uint8_t)type, (uint8_t)len, (uint8_t)(len >> 8), (uint8_t)(len >> 16), (uint8_t)(len >> 24)};
  pc.write(h, 5);
  if (len) pc.write(data, len);
}
void wsSend(JsonDocument &doc) { String s; serializeJson(doc, s); sendPacket('M', (const uint8_t *)s.c_str(), s.length()); }
void setMode(const char *m) { JsonDocument d; d["type"] = "mode"; d["mode"] = m; wsSend(d); }

// ------------------------------------------------------------ screen: Welcome
void onStart(lv_event_t *) { goTo(scrScan); }
void onLock(lv_event_t *) { pinEntry = ""; goTo(scrPin); }

void buildHome() {
  scrHome = box(nullptr, 800, 480, C_BG);
  lv_obj_t *side = box(scrHome, 300, 480, C_GREEN_DARK);
  lv_obj_t *logo = box(side, 48, 48, 0x3FA266);
  lv_obj_set_style_radius(logo, 10, 0);
  lv_obj_set_pos(logo, 24, 28);
  lv_obj_center(label(logo, "MRF", &lv_font_montserrat_14, 0xFFFFFF));
  lv_obj_set_pos(label(side, "SapulMRF", &lv_font_montserrat_20, 0xFFFFFF), 84, 30);
  lv_obj_set_pos(label(side, "Barangay Sapul MRF", &lv_font_montserrat_12, C_GREEN_TXT), 84, 56);
  lv_obj_set_pos(label(side, "TODAY AT THE MRF", &lv_font_montserrat_12, C_GREEN_TXT), 24, 268);
  lblTodayKg = label(side, "-- kg", &lv_font_montserrat_40, 0xFFFFFF);
  lv_obj_set_pos(lblTodayKg, 24, 290);
  lblTodayDrops = label(side, "waiting for PC...", &lv_font_montserrat_14, C_GREEN_TXT);
  lv_obj_set_pos(lblTodayDrops, 24, 340);
  lv_obj_t *ln = box(side, 252, 1, 0x3A6B4D);
  lv_obj_set_pos(ln, 24, 366);
  lblNet = label(side, LV_SYMBOL_WIFI " Connecting...", &lv_font_montserrat_14, 0xFFFFFF);
  lv_label_set_long_mode(lblNet, LV_LABEL_LONG_DOT);
  lv_obj_set_width(lblNet, 252);
  lv_obj_set_pos(lblNet, 24, 378);
  lv_obj_set_pos(label(side, "Kiosk IP", &lv_font_montserrat_14, C_GREEN_TXT), 24, 404);
  lblIp = label(side, "-", &lv_font_montserrat_16, 0xFFFFFF);
  lv_obj_align(lblIp, LV_ALIGN_TOP_RIGHT, -24, 402);
  lv_obj_set_pos(label(side, "Camera PC", &lv_font_montserrat_14, C_GREEN_TXT), 24, 432);
  lblCamStatus = label(side, LV_SYMBOL_CLOSE " Waiting", &lv_font_montserrat_14, 0xF2A33A);
  lv_obj_align(lblCamStatus, LV_ALIGN_TOP_RIGHT, -24, 432);

  lv_obj_t *lock = button(scrHome, LV_SYMBOL_SETTINGS, 48, 48, C_MUTED_BG, C_TEXT2, &lv_font_montserrat_20, onLock);
  lv_obj_set_style_radius(lock, 24, 0);
  lv_obj_align(lock, LV_ALIGN_TOP_RIGHT, -32, 36);   // below the battery pill

  lv_obj_t *h = label(scrHome, "Magandang araw!\nDrop off your waste here.", &lv_font_montserrat_28, C_TEXT);
  lv_obj_set_pos(h, 340, 96);
  lv_obj_t *p = label(scrHome, "Place your bag on the scale under the camera. We'll check what's inside, weigh it, and give you a receipt code.",
                      &lv_font_montserrat_16, C_TEXT2);
  lv_label_set_long_mode(p, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(p, 420);
  lv_obj_set_pos(p, 340, 186);
  lv_obj_t *start = button(scrHome, LV_SYMBOL_EYE_OPEN "  Start drop-off", 420, 76, C_GREEN, 0xFFFFFF, &lv_font_montserrat_22, onStart);
  lv_obj_set_pos(start, 340, 360);
}

// ------------------------------------------------------------ screen: Camera scan
void onScanNext(lv_event_t *) { goTo(scrWeigh); }
void onCancel(lv_event_t *) { goTo(scrHome); }

void buildScan() {
  scrScan = box(nullptr, 800, 480, C_BG);
  lv_obj_t *vid = box(scrScan, 520, 480, 0x1B201D);

  frameBuf = (uint16_t *)ps_malloc(FRAME_W * FRAME_H * 2);
  memset(frameBuf, 0, FRAME_W * FRAME_H * 2);
  frameDsc.header.always_zero = 0;
  frameDsc.header.w = FRAME_W;
  frameDsc.header.h = FRAME_H;
  frameDsc.header.cf = LV_IMG_CF_TRUE_COLOR;
  frameDsc.data_size = FRAME_W * FRAME_H * 2;
  frameDsc.data = (const uint8_t *)frameBuf;
  imgVideo = lv_img_create(vid);
  lv_img_set_src(imgVideo, &frameDsc);
  lv_obj_align(imgVideo, LV_ALIGN_TOP_MID, 0, 40);

  lblVideoWait = label(vid, "Waiting for camera from PC...", &lv_font_montserrat_14, 0x8A968E);
  lv_obj_center(lblVideoWait);
  lv_obj_t *live = box(vid, 120, 28, C_RED);
  lv_obj_set_style_radius(live, 14, 0);
  lv_obj_set_pos(live, 16, 6);
  lblFps = label(live, "LIVE", &lv_font_montserrat_12, 0xFFFFFF);
  lv_obj_center(lblFps);
  lv_obj_t *tip = box(vid, 488, 40, 0x000000);
  lv_obj_set_style_bg_opa(tip, LV_OPA_60, 0);
  lv_obj_set_style_radius(tip, 10, 0);
  lv_obj_align(tip, LV_ALIGN_BOTTOM_MID, 0, -14);
  lv_obj_center(label(tip, "Spread items out so the camera can see each one", &lv_font_montserrat_14, 0xFFFFFF));

  lv_obj_t *side = box(scrScan, 280, 480, C_BG);
  lv_obj_set_pos(side, 520, 0);
  lv_obj_set_pos(label(side, "What we see", &lv_font_montserrat_20, C_TEXT), 22, 24);
  lv_obj_t *step = label(side, "Step 1 of 3", &lv_font_montserrat_12, C_TEXT2);
  lv_obj_align(step, LV_ALIGN_TOP_RIGHT, -22, 30);
  lv_obj_set_pos(countRow(side, "Recyclable", 0x2E9E57, &lblRecyc), 22, 64);
  lv_obj_set_pos(countRow(side, "Residual", 0xD98A1E, &lblResid), 22, 120);
  lv_obj_set_pos(countRow(side, "Biodegradable", 0x7A5A2E, &lblBio), 22, 176);

  lv_obj_t *hint = box(side, 236, 92, C_GREEN_SOFT);
  lv_obj_set_style_radius(hint, 10, 0);
  lv_obj_set_pos(hint, 22, 236);
  lblHint = label(hint, "Show your waste to the camera", &lv_font_montserrat_14, C_GREEN_DARK);
  lv_label_set_long_mode(lblHint, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lblHint, 210);
  lv_obj_align(lblHint, LV_ALIGN_LEFT_MID, 12, 0);

  lv_obj_t *c = button(side, "Cancel", 96, 60, C_MUTED_BG, C_TEXT, &lv_font_montserrat_16, onCancel);
  lv_obj_set_pos(c, 22, 398);
  btnNextScan = button(side, "Next: weigh", 130, 60, C_GREEN, 0xFFFFFF, &lv_font_montserrat_16, onScanNext);
  lv_obj_set_pos(btnNextScan, 128, 398);
  lv_obj_add_state(btnNextScan, LV_STATE_DISABLED);
}

// ------------------------------------------------------------ screen: Weigh & confirm
void refreshWeigh() {
  for (int i = 0; i < 4; i++) setChoice(btnCat[i], category == CATS[i]);
  for (int i = 0; i < NUM_PUROKS; i++) setChoice(btnPurok[i], purok == i + 1);
  if (weightManual) lv_label_set_text(lblWeight, weightEntry.length() ? weightEntry.c_str() : "0");
  else lv_label_set_text_fmt(lblWeight, "%.1f", weightKg);
  bool ok = purok > 0 && weightKg > 0.05f;
  if (ok) lv_obj_clear_state(btnConfirm, LV_STATE_DISABLED); else lv_obj_add_state(btnConfirm, LV_STATE_DISABLED);
}

void onCat(lv_event_t *e) { category = CATS[(intptr_t)lv_event_get_user_data(e)]; refreshWeigh(); }
void onPurok(lv_event_t *e) { purok = (intptr_t)lv_event_get_user_data(e); refreshWeigh(); }
void onWeighBack(lv_event_t *) { goTo(scrScan); }

void onWeightKey(lv_event_t *e) {
  lv_obj_t *m = lv_event_get_target(e);
  const char *k = lv_btnmatrix_get_btn_text(m, lv_btnmatrix_get_selected_btn(m));
  if (!k) return;
  if (!strcmp(k, LV_SYMBOL_BACKSPACE)) { if (weightEntry.length()) weightEntry.remove(weightEntry.length() - 1); }
  else if (!strcmp(k, ".")) { if (weightEntry.indexOf('.') < 0) weightEntry += weightEntry.length() ? "." : "0."; }
  else if (weightEntry.length() < 5) weightEntry += k;
  weightKg = weightEntry.toFloat();
  weightManual = true;
  refreshWeigh();
}

void onConfirm(lv_event_t *) {
  JsonDocument d;
  d["type"] = "intake";
  d["weight"] = weightKg;
  d["category"] = category;
  d["purok"] = purok;
  d["manual"] = weightManual;
  d["recyclables"] = detRecyc;
  wsSend(d);
  lv_label_set_text(lblCode, "...");
  lv_label_set_text_fmt(lblRcWeight, "%.1f kg", weightKg);
  lv_label_set_text(lblRcCat, category.c_str());
  lv_label_set_text_fmt(lblRcPurok, "Purok %d", purok);
  lv_label_set_text_fmt(lblRcRecyc, "%d items", detRecyc);
  goTo(scrReceipt);
}

void buildWeigh() {
  static const char *kmap[] = {"1", "2", "3", "\n", "4", "5", "6", "\n", "7", "8", "9", "\n", ".", "0", LV_SYMBOL_BACKSPACE, ""};
  scrWeigh = box(nullptr, 800, 480, C_BG);
  lv_obj_set_pos(label(scrWeigh, "Weigh & confirm", &lv_font_montserrat_22, C_TEXT), 28, 22);
  lv_obj_t *step = label(scrWeigh, "Step 2 of 3", &lv_font_montserrat_12, C_TEXT2);
  lv_obj_align(step, LV_ALIGN_TOP_RIGHT, -28, 28);

  lv_obj_t *card = box(scrWeigh, 300, 316, C_GREEN_DARK);
  lv_obj_set_style_radius(card, 16, 0);
  lv_obj_set_pos(card, 28, 64);
  lv_obj_set_pos(label(card, USE_HX711 ? "SCALE READING" : "ENTER WEIGHT (KG)", &lv_font_montserrat_12, C_GREEN_TXT), 20, 16);
  lblWeight = label(card, "0", &lv_font_montserrat_40, 0xFFFFFF);
  lv_obj_set_pos(lblWeight, 20, 36);
  lblWeightState = label(card, "kg", &lv_font_montserrat_20, C_GREEN_TXT);
  lv_obj_align_to(lblWeightState, lblWeight, LV_ALIGN_OUT_RIGHT_BOTTOM, 8, -4);
  kpWeight = lv_btnmatrix_create(card);
  lv_btnmatrix_set_map(kpWeight, kmap);
  lv_obj_set_size(kpWeight, 270, 210);
  lv_obj_set_pos(kpWeight, 15, 94);
  lv_obj_set_style_bg_opa(kpWeight, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(kpWeight, 0, 0);
  lv_obj_set_style_pad_all(kpWeight, 0, 0);
  lv_obj_set_style_pad_gap(kpWeight, 6, 0);
  lv_obj_set_style_bg_color(kpWeight, lv_color_hex(0x2C6B45), LV_PART_ITEMS);
  lv_obj_set_style_text_color(kpWeight, lv_color_hex(0xFFFFFF), LV_PART_ITEMS);
  lv_obj_set_style_text_font(kpWeight, &lv_font_montserrat_22, LV_PART_ITEMS);
  lv_obj_set_style_shadow_width(kpWeight, 0, LV_PART_ITEMS);
  lv_obj_add_event_cb(kpWeight, onWeightKey, LV_EVENT_VALUE_CHANGED, nullptr);

  lv_obj_set_pos(label(scrWeigh, "CATEGORY - from camera", &lv_font_montserrat_12, C_TEXT2), 348, 64);
  for (int i = 0; i < 4; i++) {
    btnCat[i] = button(scrWeigh, CATS[i], 202, 48, C_CARD, C_TEXT, &lv_font_montserrat_14, onCat, (void *)(intptr_t)i);
    lv_obj_set_style_radius(btnCat[i], 10, 0);
    lv_obj_set_pos(btnCat[i], 348 + (i % 2) * 212, 86 + (i / 2) * 56);
  }
  lv_obj_set_pos(label(scrWeigh, "YOUR PUROK", &lv_font_montserrat_12, C_TEXT2), 348, 212);
  for (int i = 0; i < NUM_PUROKS; i++) {
    char t[4]; snprintf(t, sizeof t, "%d", i + 1);
    btnPurok[i] = button(scrWeigh, t, 54, 54, C_CARD, C_TEXT, &lv_font_montserrat_20, onPurok, (void *)(intptr_t)(i + 1));
    lv_obj_set_style_radius(btnPurok[i], 10, 0);
    lv_obj_set_pos(btnPurok[i], 348 + i * 60, 234);
  }
  lv_obj_t *b = button(scrWeigh, "Back", 140, 60, C_MUTED_BG, C_TEXT, &lv_font_montserrat_16, onWeighBack);
  lv_obj_set_pos(b, 28, 396);
  btnConfirm = button(scrWeigh, "Confirm drop-off", 594, 60, C_GREEN, 0xFFFFFF, &lv_font_montserrat_20, onConfirm);
  lv_obj_set_pos(btnConfirm, 178, 396);
#if USE_HX711
  lv_obj_add_flag(kpWeight, LV_OBJ_FLAG_HIDDEN);   // scale fills the weight instead
#endif
}

// ------------------------------------------------------------ screen: Receipt
void onDone(lv_event_t *) { goTo(scrHome); }

void receiptTick(lv_timer_t *) {
  if (--receiptCountdown <= 0) { goTo(scrHome); return; }
  lv_label_set_text_fmt(lblCountdown, "Returning to start in %d s", receiptCountdown);
}

static lv_obj_t *rcRow(lv_obj_t *card, int y, const char *k, lv_obj_t **v) {
  lv_obj_set_pos(label(card, k, &lv_font_montserrat_16, C_TEXT2), 20, y);
  *v = label(card, "-", &lv_font_montserrat_16, C_TEXT);
  lv_obj_align(*v, LV_ALIGN_TOP_RIGHT, -20, y);
  return *v;
}

void buildReceipt() {
  scrReceipt = box(nullptr, 800, 480, C_BG);
  lv_obj_t *ok = box(scrReceipt, 64, 64, C_GREEN);
  lv_obj_set_style_radius(ok, 32, 0);
  lv_obj_set_pos(ok, 32, 28);
  lv_obj_center(label(ok, LV_SYMBOL_OK, &lv_font_montserrat_28, 0xFFFFFF));
  lv_obj_t *h = label(scrReceipt, "Salamat!\nDrop-off recorded.", &lv_font_montserrat_28, C_TEXT);
  lv_obj_set_pos(h, 32, 110);
  lv_obj_t *p = label(scrReceipt, "Show this code to the MRF staff if they ask. It's also saved in today's intake records.",
                      &lv_font_montserrat_16, C_TEXT2);
  lv_label_set_long_mode(p, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(p, 400);
  lv_obj_set_pos(p, 32, 196);
  lblCountdown = label(scrReceipt, "", &lv_font_montserrat_14, C_TEXT2);
  lv_obj_set_pos(lblCountdown, 32, 360);
  lv_obj_t *d = button(scrReceipt, "Done", 408, 64, C_GREEN, 0xFFFFFF, &lv_font_montserrat_20, onDone);
  lv_obj_set_pos(d, 32, 388);

  lv_obj_t *card = box(scrReceipt, 300, 424, C_CARD);
  lv_obj_set_style_radius(card, 16, 0);
  lv_obj_set_style_border_width(card, 1, 0);
  lv_obj_set_style_border_color(card, lv_color_hex(C_LINE), 0);
  lv_obj_set_pos(card, 468, 28);
  lv_obj_set_pos(label(card, "RECEIPT CODE", &lv_font_montserrat_12, C_TEXT2), 20, 22);
  lblCode = label(card, "...", &lv_font_montserrat_40, C_GREEN_DARK);
  lv_obj_set_pos(lblCode, 20, 44);
  lv_obj_t *ln = box(card, 260, 1, C_MUTED_BG);
  lv_obj_set_pos(ln, 20, 104);
  rcRow(card, 120, "Weight", &lblRcWeight);
  rcRow(card, 152, "Category", &lblRcCat);
  rcRow(card, 184, "Purok", &lblRcPurok);
  rcRow(card, 216, "Recyclables seen", &lblRcRecyc);
  lv_obj_t *tip = box(card, 260, 72, C_AMBER_BG);
  lv_obj_set_style_radius(tip, 10, 0);
  lv_obj_set_pos(tip, 20, 330);
  lv_obj_t *tl = label(tip, "Tip: separate bottles and cans next time for a \"Segregated\" record.", &lv_font_montserrat_14, C_AMBER_TXT);
  lv_label_set_long_mode(tl, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(tl, 236);
  lv_obj_center(tl);
}

// ------------------------------------------------------------ screen: Operator PIN
void refreshPin() {
  for (int i = 0; i < 4; i++) {
    bool on = i < (int)pinEntry.length();
    lv_obj_set_style_bg_opa(pinDots[i], on ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
  }
}

void onPinKey(lv_event_t *e) {
  lv_obj_t *m = lv_event_get_target(e);
  const char *k = lv_btnmatrix_get_btn_text(m, lv_btnmatrix_get_selected_btn(m));
  if (!k) return;
  lv_label_set_text(lblPinMsg, "");
  if (!strcmp(k, "Clear")) pinEntry = "";
  else if (!strcmp(k, "Enter")) {
    if (pinEntry.length() != 4) { lv_label_set_text(lblPinMsg, "Enter all 4 digits"); }
    else if (pcUp) {                       // PC checks it against OPERATOR_PINS
      JsonDocument d; d["type"] = "pin"; d["pin"] = pinEntry; wsSend(d);
      lv_label_set_text(lblPinMsg, "Checking...");
      pinAskedAt = millis();
    } else if (pinEntry == KIOSK_PIN) {    // PC offline: use the kiosk's own PIN
      lv_table_set_row_cnt(tblRecent, 2);
      lv_table_set_cell_value(tblRecent, 1, 0, "-");
      lv_table_set_cell_value(tblRecent, 1, 3, "Camera PC offline");
      goTo(scrOp);
    } else {
      pinEntry = "";
      lv_label_set_text(lblPinMsg, "Wrong PIN (camera PC offline)");
    }
  } else if (pinEntry.length() < 4) pinEntry += k;
  refreshPin();
}

void buildPin() {
  static const char *pmap[] = {"1", "2", "3", "\n", "4", "5", "6", "\n", "7", "8", "9", "\n", "Clear", "0", "Enter", ""};
  scrPin = box(nullptr, 800, 480, C_BG);
  lv_obj_t *side = box(scrPin, 360, 480, C_GREEN_DARK);
  lv_obj_set_pos(label(side, "OPERATOR MODE", &lv_font_montserrat_12, C_GREEN_TXT), 28, 32);
  lv_obj_set_pos(label(side, "Enter your\n4-digit PIN", &lv_font_montserrat_28, 0xFFFFFF), 28, 58);
  for (int i = 0; i < 4; i++) {
    pinDots[i] = box(side, 22, 22, 0xFFFFFF);
    lv_obj_set_style_radius(pinDots[i], 11, 0);
    lv_obj_set_style_border_width(pinDots[i], 2, 0);
    lv_obj_set_style_border_color(pinDots[i], lv_color_hex(0x9FBBA9), 0);
    lv_obj_set_pos(pinDots[i], 28 + i * 36, 150);
  }
  lblPinMsg = label(side, "", &lv_font_montserrat_14, 0xFFB4A8);
  lv_obj_set_pos(lblPinMsg, 28, 190);
  lv_obj_t *back = button(side, "Back to drop-off", 304, 52, C_GREEN_DARK, 0xFFFFFF, &lv_font_montserrat_16, onCancel);
  lv_obj_set_style_border_width(back, 1, 0);
  lv_obj_set_style_border_color(back, lv_color_hex(0x5E8C70), 0);
  lv_obj_set_pos(back, 28, 400);

  lv_obj_t *kp = lv_btnmatrix_create(scrPin);
  lv_btnmatrix_set_map(kp, pmap);
  lv_obj_set_size(kp, 404, 424);
  lv_obj_set_pos(kp, 378, 28);
  lv_obj_set_style_bg_opa(kp, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(kp, 0, 0);
  lv_obj_set_style_pad_all(kp, 0, 0);
  lv_obj_set_style_pad_gap(kp, 12, 0);
  lv_obj_set_style_bg_color(kp, lv_color_hex(C_CARD), LV_PART_ITEMS);
  lv_obj_set_style_border_color(kp, lv_color_hex(C_LINE), LV_PART_ITEMS);
  lv_obj_set_style_border_width(kp, 1, LV_PART_ITEMS);
  lv_obj_set_style_radius(kp, 14, LV_PART_ITEMS);
  lv_obj_set_style_shadow_width(kp, 0, LV_PART_ITEMS);
  lv_obj_set_style_text_font(kp, &lv_font_montserrat_28, LV_PART_ITEMS);
  lv_obj_set_style_text_color(kp, lv_color_hex(C_TEXT), LV_PART_ITEMS);
  lv_obj_add_event_cb(kp, onPinKey, LV_EVENT_VALUE_CHANGED, nullptr);
}

// ------------------------------------------------------------ screen: Operator station
void onOpLock(lv_event_t *) { JsonDocument d; d["type"] = "lock"; wsSend(d); goTo(scrHome); }
void onOpRefresh(lv_event_t *) { JsonDocument d; d["type"] = "recent"; wsSend(d); }

void buildOperator() {
  scrOp = box(nullptr, 800, 480, C_BG);
  lv_obj_t *side = box(scrOp, 168, 480, C_GREEN_DARK);
  lv_obj_set_pos(label(side, "SapulMRF", &lv_font_montserrat_16, 0xFFFFFF), 16, 22);
  lv_obj_t *nav = box(side, 148, 44, C_BG);
  lv_obj_set_style_radius(nav, 8, 0);
  lv_obj_set_pos(nav, 10, 64);
  lv_obj_align(label(nav, "Intake", &lv_font_montserrat_14, C_GREEN_DARK), LV_ALIGN_LEFT_MID, 10, 0);
  lv_obj_t *lock = button(side, LV_SYMBOL_CLOSE " Lock", 148, 44, C_GREEN_DARK, 0xFFFFFF, &lv_font_montserrat_14, onOpLock);
  lv_obj_set_style_border_width(lock, 1, 0);
  lv_obj_set_style_border_color(lock, lv_color_hex(0x5E8C70), 0);
  lv_obj_set_pos(lock, 10, 420);

  lv_obj_set_pos(label(scrOp, "Intake records", &lv_font_montserrat_20, C_TEXT), 186, 16);
  lv_obj_set_pos(label(scrOp, "Newest first - from the kiosk bridge", &lv_font_montserrat_12, C_TEXT2), 186, 42);
  lv_obj_t *r = button(scrOp, LV_SYMBOL_REFRESH " Refresh", 130, 44, C_GREEN, 0xFFFFFF, &lv_font_montserrat_14, onOpRefresh);
  lv_obj_align(r, LV_ALIGN_TOP_RIGHT, -112, 14);     // leaves room for the battery pill

  tblRecent = lv_table_create(scrOp);
  lv_obj_set_pos(tblRecent, 186, 74);
  lv_obj_set_size(tblRecent, 596, 392);
  lv_table_set_col_cnt(tblRecent, 5);
  const int cw[5] = {90, 104, 100, 190, 100};
  const char *hd[5] = {"Code", "Time", "Purok", "Category", "Weight"};
  for (int i = 0; i < 5; i++) { lv_table_set_col_width(tblRecent, i, cw[i]); lv_table_set_cell_value(tblRecent, 0, i, hd[i]); }
  lv_obj_set_style_text_font(tblRecent, &lv_font_montserrat_14, LV_PART_ITEMS);
  lv_obj_set_style_pad_ver(tblRecent, 10, LV_PART_ITEMS);
  lv_obj_set_style_border_color(tblRecent, lv_color_hex(C_LINE), 0);
  lv_obj_set_style_radius(tblRecent, 10, 0);
}

// ------------------------------------------------------------ navigation
void goTo(lv_obj_t *scr) {
  lv_obj_t *cur = lv_scr_act();
  if (cur == scrScan && scr != scrScan) setMode("idle");
  if (scr == scrScan) {
    detRecyc = detResid = detBio = 0; detStable = false;
    lv_obj_clear_flag(lblVideoWait, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_state(btnNextScan, LV_STATE_DISABLED);
    setMode("scan");
  }
  if (scr == scrWeigh) {
    if (!USE_HX711 && cur == scrScan) { weightEntry = ""; weightKg = 0; }
    refreshWeigh();
  }
  if (scr == scrReceipt) {
    receiptCountdown = RECEIPT_RETURN_S;
    lv_label_set_text_fmt(lblCountdown, "Returning to start in %d s", receiptCountdown);
    if (!receiptTimer) receiptTimer = lv_timer_create(receiptTick, 1000, nullptr);
    else lv_timer_resume(receiptTimer);
  } else if (receiptTimer) lv_timer_pause(receiptTimer);
  if (scr == scrHome) { purok = 0; category = CATS[0]; JsonDocument d; d["type"] = "stats"; wsSend(d); }
  if (scr == scrPin) { pinEntry = ""; lv_label_set_text(lblPinMsg, ""); refreshPin(); }
  lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_FADE_ON, 150, 0, false);
}

// ------------------------------------------------------------ incoming data
int jpegDraw(JPEGDRAW *p) {                 // copy decoded block into the frame buffer
  for (int y = 0; y < p->iHeight; y++) {
    int dy = p->y + y;
    if (dy >= FRAME_H) break;
    int w = min(p->iWidth, FRAME_W - p->x);
    memcpy(&frameBuf[dy * FRAME_W + p->x], &p->pPixels[y * p->iWidth], w * 2);
  }
  return 1;
}

void onFrame(uint8_t *data, size_t len) {
  if (lv_scr_act() != scrScan) return;
  if (jpeg.openRAM(data, len, jpegDraw)) {
    jpeg.setPixelType(RGB565_LITTLE_ENDIAN);
    jpeg.decode(0, 0, 0);
    jpeg.close();
    lv_obj_add_flag(lblVideoWait, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(imgVideo);
  }
}

void onText(uint8_t *data, size_t len) {
  JsonDocument d;
  if (deserializeJson(d, data, len)) return;
  const char *t = d["type"] | "";
  if (!strcmp(t, "det")) {
    detRecyc = d["recyclable"] | 0; detResid = d["residual"] | 0; detBio = d["bio"] | 0;
    detFps = d["fps"] | 0; detStable = d["stable"] | false;
    const char *cat = d["category"] | "Nothing detected";
    lv_label_set_text_fmt(lblRecyc, "%d", detRecyc);
    lv_label_set_text_fmt(lblResid, "%d", detResid);
    lv_label_set_text_fmt(lblBio, "%d", detBio);
    lv_label_set_text_fmt(lblFps, LV_SYMBOL_VIDEO " LIVE %d fps", detFps);
    if (!strcmp(cat, "Nothing detected")) {
      lv_label_set_text(lblHint, "Show your waste to the camera");
    } else {
      lv_label_set_text_fmt(lblHint, "Looks like %s.\n%s", cat, detStable ? "Got it - tap Next." : "Hold steady...");
      for (int i = 0; i < 4; i++) if (!strcmp(cat, CATS[i])) category = CATS[i];
    }
    if (detStable) lv_obj_clear_state(btnNextScan, LV_STATE_DISABLED);
  } else if (!strcmp(t, "receipt")) {
    lv_label_set_text(lblCode, d["code"] | "OFFLINE");
  } else if (!strcmp(t, "stats")) {
    lv_label_set_text_fmt(lblTodayKg, "%.1f kg", (float)(d["kg"] | 0.0));
    lv_label_set_text_fmt(lblTodayDrops, "collected from %d drop-offs", (int)(d["drops"] | 0));
  } else if (!strcmp(t, "auth")) {
    pinAskedAt = 0;
    if (d["ok"] | false) { onOpRefresh(nullptr); goTo(scrOp); }
    else { pinEntry = ""; refreshPin(); lv_label_set_text(lblPinMsg, "Wrong PIN, try again"); }
  } else if (!strcmp(t, "recent")) {
    JsonArray rows = d["rows"];
    lv_table_set_row_cnt(tblRecent, rows.size() + 1);
    int r = 1;
    for (JsonObject o : rows) {
      lv_table_set_cell_value(tblRecent, r, 0, o["code"] | "");
      lv_table_set_cell_value(tblRecent, r, 1, o["time"] | "");
      lv_table_set_cell_value(tblRecent, r, 2, o["purok"] | "");
      lv_table_set_cell_value(tblRecent, r, 3, o["cat"] | "");
      lv_table_set_cell_value(tblRecent, r, 4, o["kg"] | "");
      r++;
    }
  }
}

// ------------------------------------------------------------ PC link (TCP server, like sapul stream)
void setPcStatus(bool up) {
  pcUp = up;
  lv_label_set_text(lblCamStatus, up ? LV_SYMBOL_OK " Connected" : LV_SYMBOL_CLOSE " Waiting");
  lv_obj_set_style_text_color(lblCamStatus, lv_color_hex(up ? C_GREEN_OK : 0xF2A33A), 0);
  if (!up && lv_scr_act() == scrScan)
    lv_label_set_text(lblVideoWait, "Camera PC disconnected - waiting...");
}

void handlePacket() {
  if (rxType == 'J') {
    onFrame(rxBuf, rxLen);
    sendPacket('A', nullptr, 0);          // ACK: shown, PC may send the next frame
  } else if (rxType == 'M') {
    onText(rxBuf, rxLen);
  }
}

void pcLoop() {
  if (server.hasClient()) {               // new PC: replace the old one
    WiFiClient n = server.available();
    if (pc && pc.connected()) pc.stop();
    pc = n;
    pc.setNoDelay(true);
    rxHdrGot = 0; rxType = 0;
    Serial.printf("PC connected from %s\n", pc.remoteIP().toString().c_str());
    setPcStatus(true);
    setMode(lv_scr_act() == scrScan ? "scan" : "idle");
    JsonDocument d; d["type"] = "stats"; wsSend(d);
  }
  if (pcUp && !pc.connected()) {
    Serial.println("PC disconnected");
    pc.stop();
    setPcStatus(false);
    rxHdrGot = 0; rxType = 0;
  }
  uint32_t t0 = millis();
  while (pc && pc.available() && millis() - t0 < 30) {   // read a slice, keep the UI smooth
    if (rxType == 0) {
      while (rxHdrGot < 5 && pc.available()) rxHdr[rxHdrGot++] = pc.read();
      if (rxHdrGot < 5) break;
      rxHdrGot = 0;
      rxType = (char)rxHdr[0];
      rxLen = rxHdr[1] | (rxHdr[2] << 8) | (rxHdr[3] << 16) | ((uint32_t)rxHdr[4] << 24);
      rxGot = 0;
      if (rxLen > MAX_PACKET || (rxType != 'J' && rxType != 'M')) {
        Serial.printf("Bad packet %c len %u - dropping PC\n", rxType, rxLen);
        pc.stop(); rxType = 0; break;
      }
      if (rxLen == 0) { handlePacket(); rxType = 0; continue; }
    }
    int n = pc.read(rxBuf + rxGot, rxLen - rxGot);
    if (n <= 0) break;
    rxGot += n;
    if (rxGot == rxLen) { handlePacket(); rxType = 0; }
  }
}

// ------------------------------------------------------------ Wi-Fi / hotspot
void showNet(const char *line) {
  lv_label_set_text(lblNet, line);
  lv_timer_handler();
}

bool tryJoin(const char *ssid, const char *pass) {
  if (!ssid || !*ssid) return false;
  char buf[64]; snprintf(buf, sizeof buf, LV_SYMBOL_WIFI " Joining %s...", ssid);
  showNet(buf);
  WiFi.begin(ssid, pass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TRY_MS) { lv_timer_handler(); delay(20); }
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.disconnect();
  delay(100);
  return false;
}

void netStart() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                   // keeps frame latency low
  if (tryJoin(WIFI1_SSID, WIFI1_PASS)) netName = WIFI1_SSID;
  else if (tryJoin(WIFI2_SSID, WIFI2_PASS)) netName = WIFI2_SSID;

  if (netName.length()) {
    apMode = false;
    WiFi.setAutoReconnect(true);
    netIp = WiFi.localIP().toString();
    lv_label_set_text_fmt(lblNet, LV_SYMBOL_WIFI " Wi-Fi: %s", netName.c_str());
  } else {                                // no network found: be the hotspot
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    apMode = true;
    netName = AP_SSID;
    netIp = WiFi.softAPIP().toString();
    lv_label_set_text_fmt(lblNet, LV_SYMBOL_WIFI " Hotspot: %s", AP_SSID);
  }
  lv_label_set_text(lblIp, netIp.c_str());
  Serial.printf("%s %s  IP %s  port %d\n", apMode ? "Hotspot" : "Wi-Fi", netName.c_str(), netIp.c_str(), STREAM_PORT);
  server.begin();
  server.setNoDelay(true);
}

void beaconLoop() {                       // lets the PC script find the kiosk without typing the IP
  static uint32_t last = 0;
  if (millis() - last < 2000) return;
  last = millis();
  IPAddress ip = apMode ? WiFi.softAPIP() : WiFi.localIP();
  IPAddress mask = apMode ? IPAddress(255, 255, 255, 0) : WiFi.subnetMask();
  IPAddress bc(ip[0] | ~mask[0], ip[1] | ~mask[1], ip[2] | ~mask[2], ip[3] | ~mask[3]);
  char msg[48];
  snprintf(msg, sizeof msg, "SAPULMRF %s %d", ip.toString().c_str(), STREAM_PORT);
  beacon.beginPacket(bc, BEACON_PORT);
  beacon.write((const uint8_t *)msg, strlen(msg));
  beacon.endPacket();

  if (!apMode) {                          // Wi-Fi dropped / came back
    bool up = WiFi.status() == WL_CONNECTED;
    String now = up ? WiFi.localIP().toString() : String("-");
    if (now != netIp) {
      netIp = now;
      lv_label_set_text(lblIp, netIp.c_str());
      lv_label_set_text_fmt(lblNet, up ? LV_SYMBOL_WIFI " Wi-Fi: %s" : LV_SYMBOL_WARNING " Lost %s", netName.c_str());
    }
  }
}

// ------------------------------------------------------------ battery (INA219, as in the Alangan translator)
bool inaWrite(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg); Wire.write(v >> 8); Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}

bool inaRead(uint8_t reg, uint16_t &v) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2) != 2) return false;
  v = (Wire.read() << 8) | Wire.read();
  return true;
}

float lipoPercent(float mv) {             // resting Li-ion curve, single cell
  static const float T[][2] = {{3300, 0}, {3500, 5}, {3600, 10}, {3700, 30}, {3750, 45}, {3800, 55},
                               {3850, 63}, {3900, 70}, {4000, 82}, {4100, 93}, {BATTERY_FULL_MV, 100}};
  const int N = sizeof T / sizeof T[0];
  if (mv <= T[0][0]) return 0;
  if (mv >= T[N - 1][0]) return 100;
  for (int i = 1; i < N; i++)
    if (mv < T[i][0]) return T[i - 1][1] + (mv - T[i - 1][0]) * (T[i][1] - T[i - 1][1]) / (T[i][0] - T[i - 1][0]);
  return 100;
}

void buildBatteryPill() {                 // on lv_layer_top(): visible on every screen
  batPill = box(lv_layer_top(), 96, 24, C_CARD);
  lv_obj_set_style_radius(batPill, 12, 0);
  lv_obj_set_style_border_width(batPill, 1, 0);
  lv_obj_set_style_border_color(batPill, lv_color_hex(C_LINE), 0);
  lv_obj_align(batPill, LV_ALIGN_TOP_RIGHT, -8, 4);
  lv_obj_clear_flag(batPill, LV_OBJ_FLAG_CLICKABLE);
  lblBat = label(batPill, LV_SYMBOL_BATTERY_EMPTY " --%", &lv_font_montserrat_14, C_TEXT);
  lv_obj_center(lblBat);
  lv_obj_add_flag(batPill, LV_OBJ_FLAG_HIDDEN);
}

void batteryInit() {
#if BATTERY_USE_INA219
  Wire.beginTransmission(INA219_ADDR);
  batFound = Wire.endTransmission() == 0 && inaWrite(0x00, 0x399F);   // 32V, ±320mV, continuous
  Serial.printf("Battery: INA219 %s (0x%02X)\n", batFound ? "found" : "not found", INA219_ADDR);
#endif
}

void batteryLoop() {
  static uint32_t last = 0, lastLog = 0;
  if (!batFound || millis() - last < 2000) return;
  last = millis();
  uint16_t rb, rs;
  if (!inaRead(0x02, rb) || !inaRead(0x01, rs)) return;
  batMv = (rb >> 3) * 4.0f;                              // bus voltage, 4 mV/bit
  batMa = ((int16_t)rs * 0.01f) / INA219_SHUNT_OHM;      // shunt 10 uV/bit -> mA (+ = discharging)
  if (batMv < 2500) { lv_obj_add_flag(batPill, LV_OBJ_FLAG_HIDDEN); return; }  // no cell on VIN+

  bool charging = batMa < BATTERY_CHARGE_MA;
  float restMv = batMv + (batMa > 0 ? batMa * BATTERY_INTERNAL_MOHM / 1000.0f : 0);  // undo load sag
  float p = lipoPercent(restMv);
  batPct = batPct < 0 ? p : batPct * 0.8f + p * 0.2f;   // smooth
  int pct = (int)(batPct + 0.5f);

  const char *sym = charging ? LV_SYMBOL_CHARGE
                  : pct > 80 ? LV_SYMBOL_BATTERY_FULL : pct > 55 ? LV_SYMBOL_BATTERY_3
                  : pct > 30 ? LV_SYMBOL_BATTERY_2 : pct > 10 ? LV_SYMBOL_BATTERY_1 : LV_SYMBOL_BATTERY_EMPTY;
  lv_label_set_text_fmt(lblBat, "%s %d%%", sym, pct);
  bool low = pct < BATTERY_LOW_PCT && !charging;
  lv_obj_set_style_text_color(lblBat, lv_color_hex(low ? C_RED : charging ? C_GREEN : C_TEXT), 0);
  lv_obj_clear_flag(batPill, LV_OBJ_FLAG_HIDDEN);

  if (millis() - lastLog > 10000) {
    lastLog = millis();
    Serial.printf("Battery: %.0f mV, %.0f mA%s -> %d%%\n", batMv, batMa, charging ? ", charging" : "", pct);
  }
}

// ------------------------------------------------------------ backlight (PWM on TFT_BL)
void setBacklight(int pct) {
  pct = constrain(pct, 5, 100);
  uint32_t duty = (255 * pct) / 100;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  static bool attached = false;
  if (!attached) { ledcAttach(TFT_BL, 5000, 8); attached = true; }
  ledcWrite(TFT_BL, duty);
#else
  static bool attached = false;
  if (!attached) { ledcSetup(0, 5000, 8); ledcAttachPin(TFT_BL, 0); attached = true; }
  ledcWrite(0, duty);
#endif
}

// ------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("PSRAM: %u KB total, %u KB free\n", ESP.getPsramSize() / 1024, ESP.getFreePsram() / 1024);
  if (ESP.getPsramSize() == 0) Serial.println("!! PSRAM is OFF - set Tools > PSRAM > OPI PSRAM, the screen needs it");
#if BACKLIGHT_PWM
  setBacklight(BACKLIGHT_PCT);
#else
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif
  bool ok = gfx->begin();
  Serial.printf("Display begin: %s\n", ok ? "OK" : "FAILED (check PSRAM / pins)");
#if DISPLAY_TEST
  // Colour test: red, green, blue, white for half a second each
  const uint16_t cols[4] = {0xF800, 0x07E0, 0x001F, 0xFFFF};
  const char *names[4] = {"red", "green", "blue", "white"};
  for (int i = 0; i < 4; i++) { Serial.printf("Display test: %s\n", names[i]); gfx->fillScreen(cols[i]); delay(500); }
#endif
  gfx->fillScreen(0x0000);
  Wire.begin(TOUCH_SDA, TOUCH_SCL);      // shared by GT911 touch and INA219
  tp.begin();
  tp.setRotation(ROTATION_INVERTED);   // if taps land mirrored, try ROTATION_NORMAL

  lv_init();
  const size_t bufPx = 800 * 60;
  lv_buf1 = (lv_color_t *)heap_caps_malloc(bufPx * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  lv_buf2 = (lv_color_t *)heap_caps_malloc(bufPx * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  lv_disp_draw_buf_init(&draw_buf, lv_buf1, lv_buf2, bufPx);
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 800;
  disp_drv.ver_res = 480;
  disp_drv.flush_cb = my_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touch;
  lv_indev_drv_register(&indev_drv);

  rxBuf = (uint8_t *)ps_malloc(MAX_PACKET);
  buildHome(); buildScan(); buildWeigh(); buildReceipt(); buildPin(); buildOperator();
  buildBatteryPill();
  lv_scr_load(scrHome);
  lv_timer_handler();

  batteryInit();
  batteryLoop();

#if USE_HX711
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(HX711_SCALE);
  scale.tare();
  weightManual = false;
#endif

  netStart();                            // Wi-Fi -> phone hotspot -> own hotspot
}

void loop() {
  pcLoop();
  beaconLoop();
  batteryLoop();
  lv_timer_handler();

#if USE_HX711
  static uint32_t lastW = 0;
  if (lv_scr_act() == scrWeigh && millis() - lastW > 300 && scale.is_ready()) {
    lastW = millis();
    weightKg = max(0.0f, scale.get_units(3));
    weightManual = false;
    refreshWeigh();
  }
#endif

  if (pinAskedAt && millis() - pinAskedAt > 3000) {   // PC never answered the PIN
    pinAskedAt = 0;
    pinEntry = ""; refreshPin();
    lv_label_set_text(lblPinMsg, "No answer from camera PC - try again");
  }

  if (lv_scr_act() != scrHome && lv_disp_get_inactive_time(nullptr) > IDLE_RETURN_MS) {
    goTo(scrHome);
    lv_disp_trig_activity(nullptr);
  }
  delay(2);
}
