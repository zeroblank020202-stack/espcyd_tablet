# SapulMRF Kiosk

7" ESP32-S3 touch kiosk (800×480) for Barangay Sapul MRF. The PC runs YOLO on the USB camera.

```
USB cam ─► PC: kiosk_bridge.py (YOLO, draws boxes, saves intake)
               │  connects to the kiosk: TCP 5000 (JPEG + ACK, like the SapulMRF stream)
               ▼
        ESP32-S3 7" kiosk (LVGL) = the server, shows its IP
        Welcome → Camera scan → Weigh & confirm → Receipt
        🔒 → Operator PIN → Operator station
        🔋 INA219 battery % on every screen
```

## How they find each other
At power-on the kiosk tries, in order:
1. **Your Wi-Fi** (`WIFI1_SSID`)
2. **A phone hotspot** (`WIFI2_SSID`)
3. If neither is found, it makes **its own hotspot** `SapulMRF-Kiosk` (password `sapulmrf123`) at **192.168.4.1**.

The welcome screen shows which network it's on, its IP, and whether the camera PC is connected.
Put the PC on that same network (for option 3, join `SapulMRF-Kiosk` from the PC's Wi-Fi list).
The kiosk broadcasts a beacon every 2 s, so the PC script finds it without typing the IP.

## 1. PC bridge
```
cd pc_bridge
pip install ultralytics opencv-python requests
python kiosk_bridge.py --model path/to/best.pt --cam 1
```
- If auto-find doesn't work (some hotspots block broadcasts), add the IP from the kiosk screen:
  `python kiosk_bridge.py --model best.pt --cam 1 192.168.1.77`
- If the PC is on the kiosk's own hotspot and the beacon isn't heard, the script tries 192.168.4.1 automatically.
- It reconnects by itself if the kiosk restarts or Wi-Fi drops.
- Edit `RECYCLABLE` / `BIODEGRADABLE` near the top so they match your model's class names. Everything else counts as Residual.
- Change `OPERATOR_PINS` (default `1234`).
- Every drop-off is saved to `kiosk_intake.csv`, which works offline. To also push records to your SapulMRF backend, set `SAPULMRF_API=http://<server>` (and optionally `SAPULMRF_TOKEN`). The bridge POSTs to `/api/intake` and uses the returned `code`. Adjust `save_intake()` if your API differs.
- Windows Firewall: allow Python on **private** networks (it must receive UDP 5001 for auto-find).

## 2. Kiosk firmware (Arduino IDE)
1. Board: **ESP32S3 Dev Module**, PSRAM **OPI PSRAM**, Flash **16MB QIO**, Partition **Huge APP**.
2. Install the libraries listed at the top of `SapulMRF_Kiosk.ino`. Use **lvgl 8.3.x**, not 9. The WebSockets library is no longer needed.
3. Copy `SapulMRF_Kiosk/lv_conf.h` into your Arduino `libraries` folder, next to `lvgl/`.
4. In `config.h`, set `WIFI1_*` (router) and `WIFI2_*` (phone hotspot). Leave either as `""` to skip it.
5. Upload.

## 3. Battery indicator (INA219, same wiring as the Alangan translator)
| INA219 | Connect to |
|---|---|
| VCC | 3.3V |
| GND | GND, shared with battery − and the board |
| SDA / SCL | GPIO 19 / GPIO 20 (P3 connector, shared with touch) |
| VIN+ | battery + (after the switch) |
| VIN− | charger/boost module B+ |

- The icon and % sit top-right on every screen: green ⚡ while charging, red below 15%.
- If no INA219 is found, or no battery reaches VIN+, the icon stays hidden.
- The serial monitor prints `Battery: INA219 found (0x40)` at boot, then `Battery: 3912 mV, 480 mA -> 72%` every 10 s.
- Settings in `config.h`: `INA219_ADDR`, `INA219_SHUNT_OHM`, `BATTERY_INTERNAL_MOHM` (raise it if % drops when the video starts), `BATTERY_USE_INA219 0` to turn it off.
- If ⚡ shows while unplugged, VIN+ and VIN− are swapped.

## Troubleshooting
| Symptom | Fix |
|---|---|
| Kiosk always ends up on its own hotspot | Check SSID/password spelling; the ESP32 only sees **2.4 GHz** Wi-Fi (set the phone hotspot to 2.4 GHz / "maximize compatibility") |
| PC says "Looking for the kiosk" forever | Pass the IP shown on the kiosk; check the PC is on the same network and the firewall allows Python |
| Screen flickers / shifts while video streams | Lower `RGB_PCLK_HZ` to 12000000, or lower `JPEG_QUALITY` / `SEND_FPS` on the PC |
| Taps land in the wrong place | Change `tp.setRotation(ROTATION_INVERTED)` to `ROTATION_NORMAL` |
| Touch stops working after adding the INA219 | Remove the pull-up resistors from the INA219 module (the touch bus already has them) |
| "Next: weigh" never enables | The category must stay the same for `STABLE_FRAMES` frames. Lower it, or lower `CONF` |

## Optional load-cell scale
Set `USE_HX711 1` in `config.h`, wire the HX711 to two free GPIOs (defaults 17/18; check your board's headers), and calibrate `HX711_SCALE` with a known weight. Without it, residents type the weight on the keypad.
