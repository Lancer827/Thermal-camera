# Dart1 Thermal Camera Firmware (v3.0)

Firmware for the Dart1 Thermal Camera module utilizing an ESP32 micro-controller and a 5-button navigation interface. Features full runtime configuration over UART protocol, non-volatile parameter persistence, electronic zoom, and optical alignment via X/Y offset calibration.

##  Pinout & Hardware Controls Summary

### Hardware Connections

* **NAV_LEFT_PIN**: `GPIO 0` (Internal Pullup)

* **NAV_RIGHT_PIN**: `GPIO 1` (Internal Pullup)

* **THUMB_BUTTON**: `GPIO 2` (Internal Pullup)

* **EXTRA_BUTTON_PIN**: `GPIO 6` (Internal Pullup)

* **NAV_MID_PIN**: `GPIO 7` (Internal Pullup)

* **SCREEN_BRIGHT_PLUS**: `GPIO 9`

* **SCREEN_BRIGHT_MINUS**: `GPIO 10`

* **Camera RX / TX**: `GPIO 20` / `GPIO 21` (Serial1 @ 115200 8N1)

### Controls Quick Reference

| Button | Primary Action | Shift Action (Hold Thumb Button) | 
| ----- | ----- | ----- | 
| **Nav Left / Right** | Electronic Zoom Level (1.0x – 7.0x) | Adjust ISP Brightness / Luma (0 – 100) | 
| **Nav Middle** | Cycle Pseudocolor Palette (Slots 0 – 5) | Cycle ISP Contrast (Short Press <2s)  Manual OOC Calibration (Long Press >3s) | 
| **Thumb Button** | Manual Shutter Calibration (30ms – 500ms) | *Acts as SHIFT key for secondary actions* | 
| **Extra Button** | Toggle Auto-Shutter Cal (Short Press <1.0s)  Toggle Offset Cal Mode (Long Press >1.5s) | **Extra + Nav Left/Right**: Adjust Microdisplay Brightness | 

##  Operating Instructions

### 1. Electronic Zoom & Offset Constraints

* **Controls:** Press **Nav Left** or **Nav Right**.

* **Behavior:** Adjusts electronic zoom between **1.0×** and **7.0×**.

* **Dynamic Bounds:** When X/Y offsets are active, the minimum selectable zoom level is automatically constrained so that the cropped window stays strictly within physical sensor bounds ($640 \times 512$).

### 2. Microdisplay Screen Brightness

* **Controls:** Hold **Extra Button** + Press **Nav Left** or **Nav Right**.

* **Behavior:** Sends active-low pulses to `SCREEN_BRIGHT_PLUS` / `SCREEN_BRIGHT_MINUS` lines to step display brightness up or down.

### 3. Sensor Image Brightness (Luma)

* **Controls:** Hold **Thumb Button** + Press **Nav Left** or **Nav Right**.

* **Behavior:** Adjusts ISP luma setting from `0` to `100` in steps of 2.

### 4. Sensor Image Contrast

* **Controls:** Hold **Thumb Button** + Tap **Nav Middle** (press < 2s).

* **Behavior:** Cycles through 8 fixed contrast steps (`13`, `25`, `38`, `50`, `63`, `75`, `88`, `100`).

### 5. Thermal Pseudocolor Palette

* **Controls:** Tap **Nav Middle** (standalone press < 2s).

* **Behavior:** Cycles through up to 6 pseudocolor LUT palettes (White Hot, Black Hot, Rainbow, Iron Red, etc.).

### 6. Shutter Calibration

* **Manual NUC / Shutter:** Tap **Thumb Button** (30ms – 500ms duration). Runs a physical shutter cycle regardless of whether automatic calibration is enabled.

* **Manual OOC:** Hold **Thumb Button** + Hold **Nav Middle** for **> 3 seconds**. Triggers a logical Out-Of-Center calibration.

* **Toggle Auto-Calibration:** Short press **Extra Button** (< 1.0s) to toggle automated shutter calibration ON/OFF.

##  X/Y Offset Calibration Mode

Offset Calibration allows precise alignment of the thermal core's optical center relative to external optics or scope reticles.

### Mode Operation

1. **Enter / Exit:** Long-press **Extra Button** for **> 1.5 seconds**.

2. **Horizontal X-Offset:** Press **Nav Left** or **Nav Right** (step size defined by `CALIBRATION_MULTIPLIER = 1`).

3. **Vertical Y-Offset:** Hold **Thumb Button** + Press **Nav Left** or **Nav Right**.

4. **Auto-Saving:** Exiting calibration mode automatically commits offset values to non-volatile memory (`Preferences`).

##  Non-Volatile Memory (NV-RAM)

All operational settings automatically persist to ESP32 flash memory via the `Preferences` library and restore on boot:

* `autocal`: Auto-shutter state

* `brightness`: ISP luma setting

* `screenBright`: Microdisplay hardware brightness level

* `contrastIdx`: Active contrast preset index

* `pseudocolor`: Active thermal palette slot

* `zoomX10`: Electronic zoom level

* `offsetX` / `offsetY`: X and Y optical offset coordinates

##  Serial Debug Console

Serial logging is controlled by the macro definition at the top of `Dart_firmware_3_0.ino`:

```
#define VERBOSE 1 // Set to 1 for debug logging, 0 for quiet operation

```

When enabled (`1`), serial output at **115200 baud** provides ASCII visual feedback during calibration, frame hex dumps, and state change logs.
