# AS5600 ESP-IDF Driver

A component driver for the AMS AS5600 contactless magnetic rotary position sensor, built for ESP-IDF (v6.0.3) using the new `i2c_master` API and FreeRTOS software timers + task notifications for sampling.

Part of an internal Thryv Mobility project. Work in progress — revised as features are added.

---

## Status: Stage 1 + Stage 2 complete

- [x] I2C bus + device init (`as_init`)
- [x] Angle reads: `ANGLE` (filtered) and `RAW_ANGLE` (unfiltered)
- [x] Magnitude read (`AGC`-adjusted field strength via `MAGNITUDE` register)
- [x] Status-gated reads — angle/magnitude calls check `STATUS` (MD/ML/MH) before returning data, and fail with a specific error code instead of returning garbage
- [x] Config registers: `ZPOS`, `MPOS`, `MANG` (12-bit R/W)
- [x] `CONF` register (14-bit R/W) — packed/unpacked via a plain struct (not C bitfields, to avoid compiler-dependent bit ordering on the wire)
- [x] Software zero offset (`as_zero_here`, `as_set_soft_zero`) — in-RAM angle offset, does **not** touch the chip's OTP memory
- [ ] Mutex protection for concurrent access (next)
- [ ] `BURN` command (permanent zero/config commit) — deliberately not implemented yet; OTP writes are irreversible and limited (3 angle burns, 1 config burn, per datasheet)
- [ ] NVS persistence for zero offset across reboots
- [ ] Velocity estimation (angle delta / time, with wraparound handling)
- [ ] Custom `as5600_err_to_name()` for logging

---

## Hardware

- **Sensor:** AS5600, I2C mode, fixed 7-bit address `0x36`
- **Magnet:** diametrically magnetized disc, mounted above the chip
- **Air gap:** empirically tuned per magnet (see Notes below) — datasheet reference range is 0.5–3mm for a 6mm magnet; smaller magnets need a tighter gap
- **Wiring (default config in this repo):** SDA → GPIO21, SCL → GPIO22, 100kHz — override via `as5600_config_t` at init, not hardcoded in the driver

---

## API overview

### Init

```c
as5600_config_t config = {
    .i2c_port = I2C_NUM_0,
    .sda_num  = 21,
    .scl_num  = 22,
    .scl_hz   = 100000,
};
as5600_t dev = {0};
as_init(&config, &dev);
```

`as_init` owns I2C bus creation internally — callers never touch `i2c_master_bus_config_t`/`i2c_device_config_t` directly.

### Reading

| Function | Returns | Notes |
|---|---|---|
| `as_get_angle(dev, float *angle)` | filtered angle, 0–360° | reads `ANGLE` register |
| `as_get_angle_r(dev, float *angle)` | raw angle, 0–360° | reads `RAW_ANGLE`; applies software zero offset |
| `as_get_mag(dev, uint16_t *mag)` | 0–4095 | field strength magnitude |

All three check `STATUS` first and return one of the error codes below instead of stale/garbage data if the magnet isn't in a valid state.

### Config registers (12-bit: ZPOS/MPOS/MANG)

```c
esp_err_t as_set_zero_position(as5600_t *dev, uint16_t raw_value);
esp_err_t as_get_zero_position(as5600_t *dev, uint16_t *out);
esp_err_t as_set_max_position(as5600_t *dev, uint16_t raw_value);
esp_err_t as_get_max_position(as5600_t *dev, uint16_t *out);
esp_err_t as_set_max_angle(as5600_t *dev, uint16_t raw_value);
esp_err_t as_get_max_angle(as5600_t *dev, uint16_t *out);
```

**These write to the chip's ZPOS/MPOS/MANG registers directly (RAM, not OTP) — safe to experiment with, but not persistent across power loss until burned.** Burning is not implemented in this driver yet.

### CONF register (14-bit)

```c
typedef struct {
    uint8_t power_mode;          // 0=NOM, 1=LPM1, 2=LPM2, 3=LPM3
    uint8_t hysteresis;          // 0=off, 1=1LSB, 2=2LSB, 3=3LSB
    uint8_t output_stage;        // 0=analog full, 1=analog reduced, 2=PWM
    uint8_t pwm_freq;            // 0=115Hz, 1=230Hz, 2=460Hz, 3=920Hz
    uint8_t slow_filter;         // 0=16x, 1=8x, 2=4x, 3=2x
    uint8_t fast_filter_threshold; // 0=slow filter only, 1..7=thresholds
    bool    watchdog_en;
} as5600_conf_reg_t;

esp_err_t as_set_conf(as5600_t *dev, const as5600_conf_reg_t *conf);
esp_err_t as_get_conf(as5600_t *dev, as5600_conf_reg_t *conf);
```

Bit layout (see `as_get_conf` implementation for the authoritative mapping):

```
Bit 13      WD    Watchdog
Bits 12:10  FTH   Fast Filter Threshold
Bits 9:8    SF    Slow Filter
Bits 7:6    PWMF  PWM Frequency
Bits 5:4    OUTS  Output Stage
Bits 3:2    HYST  Hysteresis
Bits 1:0    PM    Power Mode
```

### Software zero offset

```c
esp_err_t as_zero_here(as5600_t *dev);              // "wherever it's pointing now = 0°"
esp_err_t as_set_soft_zero(as5600_t *dev, uint16_t raw_offset);  // set a known offset directly
```

Purely in-RAM — resets to 0 on every reboot/power cycle. Applied only inside `as_get_angle_r` currently. Does not touch the chip's OTP; safe to call as often as needed. See "Design notes" below for why this exists instead of burning `ZPOS`.

### Error codes

```c
#define AS5600_ERR_BASE                 0x9000
#define AS5600_ERR_MAGNET_NOT_DETECTED  (AS5600_ERR_BASE + 1)
#define AS5600_ERR_MAGNET_TOO_WEAK      (AS5600_ERR_BASE + 2)
#define AS5600_ERR_MAGNET_TOO_STRONG    (AS5600_ERR_BASE + 3)
```

Plain `esp_err_t`-compatible values — usable directly in `if (ret == AS5600_ERR_...)` alongside standard ESP-IDF codes. Standard I2C failures (timeout, NACK, etc.) pass through as the underlying `esp_err_t` from the I2C driver, unmodified.

---

## Design notes

- **Two 12-bit read/write helpers, two 14-bit ones** (`read_12b_reg`/`write_12b_reg`, `read_14b_reg`/`write_14b_reg`) — all register access funnels through these four. Mask width differs (`0x0F` high-byte mask for 12-bit registers, `0x3F` for the 14-bit `CONF`), so don't call the wrong pair for a given register.
- **No C bitfields used for `CONF`** — deliberate choice. Bit ordering inside a C bitfield isn't guaranteed by the standard (compiler/platform-defined), which is a real risk for a value going out over a wire protocol. Manual shift/mask into a plain struct is slower to write but portable.
- **Status-gating lives inside the read functions**, not left to the caller — `as_get_angle`/`as_get_angle_r`/`as_get_mag` all check `STATUS` internally and refuse to return angle/magnitude data if the magnet isn't in a valid state, rather than handing back stale or undefined numbers.
- **AGC-based air-gap tuning**: for the specific magnet in use, air gap was tuned empirically by watching `AGC` while adjusting physical spacing — target AGC near mid-scale for the supply rail (≈64 at 3.3V, ≈128 at 5V), with `MH`/`ML` both 0 through a full rotation, not just at one static point.

---

## Known gaps / TODO

- Not thread-safe yet — concurrent calls into `dev->dev_handle` from multiple tasks will race. Mutex wrap planned as the next stage.
- `as_get_angle` (filtered) does not currently apply the software zero offset — only `as_get_angle_r` does. Revisit if both should be consistent.
- No persistence — zero offset and any CONF changes are lost on reset.
- `BURN` not implemented — intentional, given the OTP burn-count limits.

---

## File layout

```
components/as5600/
├── CMakeLists.txt
├── include/
│   └── as5600.h
└── as5600.c
```