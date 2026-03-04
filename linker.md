# BetaSpray Custom Linker Script Guide

## Overview

This guide explains how to use a custom linker script (`betaspray_linker.ld`) to explicitly control where buffers and data sections are allocated in the ESP32-S3's memory.

## Why Custom Linker Scripts?

The default ESP-IDF linker script places all `.bss` and `.data` sections automatically. A custom script gives you:

- **Explicit control** over which objects go where
- **Memory layout visibility** - see exactly what's in DRAM vs Flash
- **Prevent fragmentation** - allocate large buffers early
- **Performance optimization** - colocate related data
- **Debugging** - know exact addresses of buffers

## ESP32-S3 Memory Regions

```
IRAM (Instruction):   0x40370000 - 0x403E0000  (448 KB)
DRAM (Data):          0x3FC88000 - 0x3FD00000  (480 KB)  ← Where .bss goes
Flash Code:           0x42000020 - 0x42800000  (8 MB)
Flash Data (RODATA):  0x3C000020 - 0x3E000000  (32 MB)
RTC Fast:             0x600FE000 - 0x60100000  (8 KB)
RTC Slow:             0x50000000 - 0x50002000  (8 KB)
```

## Creating `betaspray_linker.ld`

### Basic Structure

```ld
/* BetaSpray Custom Linker Script */

SECTIONS
{
  /**
   * Section name : { contents } > memory_region
   *
   * NOLOAD   = Don't initialize (zero) this section
   * ALIGN(n) = Align to n-byte boundary
   * >        = Place in memory region
   */

  .camera_bss (NOLOAD) :
  {
    . = ALIGN(4);
    _camera_bss_start = ABSOLUTE(.);

    /* Match object files */
    *camera.o(.bss .bss.* COMMON)

    . = ALIGN(4);
    _camera_bss_end = ABSOLUTE(.);
  } > dram0_0_seg;

  .servo_bss (NOLOAD) :
  {
    . = ALIGN(4);
    _servo_bss_start = ABSOLUTE(.);

    *servo.o(.bss .bss.* COMMON)

    . = ALIGN(4);
    _servo_bss_end = ABSOLUTE(.);
  } > dram0_0_seg;
}
```

## Memory Region Aliases (Available in ESP-IDF)

- `iram0_0_seg` - Instruction RAM (executable code)
- `iram0_2_seg` - Flash-mapped instructions
- `dram0_0_seg` - **Data RAM** (where .bss, .data, heap, stack go)
- `drom0_0_seg` - Flash-mapped read-only data
- `rtc_iram_seg` - RTC fast RAM (persists in deep sleep)
- `rtc_slow_seg` - RTC slow RAM

## Matching Patterns

### Match specific files:
```ld
*camera.o(.bss)          /* All .bss in camera.o */
*camera.c.o(.bss)        /* Specifically camera.c.o */
*servo*.o(.bss)          /* Wildcard: servo.o, servo_driver.o, etc */
```

### Match specific symbols:
```ld
*(.camera_section)       /* Symbols in custom section */
*(.bss .bss.*)           /* All .bss variants */
```

### Match multiple object files:
```ld
*camera.o(.bss)
*vision.o(.bss)
*(.bss)                  /* Catch-all for anything else */
```

## Example: Camera Buffer Allocation

### In `camera.h`, mark the buffer:
```c
#define CAMERA_BSS_BUF_SIZE (40 * 1024)
```

### In `camera.c`, define the buffer:
```c
/* This will match *camera.o(.bss) in the linker script */
static uint8_t s_bss_buf[CAMERA_BSS_BUF_SIZE];
```

### In `betaspray_linker.ld`:
```ld
.camera_bss (NOLOAD) :
{
  . = ALIGN(4);
  _camera_bss_start = ABSOLUTE(.);

  *camera.o(.bss .bss.* COMMON)

  . = ALIGN(4);
  _camera_bss_end = ABSOLUTE(.);
} > dram0_0_seg;
```

### Access addresses in code:
```c
extern uint8_t _camera_bss_start;
extern uint8_t _camera_bss_end;

void show_camera_buffer_address(void) {
  uint32_t start = (uint32_t)&_camera_bss_start;
  uint32_t end = (uint32_t)&_camera_bss_end;
  size_t size = end - start;

  printf("Camera BSS: 0x%08lx - 0x%08lx (%lu KB)\n",
         start, end, size / 1024);
}
```

## Enabling the Linker Script

### Method 1: platformio.ini
```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = espidf

build_flags =
    -Wl,--script=betaspray_linker.ld
```

### Method 2: linker_flags in platformio.ini
```ini
build_flags =
    -Wl,-Map=build_output.map
    -Wl,--script=betaspray_linker.ld
    -Wl,--gc-sections
```

### Method 3: CMakeLists.txt (if using ESP-IDF directly)
```cmake
target_link_options(${COMPONENT_NAME} PRIVATE
    -Wl,--script=${CMAKE_CURRENT_LIST_DIR}/betaspray_linker.ld)
```

## Verifying the Layout

### Generate a memory map:
```bash
pio run -t clean && pio run -b esp32-s3-devkitc-1 -e esp32-s3-devkitc-1
```

### Examine symbol addresses:
```bash
xtensa-esp32s3-elf-nm .pio/build/esp32-s3-devkitc-1/firmware.elf | grep -E "_bss_start|_bss_end|s_bss_buf"
```

Expected output:
```
3fc9c000 d _camera_bss_start
3fca9000 d _camera_bss_end
3fc9c000 b s_bss_buf
```

### Check linker map file:
```bash
cat .pio/build/esp32-s3-devkitc-1/firmware.map | grep -A 5 "camera.o"
```

## Complete Example: Multi-Buffer Layout

```ld
/* betaspray_linker.ld - BetaSpray Memory Layout */

SECTIONS
{
  /* 1. Camera frame buffer - 40 KB, allocated first */
  .camera_bss (NOLOAD) :
  {
    . = ALIGN(4);
    _camera_bss_start = ABSOLUTE(.);
    *camera.o(.bss .bss.* COMMON)
    . = ALIGN(4);
    _camera_bss_end = ABSOLUTE(.);
  } > dram0_0_seg;

  /* 2. Servo control structures - smaller, needs alignment for DMA */
  .servo_bss (NOLOAD) :
  {
    . = ALIGN(8);  /* 8-byte alignment for safety */
    _servo_bss_start = ABSOLUTE(.);
    *servo.o(.bss .bss.* COMMON)
    . = ALIGN(4);
    _servo_bss_end = ABSOLUTE(.);
  } > dram0_0_seg;

  /* 3. WiFi/HTTP server buffers */
  .wifi_bss (NOLOAD) :
  {
    . = ALIGN(4);
    _wifi_bss_start = ABSOLUTE(.);
    *wifi.o(.bss .bss.* COMMON)
    *server.o(.bss .bss.* COMMON)
    . = ALIGN(4);
    _wifi_bss_end = ABSOLUTE(.);
  } > dram0_0_seg;

  /* 4. LED strip driver data */
  .led_bss (NOLOAD) :
  {
    . = ALIGN(4);
    _led_bss_start = ABSOLUTE(.);
    *led_strip.o(.bss .bss.* COMMON)
    . = ALIGN(4);
    _led_bss_end = ABSOLUTE(.);
  } > dram0_0_seg;

  /* 5. Catch-all for remaining BSS (heap will start after this) */
  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _bss_start = ABSOLUTE(.);
    *(.bss .bss.* COMMON)
    . = ALIGN(4);
    _bss_end = ABSOLUTE(.);
  } > dram0_0_seg;
}
```

## Debugging Memory Issues

### Check if something isn't being linked:
```bash
xtensa-esp32s3-elf-readelf -S .pio/build/esp32-s3-devkitc-1/firmware.elf | grep bss
```

### Disassemble to find symbols:
```bash
xtensa-esp32s3-elf-objdump -t .pio/build/esp32-s3-devkitc-1/firmware.elf | grep camera
```

### Check if sections actually exist:
```bash
xtensa-esp32s3-elf-objdump -h .pio/build/esp32-s3-devkitc-1/lib*/libcamera.a
```

## Common Issues

### Issue: Linker script not being used
**Solution:** Check `platformio.ini` has correct path and PlatformIO cached old build. Run `pio run -t clean`.

### Issue: Symbol not found (segfault)
**Solution:** Symbol might be in a different object file. Check with `nm` or adjust wildcard pattern.

### Issue: Out of memory
**Solution:** Check section sizes with `_camera_bss_end - _camera_bss_start`. Reduce `CAMERA_BSS_BUF_SIZE` if needed.

### Issue: Linker warnings about unrecognized sections
**Solution:** Ensure section names match what the compiler generates (check with `objdump -h`).

## References

- [ESP-IDF Linker Script Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/linker-script-generation.html)
- [GNU LD Manual - Linker Scripts](https://sourceware.org/binutils/docs/ld/Scripts.html)
- ESP32-S3 TRM: Memory Organization
