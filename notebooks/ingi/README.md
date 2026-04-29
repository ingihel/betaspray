---
title: Engineering Notebook (ECE 445 Spring 26)
author: Ingi Helgason
---

## April 27

We set the camera mounts in CA glue for better integrity. TODO

## April 26

We tested the design at the gym. TODO

## April 25

It was mostly a work day and I forgot to write much so far. TODO

We tested the design with our newly manufactured testing board for the demo, and
developed a new calibration routine for the camera, courtesy of Max.

## Evening of April 24

Pasted the printouts onto the trifold, ready for testing tomorrow!

## April 24

Due to repeated unavailability of the large format printer at Siebel Center for Design
(SCD), I ended up creating and poster-slicing a life-scale document with a variety
of real climbing holds, using high-resolution scale model/image uploads from the company
So iLL for the purpose of our demo.

This print ended up being surprisingly expensive (about $14); in hindsight, it would have
been good to get to SCD earlier in the week. The vast majority of prints in the queues when
we visited were elaborate, multi-hour jobs, preventing us from using it almost entirely.
Not sure what caused such a surge in usage this time: it's generally been available whenever I
try to go.

Anyways, with the poster printed and sliced, all that remains is to attach it to the trifold (a
task for tonight).

## April 22

We ended up re-doing the software work from the previous note, but it is left for posterity's
sake.

## April 18-19

This is a slightly longer note, since we were working separately before convening to work
on the project. Further, it is code-related, so there are easy samples to include.

We have four aiming gimbals, each a 2-DOF pan/tilt head driven by a pair of SG90 servos.
The gimbals are bolted to the frame at different heights relative to the camera.

Until now the firmware assumed every gimbal shares the camera's optical axis, which worked
fine when we only had one gimbal with some transform wrt the lens.  With four heads this breaks:
further, a gimbal sitting 15–30 cm above the camera physically cannot aim at low wall holds without
going past its mechanical stop, and the mapping math doesn't know that.

The current pixel -> servo conversion lives in `src/route.c` and looks like this:
```c
    static int pixel_to_servo_y(float py) {
        float angle_deg = (s_transform.image_height - py)
                          * (s_transform.vfov_deg / s_transform.image_height);
        int servo_angle = (int)(90.0f + angle_deg);
        return servo_angle < 0 ? 0 : servo_angle > 180 ? 180 : servo_angle;
    }
```

There is no per-gimbal adjustment here — every head gets the same angle for the same pixel.
The clamp is also hardcoded to [0, 180] regardless of where a gimbal is mounted; this full range
is not necessarily used.

### SIDE NOTE; camera calibration

Prakhar is currently working on the Camera Distortion problem, so re-evaluation of the correctness
of this change will wait until integration testing occurs for these system changes.

### Problem, expanded

If gimbal is mounted h metres above the camera and the wall is d metres away, the gimbal's
optical axis points at the wall at an angle:
```
    tilt_offset_rad = arctan(h / d)
```

For our rig with d approximately equal to 3 m and h values of roughly 0, 5, 15, and 30 cm:
```
    gimbal 0:  atan(0.00 / 3) \approx  0.0 deg -> (theoretical perfect, not really attained)
    gimbal 1:  atan(0.05 / 3) \approx  1.0 deg -> (negligible)
    gimbal 2:  atan(0.15 / 3) \approx  2.9 deg -> use a larger value (empirical)?
    gimbal 3:  atan(0.30 / 3) \approx  5.7 deg -> use a larger value?
```

The "in practice" numbers will need empirical calibration — the arm geometry adds extra
angular throw.  The `tilt_offset_deg` field in the struct below is that calibrated value.

Keep in mind that with the currently planned enclosure geometry, the height above the camera is no
more than 30 mm; this work might be a bit excessive for the vertical dimension, and is more relevant
for side-to-side near-clustered holds.

Working in one axis basically makes it worth it to work in the other axis too.

For vertical (tilt), the solution is to compute
```
    angle_servo = angle_from_camera - tilt_offset_deg
```

which can be subsequently clamped by the safe-hard-stops selected. Similarly, the camera reference
frame can be approximately re-computed into the laser reference frame for the horizontal axis.

### Proposed data structure for each gimbal mount

    typedef struct {
        float pan_offset_deg;   // gimbal pan centre vs camera optical axis (+right)
        float tilt_offset_deg;  // gimbal tilt centre vs camera optical axis (+up)
        int   min_pan_angle;    // servo hard limits (degrees, [0,180])
        int   max_pan_angle;
        int   min_tilt_angle;   // raised mounts: this floor is > 0
        int   max_tilt_angle;
    } gimbal_mount_t;

    void route_set_gimbal_mount(int g, const gimbal_mount_t *m);

Can also keep a table of these values in `src/route.c`: four entries, one per laser gimbal.
We can go fully off what's in the file and adjust these on-the-fly over HTTP if they are a
configuration hassle

```c
    static gimbal_mount_t s_gimbal_mounts[4] = {
        // g0: co-axial with camera
        { .pan_offset_deg = 0,  .tilt_offset_deg =  0,
          .min_pan_angle = 0,   .max_pan_angle = 180,
          .min_tilt_angle = 0,  .max_tilt_angle = 180 },

        // g1: slight right offset, same height
        { .pan_offset_deg = 5,  .tilt_offset_deg =  0,
          .min_pan_angle = 0,   .max_pan_angle = 175,
          .min_tilt_angle = 0,  .max_tilt_angle = 180 },

        // g2: higher mount — cannot point far downward
        { .pan_offset_deg = 0,  .tilt_offset_deg = 15,
          .min_pan_angle = 0,   .max_pan_angle = 180,
          .min_tilt_angle = 20, .max_tilt_angle = 180 },

        // g3: highest mount — most restricted downward range
        { .pan_offset_deg = 0,  .tilt_offset_deg = 30,
          .min_pan_angle = 0,   .max_pan_angle = 180,
          .min_tilt_angle = 35, .max_tilt_angle = 180 },
    };
```

## April 16

The enclosure design initially created has some tolerance issues. Some of these I am currently
chalking up to the table physically shaking due to the lack of securement and the Bambu printers,
which is actually a pretty funny source of error.

Overall, I decided to go for a second revision with a set of around 5 major changes:
- Remove some low-yield fillets (change dimensions)
- Flip orientation in the slicer to save some print type, at a slight expense of filament
- Thicken front camera mount to avoid snapping
- Fix tolerances on main enclosure shell
- Fix tolerances on M2.5 and M5 screw holes for better strength after self-tapping

## April 15

I ended up working on the revised enclosure on this day. The main deficiencies in the original design
which we wanted to address were the following:

- Standoff holes for the PCB are too small (M3 inner diameter was used as the outer diameter for standoffs).
- Camera mount does not work with the length of cable alotted, and lacks sufficent mounting hardware.
- The laser gimbal mounts are too close to one another, meaning thrashing/physical conflict between the gimbals
  will inherently occur.
  - Regarding this point: laser I/O (transistor-signaled from 3V3 onto the 5V power line) is somewhat inconveniently
    placed, requiring long cables. This does not prevent any impedance (no pun intended) to the project working as
    expected, it's just a little inconvenient.
- Drilling hole for antenna (~M6 threaded hole) turned out to not be practical
- Self-tapping holes are slightly misspaced

While I am being over-critical here, the general gist of what I am trying to say is that we have things that we are
going to work to improve.

While not directly related to the printed product, there is also another issue I am stricken with; Fusion 360 over
the web is quite slow, and I don't meet the system requirements to run it locally.

After some investigation, I determined that Onshape would be a suitable alternative. The controls/manipulation is
pretty familiar to SolidWorks, so it does not prevent too much of a learning hassle. I will finish the rest of the design
on this evening, and aim to print it whenever the resource in the lab is available.

## April 14

We worked on integration, in 2070 first. I don't have many notes from the day; we were working in the ECEB atrium.

Please infer that we did, in fact, do work.

Key tasks that we were working on was the conversion of the ImageJ pipeline to something embeddable as browser middleware,
for a more fluent user experience.

## April 10

I spent most of this day sourcing mechanical parts for assembly. The spare components needed to finish the board
arrived on this day, so we scheduled a meeting for April 14th.

## April 7

This day was the breadboard demo. We didn't really do work on this day, though after the demo we realized
that defining the plan for printing the poster would be wise (especially with upcoming conferences).

## April 6

We finally got the camera to work with the faulty PCB. In a funny act of foreshadowing, it turned
out that the cable length was an issue.

Before fixing, we saw a lot of errors of the following form:
```
cam_hal: EV-VSYNC-OVF
cam_hal: NO-SOI - JPEG start marker missing
cam_hal: NO-SOI - JPEG start marker missing
cam_hal: NO-EOI - JPEG end marker missing
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: NO-SOI - JPEG start marker missing - 100 additional misses
cam_hal: NO-SOI - JPEG start marker missing - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: NO-SOI - JPEG start marker missing - 100 additional misses
cam_hal: NO-SOI - JPEG start marker missing - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
cam_hal: EV-VSYNC-OVF - 100 additional misses
```

We deduced that these were due to poor signal integrity: key changes were to adjust the
wire length (involved re-cutting and splicing the ribbon cable) and to adjust the
clock divider / JPEG quality factor to eliminate subsequent JPEG start- / end-packet
integrity issues.


## April 5

Fixed the USB-C controller by replacing and re-soldering. It presents and persists permanently
under the previously-designed `watch -n 5 -g lsusb` evaluation method (re-runs `lsusb` regularly
and exits once it finds a new device, the Espressif USB device).

We went on to test flashing, getting it to post the network, and it all worked as it did with
the previous board.

There were some complaints about board parameters not matching so I updated the `platformio.ini`
file, and afterwards, the "default upload parameters" section to make it work.

```ini
[env:esp32s3]
platform  = espressif32
board     = esp32-s3-devkitc-1
framework = espidf

board_build.flash_size    = 4MB
board_build.partitions    = partitions.csv
board_build.arduino.memory_type = qio_opi
monitor_speed             = 115200
upload_speed              = 921600

[env:esp32s3.upload]
upload_protocol               = esptool
board_upload.use_1200bps_touch = false
board_upload.wait_for_upload_port = false
```


## April 2

Had issues with LDO supply, since the E-Shop form was not up to date.

I contacted our TA and we looked to see if there were spare parts for our board, to no avail.

We did an assembly test after removing components from other test boards we had made, but
ended up stuck after snapping a USB conector.

## March 31

Finalized hold detection/interface, making it perform significantly better.

The main change was switching from a single fixed intensity threshold to a per-frame adaptive
threshold derived from the image histogram median. This eliminated most false positives on
textured/shadowed walls. The hold outline pass was also tightened: minimum contour area was
raised so that specular glints and small debris no longer register as candidates. End-to-end
latency for a 320×240 JPEG frame improved noticeably after removing a redundant decode step
that was capturing the same frame twice. (This was confirmed on the breadboard, though the
breadboard/devkit setup is not being used much right now due to the need to prepare for the
custom PCB arrival and associated development effort).


## March 27

More initial enclosure work.

Most of the work is basically just CAD work, I attach some images of the CAD in an `img` folder to
this repository.


## March 25

Focused on doing DRC fixes and placing the new PCBWay order for the last-round board.

After thinking about it a little more we have realized we aren't dead in the water with
our other board; trace length minimizing/matching issues should not be too severe at 25 MHz.

Quick sanity check: at 25 MHz the clock period is T = 40 ns. FR4 has εr ≈ 4.5, giving a
propagation velocity of roughly v = c / √εr ≈ 14 cm/ns. A 1 cm trace-length mismatch
therefore introduces a skew of ~71 ps — about 0.18 % of T. Even a worst-case 5 cm mismatch
across the DVPI data bus is only ~0.9 % of the period, well inside any receiver hold-time
budget. So the board should work fine at this speed without active length matching.

We should be careful to have the cable not be too long so that the λ/10 assumption
(no TL effects) holds roughly. I suspect that having a long cable as in the case of the breadboard
or longer would be an issue for integrity.

## March 24

Contacted Gregg to make sure he understands we aren't going to need the E-Shop manufacture
services for our project (regarding the contents of the previous note)


## March 23

Noticed pitch issue on 9-pin connectors on board.

After I went through the process of ordering some back-up parts for testing/adapting (may
not be needed but would be good to have), we also had a pretty lengthy discussion about
physical enclosure assembly.

We concluded that for a variety of reasons we would benefit greatly from creating our own
3D printed parts, fittings, adapters, etc.

For this reason, we concluded that we may have minimal need for the E-Shop manufacturing
capabilities. We decided to think about it separately before making a rash decision but
were generally in agreement about this small re-direction.

# SPRING BREAK

## March 11

More software work; defining remaining tasks before break.

## March 8

Homing tests on board with Max in the ECEB lobby.

Identified the need for hard-stop zeroing (or lack thereof) through repeat tests.

Test procedure: we arranged n = 20 holds in a 4×5 grid on a flat whiteboard, 2-inch centre-to-centre
spacing. The board was placed at roughly 3 m from the rig. We commanded the system to aim at
each hold in sequence, recorded the servo angles dispatched, and compared the laser dot landing
position visually (and, where possible, with a tape measure). Homing was re-run between each
full sweep to isolate cumulative drift.

Key finding: without hard-stop zeroing the gimbal's idea of 0° drifted over successive runs,
causing the far corners of the grid to be missed by an increasing margin. This makes a
proper hard-stop homing sequence a hard requirement before any accuracy work.

Low-resolution image capture at this distance made it difficult to confirm sub-centimetre
accuracy purely from the camera feed; we used the physical laser dot as ground truth instead.


## March 7

Image processing loop testing. I tried creating an ESP32-compatible DoG method.
Kinda bad, I'll explain how the method worked.

DoG (Difference of Gaussians) approximates a Laplacian-of-Gaussian blob detector by
subtracting two blurred versions of the same image at different scales. The idea was
to detect holds as bright blobs on the wall. The ESP32-compatible version I wrote replaced
the true Gaussian kernels with small integer-coefficient box-filter approximations (cheaper
on a microcontroller with no FPU in the hot path). The result was noisy — the integer
approximation introduced enough ringing that small highlights and texture seams registered
alongside real holds. Ultimately scrapped in favour of a simpler adaptive-threshold
contour approach.

On the memory side: the two intermediate greyscale buffers for a 320×240 frame are ~75 KB
each, which exhausts internal SRAM almost instantly. The fix was to allocate them from PSRAM
using `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)` instead of plain `malloc`. Once we figured
out that `malloc` silently falls back to internal SRAM (and fails at this size), everything
worked. We wrapped the allocations in an assert so it fails loudly if PSRAM is not available.

---
#### More about the method (long form post?)

After several days of work, the Difference of Gaussian test program is ready:
```c
/**
 * 
 * differenceofgaussian.c
 *
 * author: Ingi Helgason
 * 
 * Compile with:
 * 
 * $ gcc differenceofgaussian.c -o differenceofgaussian -lm -ljpeg -O3 
 * 
 * Requires libjpeg-dev (Ubuntu)
 * 
 *
 * SOURCES:
 * 
 * I have not designed the algorithmic procedure implemented here. The literature is far beyond my comprehension,
 * this is simply my attempt at implementation (cobbled together from extensive forum/document scrounging, absorbing
 * keywords and hopefully the main ideas blah blah).
 * 
 * This file implements a Difference of Gaussian filter, based approximately on a provided OpenCV
 * sample implementation found HERE: {https://github.com/jrdi/opencv-examples/blob/master/dog/main.cpp}
 * General Algorithm Geneaology:
 * This is meant as a less computationally-intensive version of a very popular method of edge detection known as
 * Laplacian of Gaussian (LoG): SOURCE: {https://homepages.inf.ed.ac.uk/rbf/HIPR2/log.htm}
 * 
 * 
 * 
 * The key step is to apply Gaussian blurs at different sizes. This is done inside of `make_gauss_kernel`, where the kernel
 * is truncated at DOG_KERN_TRUNC sigma from its mean. (This value can be tuned down, at the expense of potentially
 * introducing some distortion from "imperfect Gaussian computation")
 * 
 * We blur the image with Gaussian filters with Gaussian(sigma), ...,  Gaussian(2^{1 / n} * sigma)
 * We take a total of DOG_SCALES_PER_OCTAVE such applications of the Gaussian kernel
 * - Then, we take the difference of adjacent ones, which is what approximates the Laplacian
 * This means we have a total of DOG_SCALES_PER_OCTAVE-1 Gaussian-Differences (hereon referred to as `DG`-s)
 * 
 * 
 * We apply this to the image to compute the DG_i, then we downsample by a factor of 2 with a Box blur
 * , i.e. using the matrix 0.25 . O, where O is the 2x2 matrix of ones.
 * 
 * Note that subsequent downsampled iterations ("octaves") approach triviality for small image sizes.
 * This method applies constant tuned values regardless of image resolution, but would likely benefit fro these changes.
 * 
 * 
 * 
 * We then want to find keypoints.
 * 
 * We look for maxima in a given scale (value of sigma_n = 2^{1/n} sigma), after discarding pixels which do not exceed
 * a maximum level inside is_extremum. (NOTE: filtering aggressively has significant runtime benefits).
 * We necessitate that a given value is larger than any neighbors for any sigma_n [n \in \{0, ..., DOG_SCALES_PER_OCTAVE \}]
 * to be considered a potential center of a blob.
 * 
 * 
 * We then run a greedy algorithm: we sort by the strength of the signal (contrast found above) (any stable, fastquicksort suffices)
 * and annihilate any keypoints (blob centers) within a given DOG_NMS_RADIUS.
 * 
 * 
 * All-in-all, this supposedly extracts blobs in a reasonably accurate way.
 * We cannot test this on our current DEVKIT since it does not have sufficient (any) PSRAM.
 * But we should have enough memory to allocate each of the DG layers (back of the napkin math coming soon, but I concluded that for 320x240 this is definitely doable).
 * There are some miscellaneous optimizations attempted in the C code to accomplish light computational footprint, at the expense
 * of accuracy.
 * 
 * 
 * ---
 * Commentary on accuracy loss: is it okay?
 * My thought is yes, since if accuracy is an issue we may fall back to any other method of off-board image processing analysis.
 * The main reason I try this method is because our pitch/product selling point essentially requires this method to be at least serviceable,
 * so we must try it for our "conscience".
 * 
 * I remember enough digital signals to try and consult literature, make adjustments, understand what's going on here. Can make no such
 * guarantees for a YOLO/similar net deployed to the ESP32. I would rather choose something more comfortable/familiar, so that we have
 * some recourse if things don't work.
 * 
 * End of my log for now. Code follows:
 * 
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <jpeglib.h>

#define DOG_MAX_IMAGE_DIM    2048
#define DOG_MAX_KEYPOINTS    1000
#define DOG_THRESH           2
#define DOG_CONTRAST_MIN     5
#define DOG_NUM_OCTAVES      4
#define DOG_SCALES_PER_OCTAVE 3
#define DOG_SIGMA_INIT       1.6f
#define DOG_KERN_TRUNC       3.0f
#define DOG_NMS_RADIUS       4.0f

typedef struct {
    float x, y;
    float sigma;
    float response;
} dog_keypoint_t;

typedef struct {
    dog_keypoint_t *kps;
    int capacity;
    int count;
} dog_result_t;

/* --- This is a thin shim, which allows this file to be ported to the ESP32-S3 (I think) without too many modifications --- */
#define ESP_OK 0
#define ESP_ERR_NO_MEM -1
#define ESP_ERR_INVALID_ARG -2
typedef int esp_err_t;
#define LOGI(tag, fmt, ...) printf("[INFO] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define LOGE(tag, fmt, ...) fprintf(stderr, "[ERROR] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define LOGD(tag, fmt, ...) // Silence debug or use printf

static void draw_circle(uint8_t *img, int w, int h, int cx, int cy, int r, uint8_t color) {
    for (int y = -r; y <= r; y++) {
        for (int x = -r; x <= r; x++) {
            if (x * x + y * y <= r * r && x * x + y * y >= (r - 1) * (r - 1)) {
                int px = cx + x;
                int py = cy + y;
                if (px >= 0 && px < w && py >= 0 && py < h) {
                    img[py * w + px] = color;
                }
            }
        }
    }
}

void save_ppm(const char *filename, uint8_t *data, int w, int h) {
    FILE *f = fopen(filename, "wb");
    if (!f) return;
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    fwrite(data, 1, w * h, f);
    fclose(f);
}

static void *dog_alloc(size_t n) {
    return malloc(n);
}


/**
 * 
 * 
 * 
 * 
 * 
 * ALGORITHM
 * 
 * 
 * This should be computable on the ESP32-S3 (although it will run slower),
 * since there are no explicit dependencies on OpenCV.
 * 
 * 
 * 
 * 
 */

static const char *TAG = "dog_blob";

#define KERN_MAX_RADIUS  8
#define KERN_MAX_SIZE    (2 * KERN_MAX_RADIUS + 1)

typedef struct {
    float coeffs[KERN_MAX_SIZE];
    int   radius;
    int   size;
} gauss_kern_t;

static void make_gauss_kernel(float sigma, gauss_kern_t *k) {
    int r = (int)ceilf(sigma * (float)DOG_KERN_TRUNC);
    if (r < 1) r = 1;
    if (r > KERN_MAX_RADIUS) r = KERN_MAX_RADIUS;
    const float s2 = 2.0f * sigma * sigma;
    float sum = 0.0f;

    #pragma unroll
    for (int i = 0; i < 2 * r + 1; i++) {
        float x = (float)(i - r);
        k->coeffs[i] = expf(-(x * x) / s2);
        sum += k->coeffs[i];
    }
    const float inv = 1.0f / sum;

    #pragma unroll
    for (int i = 0; i < 2 * r + 1; i++) k->coeffs[i] *= inv;
    k->radius = r; k->size = 2 * r + 1;
}

static inline uint8_t px8(const uint8_t *img, int w, int h, int x, int y) {
    if (x < 0) x = 0; else if (x >= w) x = w - 1;
    if (y < 0) y = 0; else if (y >= h) y = h - 1;
    return img[y * w + x];
}

static void gauss_blur(const uint8_t *src, uint8_t *dst, uint8_t *scratch, int w, int h, float sigma) {
    gauss_kern_t k;
    make_gauss_kernel(sigma, &k);
    #pragma unroll
    for (int y = 0; y < h; y++) {
        #pragma unroll
        for (int x = 0; x < w; x++) {
            float acc = 0.0f;
            for (int ki = 0; ki < k.size; ki++) 
                acc += k.coeffs[ki] * (float)px8(src, w, h, x + ki - k.radius, y);
            int v = (int)(acc + 0.5f);
            scratch[y * w + x] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
        }
    }
    #pragma unroll
    for (int y = 0; y < h; y++) {
        #pragma unroll
        for (int x = 0; x < w; x++) {
            float acc = 0.0f;
            #pragma unroll
            for (int ki = 0; ki < k.size; ki++)
                acc += k.coeffs[ki] * (float)px8(scratch, w, h, x, y + ki - k.radius);
            int v = (int)(acc + 0.5f);
            dst[y * w + x] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
        }
    }
}

static void downsample2x(const uint8_t *src, int sw, int sh, uint8_t *dst) {
    const int dw = sw / 2, dh = sh / 2;
    #pragma unroll
    for (int y = 0; y < dh; y++) {
        #pragma unroll
        for (int x = 0; x < dw; x++) {
            const int sx = x * 2, sy = y * 2;
            const unsigned sum = (unsigned)src[sy*sw+sx] + (unsigned)src[sy*sw+sx+1] +
                                 (unsigned)src[(sy+1)*sw+sx] + (unsigned)src[(sy+1)*sw+sx+1];
            dst[y * dw + x] = (uint8_t)((sum + 2u) >> 2u);
        }
    }
}

static void compute_dog(const uint8_t *hi, const uint8_t *lo, int16_t *dst, int npix) {
    #pragma unroll
    for (int i = 0; i < npix; i++) dst[i] = (int16_t)hi[i] - (int16_t)lo[i];
}

static bool is_extremum(const int16_t *prev, const int16_t *curr, const int16_t *next, int w, int h, int x, int y) {
    const int v = (int)curr[y * w + x];
    if (v > -DOG_THRESH && v < DOG_THRESH) return false;
    const bool looking_for_max = (v > 0);
    #pragma unroll
    for (int ds = 0; ds < 3; ds++) {
        const int16_t *layer = (ds == 0) ? prev : (ds == 1) ? curr : next;
        
        // Do 3x3 search
        #pragma unroll
        for (int dy = -1; dy <= 1; dy++) {
            #pragma unroll
            for (int dx = -1; dx <= 1; dx++) {
                if (ds == 1 && dx == 0 && dy == 0) continue;
                int nx = x+dx; int ny = y+dy;
                if (nx < 0) nx = 0; else if (nx >= w) nx = w - 1;
                if (ny < 0) ny = 0; else if (ny >= h) ny = h - 1;
                const int nb = (int)layer[ny * w + nx];
                if (looking_for_max ? (nb >= v) : (nb <= v)) return false;
            }
        }
    }
    return true;
}

static int kp_cmp_desc(const void *a, const void *b) {
    const float ra = ((const dog_keypoint_t *)a)->response;
    const float rb = ((const dog_keypoint_t *)b)->response;
    return (rb > ra) - (rb < ra);
}

static int run_nms(dog_keypoint_t *kps, int n) {
    if (n <= 0) return 0;
    qsort(kps, (size_t)n, sizeof(*kps), kp_cmp_desc);
    const float r2 = (float)(DOG_NMS_RADIUS * DOG_NMS_RADIUS);
    #pragma unroll
    for (int i = 0; i < n; i++) {
        if (kps[i].response < 0.0f) continue;
        #pragma unroll
        for (int j = i + 1; j < n; j++) {
            if (kps[j].response < 0.0f) continue;
            float dx = kps[j].x - kps[i].x, dy = kps[j].y - kps[i].y;
            if (dx * dx + dy * dy < r2) kps[j].response = -1.0f;
        }
    }
    int out = 0;
    #pragma unroll
    for (int i = 0; i < n; i++) if (kps[i].response >= 0.0f) kps[out++] = kps[i];
    return out;
}

esp_err_t dog_detect_gray(const uint8_t *gray, int width, int height, dog_result_t *result) {
    if (!gray || !result || !result->kps) return ESP_ERR_INVALID_ARG;
    
    const int max_px = width * height;
    dog_keypoint_t *raw = malloc(DOG_MAX_KEYPOINTS * sizeof(dog_keypoint_t));
    uint8_t *g_prev = malloc(max_px), *g_curr = malloc(max_px), *g_tmp = malloc(max_px), *oct_buf = malloc(max_px);
    int16_t *dog_l[DOG_SCALES_PER_OCTAVE];

    #pragma unroll
    for (int i = 0; i < DOG_SCALES_PER_OCTAVE; i++) dog_l[i] = malloc(max_px * sizeof(int16_t));

    int raw_n = 0;
    const float k = powf(2.0f, 1.0f / (float)DOG_SCALES_PER_OCTAVE);
    memcpy(oct_buf, gray, max_px);

    int ow = width, oh = height, sf = 1;

    #pragma unroll
    for (int oct = 0; oct < DOG_NUM_OCTAVES; oct++) {
        if (ow < 4 || oh < 4) break;
        gauss_blur(oct_buf, g_prev, g_tmp, ow, oh, DOG_SIGMA_INIT);

        // TODO: adjust DOG_SCALES_PER_OCTAVE viable?
        for (int s = 0; s < DOG_SCALES_PER_OCTAVE; s++) {
            gauss_blur(oct_buf, g_curr, g_tmp, ow, oh, DOG_SIGMA_INIT * powf(k, s + 1));
            compute_dog(g_curr, g_prev, dog_l[s], ow * oh);
            uint8_t *t = g_prev; g_prev = g_curr; g_curr = t;
        }

        for (int s = 1; s < DOG_SCALES_PER_OCTAVE - 1; s++) {
            for (int y = 1; y < oh - 1; y++) {
                for (int x = 1; x < ow - 1; x++) {
                    int v = dog_l[s][y * ow + x];
                    if (abs(v) < DOG_CONTRAST_MIN) continue;
                    if (is_extremum(dog_l[s-1], dog_l[s], dog_l[s+1], ow, oh, x, y)) {
                        if (raw_n < DOG_MAX_KEYPOINTS) {
                            raw[raw_n++] = (dog_keypoint_t){(float)(x*sf), (float)(y*sf), DOG_SIGMA_INIT*powf(k, s)*sf, (float)abs(v)};
                        }
                    }
                }
            }
        }
        if (oct < DOG_NUM_OCTAVES - 1) {
            gauss_blur(oct_buf, g_curr, g_tmp, ow, oh, DOG_SIGMA_INIT * powf(k, DOG_SCALES_PER_OCTAVE - 1));
            downsample2x(g_curr, ow, oh, oct_buf);
            ow /= 2; oh /= 2; sf *= 2;
        }
    }

    result->count = run_nms(raw, raw_n);
    memcpy(result->kps, raw, result->count * sizeof(dog_keypoint_t));

    free(raw); free(g_prev); free(g_curr); free(g_tmp); free(oct_buf);
    for (int i = 0; i < DOG_SCALES_PER_OCTAVE; i++) free(dog_l[i]);
    return ESP_OK;
}


// NOTE: this code is not portable, we use jpeg_decoder.h on the ESP32
esp_err_t dog_detect_jpeg(const char *filename, dog_result_t *result) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE *infile = fopen(filename, "rb");
    if (!infile) return ESP_ERR_INVALID_ARG;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, infile);
    jpeg_read_header(&cinfo, TRUE);
    jpeg_start_decompress(&cinfo);

    int w = cinfo.output_width;
    int h = cinfo.output_height;
    int channels = cinfo.output_components;
    
    uint8_t *gray = malloc(w * h);
    uint8_t *row_buffer = malloc(w * channels);

    while (cinfo.output_scanline < h) {
        int y = cinfo.output_scanline;
        jpeg_read_scanlines(&cinfo, &row_buffer, 1);
        for (int x = 0; x < w; x++) {
            if (channels == 3) { // RGB to Luma
                uint8_t r = row_buffer[x * 3], g = row_buffer[x * 3 + 1], b = row_buffer[x * 3 + 2];
                gray[y * w + x] = (uint8_t)((77u * r + 150u * g + 29u * b) >> 8u);
            } else {
                gray[y * w + x] = row_buffer[x];
            }
        }
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(infile);
    free(row_buffer);

    esp_err_t ret = dog_detect_gray(gray, w, h, result);


    if (ret == ESP_OK) {
        LOGI(TAG, "Drawing %d blobs...", result->count);
        for (int i = 0; i < result->count; i++) {
            int radius = (int)(result->kps[i].sigma * 1.414f); 
            draw_circle(gray, w, h, (int)result->kps[i].x, (int)result->kps[i].y, radius, 255);
        }
        save_ppm("output_blobs.ppm", gray, w, h);
    }

    free(gray);
    return ret;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <image.jpg>\n", argv[0]);
        return 1;
    }

    dog_result_t res;
    res.capacity = DOG_MAX_KEYPOINTS;
    res.kps = malloc(res.capacity * sizeof(dog_keypoint_t));

    if (dog_detect_jpeg(argv[1], &res) == ESP_OK) {
        LOGI(TAG, "Detected %d blobs", res.count);
        for (int i = 0; i < res.count; i++) {
            printf("(x,y) of Blob %d: %.1f,%.1f\n", i, res.kps[i].x, res.kps[i].y);
                   // also have res.kps[i].sigma, res.kps[i].response to play with
        }
    }

    free(res.kps);
    return 0;
}
```

I extended our group's HTTP client application to support interacting with this program (+an image viewer), to allow sending these coordinates as a route-file to the ESP32.

This is going to be used in our demo functionality, and exists as a general fallback approach if on-board computer vision appears impossible.

More notes will follow, today or some later day.


## March 5

Finalized PCBWay order with Max. This was the second-revision board, incorporating the
connector pitch fix (March 23) and updated antenna clearance. We also locked in the custom
partition layout below, which carves out 960 KB for a FAT filesystem on the 4 MB flash —
enough to cache a session's worth of JPEG frames without hitting the app binary.


```
  nvs       0x9000    24 KB
phy_init  0xf000     4 KB
factory   0x10000    1 MB   ← app binary
storage   0x110000 960 KB   ← FAT filesystem
```


## March 3

Got the camera working FINALLY; there was a BSS address mismatch

At this time, we concluded the following
```
   verified on oscope that the pwm signal looks good

- but need to plug in a servo, calibrate tuning
- -> do the time to angle conversion, we should do some actual regression tests with the gimbals to make sur eits approximately good

- get the camera wokring



== for the breadboard demo

we have working http demo ( )

we have working (ish) servo control ( )

we NEED to have the camera up and running (high priority)

we NEED to port the CV algorithm to C (ingi will do this)

we NEED to connect the http server to control commands ( ) 

we NEED to connect the http to take pictures ( ) 

we NEED to fix the fatfs implementation (prakhar will do this, asynch or on thursday)
```

## March 2

More PCB decision making.

After the breadboard work validated the core servo + camera + HTTP flow, the question was
whether to spin a new revision of the PCB or continue on the breadboard through the demo.
We decided to go ahead with a second PCB revision for two reasons: (1) the breadboard's
wiring harness for the four gimbals was already fragile enough to cause intermittent servo
glitches, and (2) getting a clean board now leaves buffer time before the demo if PCBWay
delivery slips. The design changes earmarked for Rev 2 at this point were the connector pitch
correction and tighter power-plane routing for the camera supply.


## March 1

We performed basic current calculations, etc. to design the PCB.


## Feburary 28

We worked to finalize or specification for the purpose of the Design Review, withholding further work
until after receiving this feedback.


## Feburary 26

Define subsystems better. Search for parts. Most of the parts decisions happened in person
and synchronously, the part selections are reflected in our revision 1 BOM:

```tsv
Item	Description	Quantity	Unit Price	Total Price	Vendor	Part Number / SKU	Link
ESP32-S3 DevKit	ESP32-S3-DEVKITC-1-N8R8 (8MB Flash + 8MB PSRAM)	2	$15.00	$30.00	DigiKey	ESP32-S3-DEVKITC-1-N8R8	https://www.digikey.com/en/products/detail/espressif-systems/ESP32-S3-DEVKITC-1-N8R8/15295894
Class 2 Laser Module	5V 650nm Red Laser Diode Module <1mW (pack of 5)	1	$6.79	$6.70	Amazon	-	https://www.amazon.com/HiLetgo-10pcs-650nm-Diode-Laser/dp/B071FT9HSV?dib=eyJ2IjoiMSJ9.QBv3aqMwJBtdArCtopYwM5pHsWnRIosV6SelUPVBXNtetOHmpsv7fQdMv51PljhnEJC5rZvA4VYL3DiOa2cu6tkz_dtGTKkIEHotFIxYLzUT_4l2eq6x2ddY9AK35HWktivqF0nN8NQFS_tWQQve0JDyEACD3NRwTNVV2r6fv45EgpD309n5IaCbsJS_jRaNajLAMChSPg-XIVNXqO0WVZ_N7ifqvf4jrqXw10VJffw.Njm2vD6orYxQ5s-NvSGyK2ae5IpWHDCt7SOyVl1AgQc&dib_tag=se&keywords=5v+650nm+laser+module+less+than+1mw&qid=1771291458&sr=8-3
Adafruit Mini Pan-Tilt Kit	Micro servos with 180/150 degree range, good for testing and works with standard Servo.h library. PWM controlled	2	$18.95	$37.90	DigiKey	1528-1106-ND	https://www.digikey.com/en/products/detail/adafruit-industries-llc/1967/5154681
Camera Module	ADAFRUIT OV5640 CAMERA BREAKOUT. Has easy to use pinout for evaluation/setting up software stack	1	$14.50	$14.50	Digikey	1528-5838-ND	https://www.digikey.com/en/products/detail/adafruit-industries-llc/5838/22163173
3.3V LDO Regulator	LM3940IT	2	?	?	ECE Shop	LM3940IT	N/A
```
