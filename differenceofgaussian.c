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
            printf("%d,%d;", (int)(res.kps[i].x), (int)(res.kps[i].y));
                   // also have res.kps[i].sigma, res.kps[i].response to play with
        }
    }

    free(res.kps);
    return 0;
}