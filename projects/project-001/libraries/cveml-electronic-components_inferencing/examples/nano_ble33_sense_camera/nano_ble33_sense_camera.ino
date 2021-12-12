/* Edge Impulse Arduino examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <cveml-electronic-components-dnn_inferencing.h>
#include <Arduino_OV767X.h>

#include <stdint.h>
#include <stdlib.h>

/* Constant variables ------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS     176
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS     144

// intermediate width and height (i.e., crop width and height)
#define EI_CAMERA_CROP_WIDTH  96
#define EI_CAMERA_CROP_HEIGHT 96

#define DWORD_ALIGN_PTR(a)   ((a & 0x3) ?(((uintptr_t)a + 0x4) & ~(uintptr_t)0x3) : a)

#ifdef __VALIDATION
// example of capacitor image, for validation
const uint8_t img_sample[EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 1] = { 
  190, 190, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 192,
  192, 191, 191, 191, 191, 190, 190, 190, 191, 190, 189, 189, 189,
  189, 188, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 192,
  191, 191, 192, 192, 192, 192, 192, 191, 191, 191, 190, 190, 190,
  190, 189, 189, 188, 192, 192, 191, 189, 190, 190, 191, 192, 191,
  191, 191, 191, 192, 192, 192, 192, 192, 192, 192, 191, 191, 190,
  190, 189, 189, 187, 186, 186, 191, 189, 189, 188, 190, 191, 191,
  192, 192, 191, 191, 192, 192, 192, 192, 193, 193, 193, 192, 191,
  190, 190, 189, 188, 188, 185, 184, 184, 189, 182, 192, 190, 189,
  190, 191, 192, 192, 192, 191, 192, 192, 192, 193, 194, 194, 193,
  193, 191, 190, 190, 189, 187, 187, 184, 177, 181, 190, 186, 217,
  193, 187, 191, 191, 192, 192, 193, 192, 192, 192, 192, 192, 194,
  194, 192, 192, 191, 191, 190, 189, 187, 186, 174, 161, 181, 189,
  182, 179, 182, 183, 190, 191, 192, 192, 192, 192, 192, 193, 192,
  193, 193, 194, 192, 192, 192, 191, 190, 189, 188, 186, 179, 170,
  179, 189, 182, 186, 185, 188, 191, 191, 193, 192, 189, 171, 154,
  153, 170, 189, 192, 194, 192, 192, 191, 190, 189, 188, 188, 187,
  185, 184, 183, 191, 191, 190, 190, 190, 191, 192, 193, 186, 133,
  96,  90,  87,  90, 130, 184, 194, 192, 192, 191, 190, 189, 188,
  188, 187, 186, 185, 184, 191, 191, 192, 192, 192, 192, 192, 190,
  149,  92, 100,  99,  86,  81,  82, 132, 187, 192, 192, 191, 190,
  189, 188, 187, 186, 185, 185, 184, 191, 192, 192, 192, 192, 192,
  193, 191, 118,  91, 153, 116,  85,  80,  80,  87, 164, 191, 192,
  191, 190, 189, 189, 188, 187, 186, 186, 185, 192, 192, 192, 192,
  192, 192, 193, 184, 105,  87, 100,  89,  84,  84,  81,  81, 127,
  187, 193, 192, 191, 190, 189, 189, 187, 186, 186, 185, 192, 192,
  191, 191, 191, 194, 195, 176,  97,  81,  82,  80,  85,  99,  91,
  82, 101, 177, 191, 191, 191, 190, 189, 188, 188, 188, 186, 186,
  192, 192, 192, 193, 193, 195, 197, 161,  90,  79,  79,  80,  81,
  89,  98,  88,  88, 160, 190, 191, 191, 190, 189, 189, 188, 187,
  187, 186, 192, 192, 193, 193, 194, 196, 197, 139,  85,  75,  76,
  76,  77,  78,  80,  81,  82, 134, 187, 191, 191, 190, 190, 189,
  189, 189, 188, 188, 192, 193, 193, 194, 194, 195, 195, 126,  80,
  74,  72,  72,  72,  73,  74,  78,  82, 109, 180, 191, 190, 190,
  190, 189, 189, 189, 188, 186, 193, 193, 193, 193, 194, 193, 192,
  120,  79,  83,  86,  82,  77,  74,  75,  79,  85,  97, 175, 191,
  191, 191, 190, 189, 189, 189, 189, 188, 192, 193, 193, 192, 193,
  192, 191, 127, 117, 178, 180, 172, 164, 164, 166, 148,  97,  95,
  176, 191, 191, 191, 190, 189, 189, 189, 189, 189, 191, 192, 193,
  192, 192, 192, 192, 166, 152, 193, 195, 195, 194, 193, 194, 188,
  117, 106, 178, 191, 192, 191, 190, 189, 189, 189, 188, 188, 190,
  191, 191, 191, 192, 193, 194, 189, 152, 187, 193, 194, 193, 194,
  193, 190, 133, 129, 184, 191, 191, 191, 190, 189, 188, 188, 188,
  188, 190, 190, 191, 192, 192, 193, 194, 192, 157, 165, 191, 192,
  192, 193, 193, 191, 148, 142, 187, 191, 191, 191, 190, 190, 189,
  189, 188, 188, 190, 190, 191, 191, 193, 192, 193, 193, 180, 135,
  187, 191, 192, 192, 192, 193, 162, 145, 188, 191, 191, 191, 190,
  190, 190, 190, 189, 188, 189, 191, 191, 191, 192, 192, 193, 193,
  193, 136, 176, 190, 192, 192, 193, 193, 174, 146, 187, 191, 191,
  190, 190, 190, 190, 189, 189, 189, 190, 190, 190, 191, 191, 191,
  191, 191, 192, 156, 157, 189, 191, 192, 192, 192, 178, 137, 185,
  191, 190, 190, 190, 190, 189, 190, 188, 188, 190, 185, 186, 189,
  190, 191, 191, 191, 191, 179, 139, 189, 192, 192, 191, 191, 184,
  130, 181, 190, 190, 189, 190, 188, 185, 185, 188, 188, 189, 183,
  186, 187, 190, 191, 191, 191, 191, 189, 137, 182, 190, 191, 191,
  191, 188, 139, 175, 189, 190, 189, 185, 167, 158, 176, 187, 189,
  189, 186, 186, 186, 189, 190, 190, 190, 191, 190, 173, 185, 190,
  190, 190, 190, 188, 146, 170, 188, 190, 189, 189, 177, 165, 178,
  185, 188, 190, 189, 187, 190, 190, 190, 190, 190, 190, 190, 190,
  190, 190, 190, 190, 190, 189, 172, 174, 188, 189, 189, 189, 176,
  176, 162, 176, 187
};
#endif

/* Edge Impulse ------------------------------------------------------------- */
class OV7670 : public OV767X {
    public:
        int begin(int resolution, int format, int fps);
        void readFrame(void* buffer, const int crop_width = 0, const int crop_height = 0);

    private:
        // pin numbers for VSYNC, HREF, PCLK, XCLK (not used in the code ?)
        // in my OV7670 model, names are a bit different :
        //  - lib name  : my model's name
        //  - VSYNC     : VS
        //  - HREF      : HS
        //  - PCLK      : PLK
        //  - XCLK      : XLK 
        int m_vsyncPin;
        int m_hrefPin;
        int m_pclkPin;
        int m_xclkPin;

        // ports and masks for 'sync pins', i.e. pins that delimit frames (VSYNC) and frame rows (HREF)
        volatile uint32_t*  m_vsyncPort;
        uint32_t            m_vsyncMask;
        volatile uint32_t*  m_hrefPort;
        uint32_t            m_hrefMask;
        volatile uint32_t*  m_pclkPort;
        uint32_t            m_pclkMask;

        uint16_t    m_width;
        uint16_t    m_height;
        uint8_t     m_bytes_per_pixel;
        uint16_t    m_bytes_per_row;
        uint8_t     m_buf_rows;
        uint16_t    m_buf_size;
        uint8_t     m_resize_height;
        uint8_t*    m_raw_buf;
        void*       m_buf_mem;
        uint8_t*    m_intrp_buf;
        uint8_t*    m_buf_limit;

        void readBuf(const int buf_rows);
        int allocate_scratch_buffs();
        int deallocate_scratch_buffs();
};

typedef struct {
    size_t width;
    size_t height;
} ei_device_resize_resolutions_t;

/**
 * @brief      Check if new serial data is available
 *
 * @return     Returns number of available bytes
 */
int ei_get_serial_available(void) {
    return Serial.available();
}

/**
 * @brief      Get next available byte
 *
 * @return     byte
 */
char ei_get_serial_byte(void) {
    return Serial.read();
}

/**
 * @brief      Printf function uses vsnprintf and output using Arduino Serial
 *
 * @param[in]  format     Variable argument list
 */
void ei_printf(const char *format, ...) {
    char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}

/* Private variables ------------------------------------------------------- */
static OV7670 Cam;
static bool is_initialised = false;

/*
** @brief points to the output of the capture
*/
static uint8_t *ei_camera_capture_out = NULL;
uint32_t resize_col_sz;
uint32_t resize_row_sz;
bool do_resize = false;
bool do_crop = false;

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *buf);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
void resizeImage(int srcWidth, int srcHeight, uint8_t *srcImage, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp);
void cropImage(int srcWidth, int srcHeight, uint8_t *srcImage, int startX, int startY, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp);

/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tImage resolution: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    bool stop_inferencing = false;

    if (ei_camera_init() == false) {
        ei_printf("ERR: Failed to initialize image sensor\r\n");
        stop_inferencing = true;
    }

    while(stop_inferencing == false) {
        ei_printf("\nStarting inferencing in 2 seconds...\n");

        // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
        if (ei_sleep(2000) != EI_IMPULSE_OK) {
            break;
        }

        ei_printf("Taking photo...\n");

        void *snapshot_mem = NULL;
        uint8_t *snapshot_buf = NULL;
        snapshot_mem = ei_malloc(EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 1);
        if(snapshot_mem == NULL) {
            ei_printf("failed to create snapshot_mem\r\n");
            break;
        }
        snapshot_buf = (uint8_t *)DWORD_ALIGN_PTR((uintptr_t)snapshot_mem);

        if (ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
            ei_printf("Failed to capture image\r\n");
            if (snapshot_mem) ei_free(snapshot_mem);
            break;
        }

        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &ei_camera_cutout_get_data;

        // run the impulse: DSP, neural network and the Anomaly algorithm
        ei_impulse_result_t result = { 0 };

        EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &result, debug_nn);
        if (ei_error != EI_IMPULSE_OK) {
            ei_printf("Failed to run impulse (%d)\n", ei_error);
            ei_free(snapshot_mem);
            break;
        }

        // print the predictions
        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                    result.timing.dsp, result.timing.classification, result.timing.anomaly);
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("    %s: \t%f\r\n", result.classification[ix].label, result.classification[ix].value);
        }

//        Serial.print("\nimg : ");
//        for (int i = 0; i < EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT; ++i)
//        {
//          char pixel[4];
//          sprintf(pixel, "%d ", ei_camera_capture_out[i]);
//          Serial.print(pixel);
//        }
//        Serial.println("");
        
#if EI_CLASSIFIER_HAS_ANOMALY == 1
            ei_printf("    anomaly score: %f\r\n", result.anomaly);
#endif

        while (ei_get_serial_available() > 0) {
            if (ei_get_serial_byte() == 'b') {
                ei_printf("Inferencing stopped by user\r\n");
                stop_inferencing = true;
            }
        }
        if (snapshot_mem) ei_free(snapshot_mem);
    }
    
    ei_camera_deinit();
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {
    if (is_initialised) return true;
    
    if (!Cam.begin(QCIF, GRAYSCALE, 1)) {
        ei_printf("ERR: Failed to initialize camera\r\n");
        return false;
    }
    is_initialised = true;

    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {
    if (is_initialised) {
        Cam.end();
        is_initialised = false;
    }
}

/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           when full resolution is expected.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) 
{
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    if (!out_buf) {
        ei_printf("ERR: invalid parameters\r\n");
        return false;
    }

    Cam.readFrame(out_buf, EI_CAMERA_CROP_WIDTH, EI_CAMERA_CROP_HEIGHT); // captures image, crops and resizes

    // The following variables should always be assigned
    // if this routine is to return true
    // cutout values
    //ei_camera_snapshot_is_resized = do_resize;
    //ei_camera_snapshot_is_cropped = do_crop;
    ei_camera_capture_out = out_buf;

    return true;
}

/**
 * @brief      Convert RGB565 raw camera buffer to RGB888
 *
 * @param[in]   offset       pixel offset of raw buffer
 * @param[in]   length       number of pixels to convert
 * @param[out]  out_buf      pointer to store output image
 */
int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset;
    size_t bytes_left = length;
    size_t out_ptr_ix = 0;

    // read byte for byte
    while (bytes_left != 0) {
#ifdef __VALIDATION
        uint8_t pixel = img_sample[pixel_ix];
#else
        uint8_t pixel = ei_camera_capture_out[pixel_ix];
#endif
        // convert to NN input format (in range [0.0, 1.0])
        float pixel_f = (float) pixel / 255.f;
        out_ptr[out_ptr_ix] = pixel_f;

        // and go to the next pixel
        out_ptr_ix++;
        pixel_ix++;
        bytes_left--;
    }

    // and done!
    return 0;
}

// This include file works in the Arduino environment
// to define the Cortex-M intrinsics
#ifdef __ARM_FEATURE_SIMD32
#include <device.h>
#endif
// This needs to be < 16 or it won't fit. Cortex-M4 only has SIMD for signed multiplies
#define FRAC_BITS 14
#define FRAC_VAL (1<<FRAC_BITS)
#define FRAC_MASK (FRAC_VAL - 1)
//
// Resize
//
// Assumes that the destination buffer is dword-aligned
// Can be used to resize the image smaller or larger
// If resizing much smaller than 1/3 size, then a more rubust algorithm should average all of the pixels
// This algorithm uses bilinear interpolation - averages a 2x2 region to generate each new pixel
//
// Optimized for 32-bit MCUs
// supports 8 and 16-bit pixels
void resizeImage(int srcWidth, int srcHeight, uint8_t *srcImage, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp)
{
    uint32_t src_x_accum, src_y_accum; // accumulators and fractions for scaling the image
    uint32_t x_frac, nx_frac, y_frac, ny_frac;
    int x, y, ty, tx;

    if (iBpp != 8 && iBpp != 16)
        return;
    src_y_accum = FRAC_VAL/2; // start at 1/2 pixel in to account for integer downsampling which might miss pixels
    const uint32_t src_x_frac = (srcWidth * FRAC_VAL) / dstWidth;
    const uint32_t src_y_frac = (srcHeight * FRAC_VAL) / dstHeight;
    const uint32_t r_mask = 0xf800f800;
    const uint32_t g_mask = 0x07e007e0;
    const uint32_t b_mask = 0x001f001f;
    uint8_t *s, *d;
    uint16_t *s16, *d16;
    uint32_t x_frac2, y_frac2; // for 16-bit SIMD
    for (y=0; y < dstHeight; y++) {
        ty = src_y_accum >> FRAC_BITS; // src y
        y_frac = src_y_accum & FRAC_MASK;
        src_y_accum += src_y_frac;
        ny_frac = FRAC_VAL - y_frac; // y fraction and 1.0 - y fraction
        y_frac2 = ny_frac | (y_frac << 16); // for M4/M4 SIMD
        s = &srcImage[ty * srcWidth];
        s16 = (uint16_t *)&srcImage[ty * srcWidth * 2];
        d = &dstImage[y * dstWidth];
        d16 = (uint16_t *)&dstImage[y * dstWidth * 2];
        src_x_accum = FRAC_VAL/2; // start at 1/2 pixel in to account for integer downsampling which might miss pixels
        if (iBpp == 8) {
        for (x=0; x < dstWidth; x++) {
            uint32_t tx, p00,p01,p10,p11;
            tx = src_x_accum >> FRAC_BITS;
            x_frac = src_x_accum & FRAC_MASK;
            nx_frac = FRAC_VAL - x_frac; // x fraction and 1.0 - x fraction
            x_frac2 = nx_frac | (x_frac << 16);
            src_x_accum += src_x_frac;
            p00 = s[tx]; p10 = s[tx+1];
            p01 = s[tx+srcWidth]; p11 = s[tx+srcWidth+1];
    #ifdef __ARM_FEATURE_SIMD32
            p00 = __SMLAD(p00 | (p10<<16), x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
            p01 = __SMLAD(p01 | (p11<<16), x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
            p00 = __SMLAD(p00 | (p01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
    #else // generic C code
            p00 = ((p00 * nx_frac) + (p10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
            p01 = ((p01 * nx_frac) + (p11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
            p00 = ((p00 * ny_frac) + (p01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
    #endif // Cortex-M4/M7
            *d++ = (uint8_t)p00; // store new pixel
        } // for x
        } // 8-bpp
        else
        { // RGB565
        for (x=0; x < dstWidth; x++) {
            uint32_t tx, p00,p01,p10,p11;
            uint32_t r00, r01, r10, r11, g00, g01, g10, g11, b00, b01, b10, b11;
            tx = src_x_accum >> FRAC_BITS;
            x_frac = src_x_accum & FRAC_MASK;
            nx_frac = FRAC_VAL - x_frac; // x fraction and 1.0 - x fraction
            x_frac2 = nx_frac | (x_frac << 16);
            src_x_accum += src_x_frac;
            p00 = __builtin_bswap16(s16[tx]); p10 = __builtin_bswap16(s16[tx+1]);
            p01 = __builtin_bswap16(s16[tx+srcWidth]); p11 = __builtin_bswap16(s16[tx+srcWidth+1]);
    #ifdef __ARM_FEATURE_SIMD32
            {
            p00 |= (p10 << 16);
            p01 |= (p11 << 16);
            r00 = (p00 & r_mask) >> 1; g00 = p00 & g_mask; b00 = p00 & b_mask;
            r01 = (p01 & r_mask) >> 1; g01 = p01 & g_mask; b01 = p01 & b_mask;
            r00 = __SMLAD(r00, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
            r01 = __SMLAD(r01, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
            r00 = __SMLAD(r00 | (r01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
            g00 = __SMLAD(g00, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
            g01 = __SMLAD(g01, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
            g00 = __SMLAD(g00 | (g01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
            b00 = __SMLAD(b00, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
            b01 = __SMLAD(b01, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
            b00 = __SMLAD(b00 | (b01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
            }
    #else // generic C code
            {
            r00 = (p00 & r_mask) >> 1; g00 = p00 & g_mask; b00 = p00 & b_mask;
            r10 = (p10 & r_mask) >> 1; g10 = p10 & g_mask; b10 = p10 & b_mask;
            r01 = (p01 & r_mask) >> 1; g01 = p01 & g_mask; b01 = p01 & b_mask;
            r11 = (p11 & r_mask) >> 1; g11 = p11 & g_mask; b11 = p11 & b_mask;
            r00 = ((r00 * nx_frac) + (r10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
            r01 = ((r01 * nx_frac) + (r11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
            r00 = ((r00 * ny_frac) + (r01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
            g00 = ((g00 * nx_frac) + (g10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
            g01 = ((g01 * nx_frac) + (g11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
            g00 = ((g00 * ny_frac) + (g01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
            b00 = ((b00 * nx_frac) + (b10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
            b01 = ((b01 * nx_frac) + (b11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
            b00 = ((b00 * ny_frac) + (b01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
            }
    #endif // Cortex-M4/M7
            r00 = (r00 << 1) & r_mask;
            g00 = g00 & g_mask;
            b00 = b00 & b_mask;
            p00 = (r00 | g00 | b00); // re-combine color components
            *d16++ = (uint16_t)__builtin_bswap16(p00); // store new pixel
        } // for x
        } // 16-bpp
    } // for y
} /* resizeImage() */
//
// Crop
//
// Assumes that the destination buffer is dword-aligned
// optimized for 32-bit MCUs
// Supports 8 and 16-bit pixels
//
void cropImage(int srcWidth, int srcHeight, uint8_t *srcImage, int startX, int startY, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp)
{
    uint32_t *s32, *d32;
    int x, y;

    if (startX < 0 || startX >= srcWidth || startY < 0 || startY >= srcHeight || (startX + dstWidth) > srcWidth || (startY + dstHeight) > srcHeight)
       return; // invalid parameters
    if (iBpp != 8 && iBpp != 16)
       return;

    if (iBpp == 8) {
      uint8_t *s, *d;
      for (y=0; y<dstHeight; y++) {
        s = &srcImage[srcWidth * (y + startY) + startX];
        d = &dstImage[(dstWidth * y)];
        x = 0;
        if ((intptr_t)s & 3 || (intptr_t)d & 3) { // either src or dst pointer is not aligned
          for (; x<dstWidth; x++) {
            *d++ = *s++; // have to do it byte-by-byte
          }
        } else {
          // move 4 bytes at a time if aligned or alignment not enforced
          s32 = (uint32_t *)s;
          d32 = (uint32_t *)d;
          for (; x<dstWidth-3; x+= 4) {
            *d32++ = *s32++;
          }
          // any remaining stragglers?
          s = (uint8_t *)s32;
          d = (uint8_t *)d32;
          for (; x<dstWidth; x++) {
            *d++ = *s++;
          }
        }
      } // for y
    } // 8-bpp
    else
    {
      uint16_t *s, *d;
      for (y=0; y<dstHeight; y++) {
        s = (uint16_t *)&srcImage[2 * srcWidth * (y + startY) + startX * 2];
        d = (uint16_t *)&dstImage[(dstWidth * y * 2)];
        x = 0;
        if ((intptr_t)s & 2 || (intptr_t)d & 2) { // either src or dst pointer is not aligned
          for (; x<dstWidth; x++) {
            *d++ = *s++; // have to do it 16-bits at a time
          }
        } else {
          // move 4 bytes at a time if aligned or alignment no enforced
          s32 = (uint32_t *)s;
          d32 = (uint32_t *)d;
          for (; x<dstWidth-1; x+= 2) { // we can move 2 pixels at a time
            *d32++ = *s32++;
          }
          // any remaining stragglers?
          s = (uint16_t *)s32;
          d = (uint16_t *)d32;
          for (; x<dstWidth; x++) {
            *d++ = *s++;
          }
        }
      } // for y
    } // 16-bpp case
} /* cropImage() */

//#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
//#error "Invalid model for current sensor"
//#endif

// OV767X camera library override
#include <Arduino.h>
#include <Wire.h>

#define digitalPinToBitMask(P) (1 << (digitalPinToPinName(P) % 32))
#define portInputRegister(P) ((P == 0) ? &NRF_P0->IN : &NRF_P1->IN)

//
// OV7670::begin()
//
// Extends the OV767X library function. Some private variables are needed
// to use the OV7670::readFrame function.
//
int OV7670::begin(int resolution, int format, int fps)
{
    pinMode(OV7670_VSYNC,   INPUT);     // pulses for each new frame : rows start after a VSYNC pulse
    pinMode(OV7670_HREF,    INPUT);     // HREF pulses that stay high for a whole 'row'
    pinMode(OV7670_PLK,     INPUT);     // pixel clock : each single pixel value is transmitted during a period of this clock signal
    pinMode(OV7670_XCLK,    OUTPUT);    // the arduino nano provides the system clck signal to the OV7670 camera

    // in the arduino nano, we map pins to ports like this : [pin number] : {port number, bit number}.
    // the functions below allow do obtain the values of the mapping, given a pin number as the key :
    //  - digitalPinToPort(<pin number>)    : returns the port number element of the mapping
    //  - digitalPinToBitMask(<pin number>) : returns a bitmask, with the respective bit of the mapping set to 1
    //
    // e.g., for OV7670_VSYNC (pin number 8) is mapped to {0, 21} : https://docs.arduino.cc/hardware/nano-33-ble-sense
    //
    // finally, the portInputRegister(<port number>) returns a ref. of an unsigned (?) that contains the values of the bits currently being 'read' in the port.
    // for the nRF52840 micro in the nano 33 ble sense, that would be either port 0 or port 1, as there are only 2 ports.
    m_vsyncPort   = portInputRegister(digitalPinToPort(OV7670_VSYNC));
    m_vsyncMask   = digitalPinToBitMask(OV7670_VSYNC);
    m_hrefPort    = portInputRegister(digitalPinToPort(OV7670_HREF));
    m_hrefMask    = digitalPinToBitMask(OV7670_HREF);
    m_pclkPort    = portInputRegister(digitalPinToPort(OV7670_PLK));
    m_pclkMask    = digitalPinToBitMask(OV7670_PLK);

    bool ret          = OV767X::begin(resolution, format, fps);   // init OV7670 driver to resolution passed as argument
    m_width           = OV767X::width();                          // sensor width
    m_height          = OV767X::height();                         // sensor height
    m_bytes_per_pixel = OV767X::bytesPerPixel();                  // nr. of bytes per pixel
    m_bytes_per_row   = m_width * m_bytes_per_pixel;              // each pixel is 2 bytes for RGB, 2 byte for grayscale (even though only the 'Y' byte of the YUV422 format is considered in the end)

    m_resize_height   = 7; // ???

    m_buf_mem = NULL;
    m_raw_buf = NULL;
    m_intrp_buf = NULL;

    return ret;

} /* OV7670::begin() */

int OV7670::allocate_scratch_buffs()
{
    //ei_printf("allocating buffers..\r\n");
    m_buf_rows = (EI_CAMERA_CROP_WIDTH * m_resize_height) / EI_CLASSIFIER_INPUT_HEIGHT;
    // if the grayscale color format is chosen, we only consider the 'Y' byte of the YUV422 format
    // as such, although the OV7670 camera transmits 3 byte per pixel, the final buffer will contain 1 byte per pixel
    uint16_t bytes_per_row = (OV767X::isGrayscale() ? (m_bytes_per_row / 2) : m_bytes_per_row);
    m_buf_size = bytes_per_row * m_buf_rows;

    m_buf_mem = ei_malloc(m_buf_size);
    if(m_buf_mem == NULL) {
        ei_printf("failed to create buf_mem\r\n");
        return false;
    }
    m_raw_buf = (uint8_t *) DWORD_ALIGN_PTR((uintptr_t) m_buf_mem);

    //ei_printf("allocating buffers OK\r\n");
    return 0;
}

int OV7670::deallocate_scratch_buffs()
{
    //ei_printf("deallocating buffers...\r\n");
    ei_free(m_buf_mem);
    m_buf_mem = NULL;
    
    //ei_printf("deallocating buffers OK\r\n");
    return 0;
}

//
// OV7670::readFrame()
//
// Overrides the OV767X library function. Fixes the camera output to be
// a far more desirable image. This image utilizes the full sensor size
// and has the correct aspect ratio. Since there is limited memory on the
// Nano we bring in only part of the entire sensor at a time and then
// interpolate to a lower resolution.
//
void OV7670::readFrame(void* buffer, const int crop_width, const int crop_height)
{
    allocate_scratch_buffs();

    uint8_t* out = (uint8_t*) buffer;
    noInterrupts();

    // look for the rising and falling edge transitions of a VSYNC pulse, indicating a start of a frame, 
    // as shown in fig. 6 of page 7 of the OV7670 datasheet in http://www.voti.nl/docs/OV7670.pdf
    while ((*m_vsyncPort & m_vsyncMask) == 0);
    while ((*m_vsyncPort & m_vsyncMask) != 0);

    int out_row = 0;
    int raw_height_init = ( crop_height > 0 ? (m_height - crop_height) / 2 : 0 );
    int raw_height_end  = ( crop_height > 0 ? raw_height_init + crop_height : m_height );

    int bytes_per_pixel = (OV767X::isGrayscale() ? m_bytes_per_pixel / 2 : m_bytes_per_pixel);
    int bits_per_pixel = bytes_per_pixel * 8;

    // the loop does the following operations :
    //  1) capture n rows from the image, using readBuf() (n = m_buf_rows) : the way we calculate m_buf_rows is described in allocate_scratch_buffs()
    //  2) if the captured rows are within the 'cropping area', crop the middle section of the n rows (forming a [crop_width x m_buf_rows] array), 
    //      resizeImage() on the cropped area to [EI_CLASSIFIER_INPUT_WIDTH x m_buf_rows], and save it to the buffer passed as argument

    for (int raw_height = 0; raw_height < m_height; ) {

        // read in m_width x buf_rows buffer to work with
        readBuf(m_buf_rows);

        // if current row is within the desired 'cropping area', crop, resize and add to output array
        if ((raw_height >= raw_height_init) & (raw_height < raw_height_end))
        {
            // crop image to crop_width px x m_buf_rows
            if (crop_width > 0)
            {
                cropImage(m_width, m_buf_rows, 
                          m_raw_buf, 
                          (m_width - crop_width) / 2, 0, 
                          crop_width, m_buf_rows, 
                          m_raw_buf, 
                          bits_per_pixel);
            }

            // resize fraction of image to DST_WIDTH x m_buf_rows
            resizeImage(crop_width, m_buf_rows,
                        m_raw_buf,
                        EI_CLASSIFIER_INPUT_WIDTH, m_resize_height,
                        &(out[out_row]),
                        bits_per_pixel);
            
            out_row += EI_CLASSIFIER_INPUT_WIDTH * m_resize_height * bytes_per_pixel;
            
        }

        // increment raw_height by the pre-determined buf_rows
        raw_height += m_buf_rows;
    }

    interrupts();

    deallocate_scratch_buffs();
} /* OV7670::readFrame() */

//
// OV7670::readBuf()
//
// Extends the OV767X library function. 
// Reads buf_rows VGA rows from the image sensor.
//
void OV7670::readBuf(const int buf_rows)
{
    int offset = 0;

    // all the data bits coming from the OV7670 camera (i.e., D0 to D7) are in port 1 of the ardunino nano 33 ble sense
    // P1.xx set of GPIO is in 'pin' 32 and above, so all we have to do is to get a ref. to port 1.
    // that can be done by passing a 33 to nrf_gpio_pin_port_decode()
    uint32_t ulPin = 33;
    NRF_GPIO_Type* port;
    port = nrf_gpio_pin_port_decode(&ulPin);

    for (int i = 0; i < buf_rows; i++) 
    {
        // look for a rising edge of the HREF signal, as shown in fig. 6 of page 7 of the OV7670 camera datasheet : http://www.voti.nl/docs/OV7670.pdf
        // this indicates data for a new row, and stays high for as long as row bytes are transmitted
        while ((*m_hrefPort & m_hrefMask) == 0);

        for (int col = 0; col < m_bytes_per_row; col++) 
        {
            // look for falling edges of the PCLK signal : this indicates the start of a new byte of data
            while ((*m_pclkPort & m_pclkMask) != 0);

            uint32_t in = port->IN; // read all bits in parallel

            // re-arrange the port 1 bits
            // the two bottom rows in the table below show the mapping between the port 1 bits and Dx pins in the OV7670 camera :
            // nano port 1 pins : | 04 | 06 | 05 | 03 | 02 | 01 |    |    |    |    |    |    | 00 | 10 |    |    |
            //                    ---------------------------------------------------------------------------------
            // nano port 1 bits : | 15 | 14 | 13 | 12 | 11 | 10 | 09 | 08 | 07 | 06 | 05 | 04 | 03 | 02 | 01 | 00 |
            //        OV7670 Dx : | D7 | D6 | D5 | D4 | D3 | D2 |    |    |    |    |    |    | D1 | D0 |    |    |

            in >>= 2;           // right-shift port 1 bits by 2, so that to make D1 and D0 the lsbs
            in &= 0x3f03;       // isolate the 8 bits we care about, i.e. bitmask : 0011 1111 0000 0011
            in |= (in >> 6);    // combine the upper 6 and lower 2 bits

            // if the color format is 'grayscale', i.e. YUV422, only consider even bytes (Y values, aka the 'luminance component') and discard the odd bytes (U or V values)
            // the YUV422 format uses 3 bytes per pixel (Y, U and V values), however U and V calues are shared between two consecutive pixels :
            //    byte nr : | 00 | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 10 | 11 | 12 | ... |
            //  byte type : | U0 | Y0 | V0 | Y1 | U2 | Y2 | V2 | Y3 | U4 | Y4 | V4 | Y5 | U6 | ... |
            //
            // the mapping with pixels is as follows :
            //  pixel 0 : [Y0, U0, V0]
            //  pixel 1 : [Y1, U0, V0] 
            //  pixel 2 : [Y2, U2, V2]
            //  pixel 3 : [Y3, U2, V2] 
            //  pixel 3 : [Y4, U4, V4] 
            //   ... 
            if (!(col & 0x01) || !OV767X::isGrayscale())
            {
                m_raw_buf[offset++] = in;
            }

            // as shown in fig. 6 of page 7 in the OV7670 datasheet, we wait for a rising edge of the PCLK signal to continue with the next byte
            while ((*m_pclkPort & m_pclkMask) == 0);
        }

        // continue with the next row after the HREF signal pulse goes to low 
        // according to fig. 6 of page 7 of OV7670 datasheet, it should stay low for 144 x t_pixel
        while ((*m_hrefPort & m_hrefMask) != 0);
    }
} /* OV7670::readBuf() */
