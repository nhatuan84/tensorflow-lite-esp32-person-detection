#include "image_provider.h"
#include <JPEGDecoder.h>
#include "esp_camera.h"

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Get the camera module ready
TfLiteStatus InitCamera(tflite::ErrorReporter* error_reporter) {
  TF_LITE_REPORT_ERROR(error_reporter, "Attempting to start esp cam");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 15;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    TF_LITE_REPORT_ERROR(error_reporter, "Camera init failed");
    return kTfLiteError;
  }
  return kTfLiteOk;
}

// Decode the JPEG image, crop it, and convert it to greyscale
TfLiteStatus DecodeAndProcessImage(tflite::ErrorReporter* error_reporter,
                                   int image_width, int image_height,
                                   uint8_t* image_data, Callback callback) {
  // Parse the JPEG headers. The image will be decoded as a sequence of Minimum
  // Coded Units (MCUs), which are 16x8 blocks of pixels.
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
      TF_LITE_REPORT_ERROR(error_reporter, "Frame buffer could not be acquired");
      return kTfLiteError;
  }
  callback(fb->buf, fb->len);
  // Parse the JPEG headers. The image will be decoded as a sequence of Minimum
  // Coded Units (MCUs), which are 16x8 blocks of pixels.
  JpegDec.decodeArray(fb->buf, fb->len);

  // Crop the image by keeping a certain number of MCUs in each dimension
  const int keep_x_mcus = image_width / JpegDec.MCUWidth;
  const int keep_y_mcus = image_height / JpegDec.MCUHeight;

  // Calculate how many MCUs we will throw away on the x axis
  const int skip_x_mcus = JpegDec.MCUSPerRow - keep_x_mcus;
  // Roughly center the crop by skipping half the throwaway MCUs at the
  // beginning of each row
  const int skip_start_x_mcus = skip_x_mcus / 2;
  // Index where we will start throwing away MCUs after the data
  const int skip_end_x_mcu_index = skip_start_x_mcus + keep_x_mcus;
  // Same approach for the columns
  const int skip_y_mcus = JpegDec.MCUSPerCol - keep_y_mcus;
  const int skip_start_y_mcus = skip_y_mcus / 2;
  const int skip_end_y_mcu_index = skip_start_y_mcus + keep_y_mcus;

  // Pointer to the current pixel
  uint16_t* pImg;
  // Color of the current pixel
  uint16_t color;

  // Loop over the MCUs
  while (JpegDec.read()) {
    // Skip over the initial set of rows
    if (JpegDec.MCUy < skip_start_y_mcus) {
      continue;
    }
    // Skip if we're on a column that we don't want
    if (JpegDec.MCUx < skip_start_x_mcus ||
        JpegDec.MCUx >= skip_end_x_mcu_index) {
      continue;
    }
    // Skip if we've got all the rows we want
    if (JpegDec.MCUy >= skip_end_y_mcu_index) {
      continue;
    }
    // Pointer to the current pixel
    pImg = JpegDec.pImage;

    // The x and y indexes of the current MCU, ignoring the MCUs we skip
    int relative_mcu_x = JpegDec.MCUx - skip_start_x_mcus;
    int relative_mcu_y = JpegDec.MCUy - skip_start_y_mcus;

    // The coordinates of the top left of this MCU when applied to the output
    // image
    int x_origin = relative_mcu_x * JpegDec.MCUWidth;
    int y_origin = relative_mcu_y * JpegDec.MCUHeight;

    // Loop through the MCU's rows and columns
    for (int mcu_row = 0; mcu_row < JpegDec.MCUHeight; mcu_row++) {
      // The y coordinate of this pixel in the output index
      int current_y = y_origin + mcu_row;
      for (int mcu_col = 0; mcu_col < JpegDec.MCUWidth; mcu_col++) {
        // Read the color of the pixel as 16-bit integer
        color = *pImg++;
        // Extract the color values (5 red bits, 6 green, 5 blue)
        uint8_t r, g, b;
        r = ((color & 0xF800) >> 11) * 8;
        g = ((color & 0x07E0) >> 5) * 4;
        b = ((color & 0x001F) >> 0) * 8;
        // Convert to grayscale by calculating luminance
        // See https://en.wikipedia.org/wiki/Grayscale for magic numbers
        float gray_value = (0.2126 * r) + (0.7152 * g) + (0.0722 * b);

        // The x coordinate of this pixel in the output image
        int current_x = x_origin + mcu_col;
        // The index of this pixel in our flat output buffer
        int index = (current_y * image_width) + current_x;
        image_data[index] = static_cast<uint8_t>(gray_value);
      }
    }
  }
  esp_camera_fb_return(fb);
  return kTfLiteOk;
}

// Get an image from the camera module
TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter, int image_width,
                      int image_height, int channels, uint8_t* image_data, Callback callback) {
  static bool g_is_camera_initialized = false;
  if (!g_is_camera_initialized) {
    TfLiteStatus init_status = InitCamera(error_reporter);
    if (init_status != kTfLiteOk) {
      TF_LITE_REPORT_ERROR(error_reporter, "InitCamera failed");
      return init_status;
    }
    g_is_camera_initialized = true;
  }
  TfLiteStatus decode_status = DecodeAndProcessImage(
      error_reporter, image_width, image_height, image_data, callback);
  if (decode_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter,"DecodeAndProcessImage failed");
    return decode_status;
  }

  return kTfLiteOk;
}
