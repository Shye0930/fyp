#include "esp_camera.h"
#include <WiFi.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"


// Manual Exposure/Gain/Brightness values - TUNE THESE CAREFULLY!
#define MANUAL_AEC_VALUE 600 // Range 0-1200. Experiment with this heavily!
#define MANUAL_AGC_GAIN 1   // Range 0-30. 0 is lowest gain (least noisy). Experiment.
#define BRIGHTNESS_VALUE 0  // Range -2 to 2. 0 is normal.
#define CONTRAST_VALUE 0    // Range -2 to 2. 0 is normal.
#define SATURATION_VALUE 0  // Range -2 to 2. 0 is normal (will be irrelevant for grayscale, but good to set).


// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "fyp";
const char *password = "";

framesize_t current_cam_framesize;
int current_cam_quality;
gainceiling_t current_cam_gain;

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.xclk_freq_hz = 20000000;//24000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_count = 2;


  current_cam_framesize = FRAMESIZE_QVGA;
  current_cam_quality = 20; //10-63 lower number means higher quality
  current_cam_gain = (gainceiling_t) 0;

  config.frame_size = current_cam_framesize;
  config.jpeg_quality = current_cam_quality; 
  config.grab_mode = CAMERA_GRAB_LATEST;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  // if (config.pixel_format == PIXFORMAT_JPEG) {
  //   if (psramFound()) {
  //     config.jpeg_quality = 12;
  //     config.fb_count = 2;
  //     config.grab_mode = CAMERA_GRAB_LATEST;
  //   } else {
      
  //     // Limit the frame size when PSRAM is not available
  //     config.frame_size = FRAMESIZE_SVGA;
  //     config.fb_location = CAMERA_FB_IN_DRAM;
  //   }
  // } 

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  
  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated

  if(s->id.PID == OV2640_PID){
    // s->xclk_freq_hz = 24000000;
    // s->set_xclk(s, LEDC_TIMER_0, 24000000);
    s->set_vflip(s, 1);        // flip it back
    s->set_hmirror(s,1);
    s->set_dcw(s, 1);           // Digital Crop Window. 

    s->set_whitebal(s, 0);      // 0 = disable, 1 = enable
    s->set_awb_gain(s, 0);      // AWB gain control. Disable this too.

    s->set_raw_gma(s,1);        // Allow raw gamma 

    s->set_bpc(s,0);

    s->set_exposure_ctrl(s, 1); // 0 = disable, 1 = enable
    s->set_aec2(s,1);
    s->set_ae_level(s,0);

    s->set_gainceiling(s, current_cam_gain); // Set a low gain ceiling even if AGC is off, to prevent excessive noise if somehow enabled.
    s->set_gain_ctrl(s, 0);     // 0 = disable, 1 = enable
    s->set_agc_gain(s, MANUAL_AGC_GAIN); // Set manual gain value when AGC is off.

    // 4. Other image enhancements/auto adjustments
    s->set_brightness(s, BRIGHTNESS_VALUE); // -2 to 2
    s->set_contrast(s, CONTRAST_VALUE);     // -2 to 2
    s->set_saturation(s, SATURATION_VALUE); // -2 to 2 (Irrelevant for grayscale, but good for completeness)
    s->set_special_effect(s, 2); // grayscale

  }

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  Serial.printf("xclk freq is %d Mhz \n", s->xclk_freq_hz/1000000);

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  if(psramFound()){
    Serial.println("PSRAM available");
  }

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // // Do nothing. Everything is done in another task by the web server
  delay(10000);
}
// #include <ICM_20948.h>

// #include "esp_camera.h"
// #include <WiFi.h>

// //
// // WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
// //            Ensure ESP32 Wrover Module or other board with PSRAM is selected
// //            Partial images will be transmitted if image exceeds buffer size
// //
// //            You must select partition scheme from the board menu that has at least 3MB APP space.
// //            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
// //            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// // ===================
// // Select camera model
// // ===================
// //#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
// //#define CAMERA_MODEL_ESP_EYE  // Has PSRAM
// //#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
// //#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
// //#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
// //#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
// //#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
// //#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
// //#define CAMERA_MODEL_M5STACK_CAMS3_UNIT  // Has PSRAM
// #define CAMERA_MODEL_AI_THINKER // Has PSRAM
// //#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
// //#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// // ** Espressif Internal Boards **
// //#define CAMERA_MODEL_ESP32_CAM_BOARD
// //#define CAMERA_MODEL_ESP32S2_CAM_BOARD
// //#define CAMERA_MODEL_ESP32S3_CAM_LCD
// //#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
// //#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
// #include "camera_pins.h"

// // ===========================
// // Enter your WiFi credentials
// // ===========================
// const char *ssid = "🍙"; //"SimEco";//"🍙"; //"raspberrypi"; //
// const char *password ="ad28kxcp"; //""aLsYeN@007219";//"ad28kxcp"; //"raspberry30#?"; //

// void startCameraServer();
// void setupLedFlash(int pin);

// void setup() {
//   Serial.begin(115200);
//   Serial.setDebugOutput(true);
//   Serial.println();

//   camera_config_t config;
//   config.ledc_channel = LEDC_CHANNEL_0;
//   config.ledc_timer = LEDC_TIMER_0;
//   config.pin_d0 = Y2_GPIO_NUM;
//   config.pin_d1 = Y3_GPIO_NUM;
//   config.pin_d2 = Y4_GPIO_NUM;
//   config.pin_d3 = Y5_GPIO_NUM;
//   config.pin_d4 = Y6_GPIO_NUM;
//   config.pin_d5 = Y7_GPIO_NUM;
//   config.pin_d6 = Y8_GPIO_NUM;
//   config.pin_d7 = Y9_GPIO_NUM;
//   config.pin_xclk = XCLK_GPIO_NUM;
//   config.pin_pclk = PCLK_GPIO_NUM;
//   config.pin_vsync = VSYNC_GPIO_NUM;
//   config.pin_href = HREF_GPIO_NUM;
//   config.pin_sccb_sda = SIOD_GPIO_NUM;
//   config.pin_sccb_scl = SIOC_GPIO_NUM;
//   config.pin_pwdn = PWDN_GPIO_NUM;
//   config.pin_reset = RESET_GPIO_NUM;
//   config.xclk_freq_hz = 10000000 ; // TOOD: I change here 20000000 8000000;
//   config.frame_size = FRAMESIZE_UXGA;
//   config.pixel_format = PIXFORMAT_JPEG;  // for streaming
//   //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
//   config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
//   config.fb_location = CAMERA_FB_IN_PSRAM;
//   config.jpeg_quality = 12;
//   config.fb_count = 1;

//   // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
//   //                      for larger pre-allocated frame buffer.
//   if (config.pixel_format == PIXFORMAT_JPEG) {
//     if (psramFound()) {
//       config.jpeg_quality = 10;
//       config.fb_count = 2;
//       config.grab_mode = CAMERA_GRAB_LATEST;
//     } else {
//       // Limit the frame size when PSRAM is not available
//       config.frame_size = FRAMESIZE_SVGA;
//       config.fb_location = CAMERA_FB_IN_DRAM;
//     }
//   } else {
//     // Best option for face detection/recognition
//     config.frame_size = FRAMESIZE_VGA;
// #if CONFIG_IDF_TARGET_ESP32S3
//     config.fb_count = 2;
// #endif
//   }

// #if defined(CAMERA_MODEL_ESP_EYE)
//   pinMode(13, INPUT_PULLUP);
//   pinMode(14, INPUT_PULLUP);
// #endif

//   // camera init
//   esp_err_t err = esp_camera_init(&config);
//   if (err != ESP_OK) {
//     Serial.printf("Camera init failed with error 0x%x", err);
//     return;
//   }

  
//   sensor_t *s = esp_camera_sensor_get();
//   // initial sensors are flipped vertically and colors are a bit saturated

//   if(s->id.PID == OV2640_PID){
//     s->set_vflip(s, 1);        // flip it back
//     s->set_brightness(s, -2);   // up the brightness just a bit
//     s->set_contrast(s, 0);
//     s->set_saturation(s,2);
//     s->set_hmirror(s,1);
//     config.jpeg_quality = 10;

//   }

//   if (s->id.PID == OV3660_PID) {
//     s->set_vflip(s, 1);        // flip it back
//     s->set_brightness(s, 1);   // up the brightness just a bit
//     s->set_saturation(s, -2);  // lower the saturation
//   }
//   // drop down frame size for higher initial frame rate
//   if (config.pixel_format == PIXFORMAT_JPEG) {
//     s->set_framesize(s, FRAMESIZE_QVGA);
//   }

// #if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
//   s->set_vflip(s, 1);
//   s->set_hmirror(s, 1);
// #endif

// #if defined(CAMERA_MODEL_ESP32S3_EYE)
//   s->set_vflip(s, 1);
// #endif

// // Setup LED FLash if LED pin is defined in camera_pins.h
// #if defined(LED_GPIO_NUM)
//   setupLedFlash(LED_GPIO_NUM);
// #endif

//   WiFi.begin(ssid, password);
//   WiFi.setSleep(false);

//   Serial.print("WiFi connecting");
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("");
//   Serial.println("WiFi connected");

//   startCameraServer();

//   Serial.print("Camera Ready! Use 'http://");
//   Serial.print(WiFi.localIP());
//   Serial.println("' to connect");
// }

// void loop() {
//   // // Do nothing. Everything is done in another task by the web server
//   delay(10000);
// }
