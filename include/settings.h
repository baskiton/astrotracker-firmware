#pragma once

// TRACKER
#define TRACKER_WIFI_AP

#define STEP_E_PIN 25
#define STEP_D_PIN 26
#define STEP_S_PIN 27
#define SHUTTER_PIN 23

/*
 TMC2209 speed map
 MS2    MS1     microstep
   0      0     1/8
   0      1     1/32
   1      0     1/64
   1      1     1/16
 */
#define STEPPER_HIGH_TIME_US 4 // for TMC2209 is 100 ns, btw
#define _MICROSTEPS 64
#define _MOTOR_STEPS_PER_REV 400
#define _REDUCTION 101.25
#define TARGET_SPEED_DEG (15.0 / 3600)  // deg/s (15'/sec)
#define STEPS_PER_DEG ((_MOTOR_STEPS_PER_REV * _MICROSTEPS * _REDUCTION) / 360.0)
#define STEPS_PER_REV (_REDUCTION * _MOTOR_STEPS_PER_REV * _MICROSTEPS)
#define SPS (STEPS_PER_REV / (3600 * 24))     // steps/s

#define SHUTTER_DIV 40000
#define SHUTTER_MUL 2000

// Camera defaults
#define CAMERA_MIN_INTV 2
#define CAMERA_EXPOSURE 30
#define CAMERA_INTV CAMERA_MIN_INTV
#define CAMERA_SHOT_DELAY 0.2

// BRIDGES
// #define GH_NO_STREAM  // отключить встроенный модуль связи stream
// #define GH_NO_HTTP    // отключить встроенный модуль связи http (для esp)
// #define GH_NO_WS      // отключить встроенный модуль связи ws (для esp)
// #define GH_NO_MQTT    // отключить встроенный модуль связи mqtt (для esp)

// #define GH_BRIDGE_AMOUNT 5       // количество мостов подключения
// #define GH_WS_PORT 81            // websocket порт
// #define GH_MQTT_RECONNECT 10000  // период переподключения встроенного MQTT (для esp)

// MODULES
// #define GH_NO_PAIRS    // отключить поддержку типа Pairs
// #define GH_NO_GET      // отключить отправку на get-топик (mqtt)
// #define GH_NO_INFO     // отключить info билдер
// #define GH_NO_MODULES  // отключить "модули" (все включены)
// #define GH_NO_REQUEST  // отключить поддержку запросов

// FS
// #define GH_NO_FS                 // отключить файловую систему (для esp)
// #define GH_FETCH_CHUNK_SIZE 512  // размер чанка при скачивании с платы (для esp)
// #define GH_UPL_CHUNK_SIZE 512    // размер чанка при загрузке на плату (для esp)
// #define GH_FS_DEPTH 5            // глубина сканирования файловой системы (esp32)

// TRANSFER
// #define GH_NO_FETCH    // отключить скачивание файлов (для esp)
// #define GH_NO_UPLOAD   // отключить загрузку файлов (для esp)
// #define GH_NO_OTA      // отключить ОТА обновления (для esp)
// #define GH_NO_OTA_URL  // отключить ОТА по ссылке (для esp)
// #define GH_REBOOT_DELAY 1500    // задержка перед перезагрузкой после OTA (для esp)

// PORTAL
// #define GH_FILE_PORTAL     // загружать сайт из памяти есп (положи файлы в папку /hub/) (для esp)
#define GH_INCLUDE_PORTAL  // включить сайт в память программы, ~50кБ (не нужно загружать файлы в память) (для esp)

// HTTP
// #define GH_PUBLIC_PATH "/www"          // путь к папке с файлами с HTTP доступом (для esp)
// #define GH_HTTP_PORT 80                // http порт (для esp)
// #define GH_CACHE_PRD "max-age=604800"  // период кеширования файлов для портала (для esp)

// #define GH_NO_HTTP_TRANSFER       // отключить загрузку/скачивание/ОТА по http (для esp)
// #define GH_NO_HTTP_DNS            // отключить DNS сервер (для esp)
// #define GH_NO_HTTP_OTA            // отключить ОТА обновления по http (для esp)
// #define GH_NO_HTTP_PUBLIC         // отключить доступ к файлам по http c ip/www (для esp)
// #define GH_NO_HTTP_FETCH          // отключить скачивание файлов по http (для esp)
// #define GH_NO_HTTP_UPLOAD         // отключить загрузку файлов по http (для esp)
// #define GH_NO_HTTP_UPLOAD_PORTAL  // упрощённую загрузку файлов с ip/hub/upload_portal (для esp)
