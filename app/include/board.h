#pragma once


// SPI2 Bus, FSPI
#define SPI2_MOSI 11
#define SPI2_MISO 13
#define SPI2_SCK  12

// I2C1 Bus
#define I2C1_SDA 36
#define I2C1_SCL 35

// IMU Pins
#define IMU_INT1  37

// LoRa Pins
#define LORA_BUSY 21
#define LORA_RST  47
#define LORA_DIO1 48
#define LORA_CS   14

// RC Receiver Pins
#define RC_SBUS 38

// GPS Pins
// GPS is on UART0, keep default pins
#define GPS_RST 1
#define GPS_PPS 2

// SD Pins
#define SD_CLK_PIN 18
#define SD_CMD_PIN 8
#define SD_D0_PIN  17

// Sensor Peripherals
#define RPM_SENSE  3
#define TEMP_ADC1  6
#define TEMP_ADC2  7
#define VSENSE_ADC 4
#define ISENSE_ADC 5

// ESC Pins
#define ESC_FORWARD 39
#define ESC_REVERSE 40

// Servo Pins
#define SRV_FLAP_L 16
#define SRV_FLAP_R 41
#define SRV_AILE_L 15
#define SRV_AILE_R 42
#define SRV_TAIL_L 9
#define SRV_TAIL_R 10

// Task Timings
#define IMU_TASK_HZ       100
#define BARO_TASK_HZ      20
#define GPS_TASK_HZ       2
#define UART_TASK_HZ      50
#define LORA_FMT_TASK_HZ  20
#define LORA_TX_TASK_HZ   50
#define SD_FMT_TASK_HZ    50
#define SD_WRITER_TASK_HZ 10
#define PARACHUTE_TASK_HZ 10


// IMU FIFO Configuration
// IMU_FIFO_x_BDR_HZ should be >= IMU_TASK_HZ to always have samples to read but
// not too high to cause overflow in the FIFO and/or excessive CPU usage in the
// IMU task
#define IMU_FIFO_ENABLE    0
#define IMU_FIFO_WATERMARK 4
#define IMU_FIFO_X_BDR_HZ  (IMU_TASK_HZ * 1.20f)
#define IMU_FIFO_G_BDR_HZ  (IMU_TASK_HZ * 1.20f)
#define IMU_CALIBRATION_TIME 5000


// Barometer configuration
// Oversampling rate: this affects the preision and sampling time, for the two
// barometers ULTRA_HIGH means about 70ms per both samples, high 50ms
// OSR_STANDARD works for 20Hz sampling
//#define BARO_OSR OSR_STANDARD
#define BARO_OSR OSR_ULTRA_HIGH
#define BARO_CALIBRATION_TIME 5000


// LoRa configuration
#define LORA_OUTPUT_POWER     0     // dBm, da capire
#define LORA_CHANNEL          1
#define LORA_SPREADING_FACTOR 7     // minimo spread factor
#define LORA_CODING_RATE      5     // 4 + 1, 1 bit su 8 di correzione di errore
#define LORA_CRC_BYTES        1


// GPS configuration
#define GPS_MIN_SATELLITES         3
#define GPS_SERIAL_BUFFER_SIZE     1024
#define GPS_SERIAL_READ_TIMEOUT_MS 100


// Parachute task parameters
#define Z_ACC_BOOST_THRESHOLD_G          3
#define Z_SPEED_BOOST_THRESHOLD_MPS      25.0
#define Z_SPEED_APOGEE_THRESHOLD_MPS     0.0
#define MAX_ALT_BURNOUT_M                700
#define MIN_AIRBRAKE_ALT_M               1500
#define Z_ALT_MAIN_DEPLOYMENT_M          400.0
#define Z_ALT_TOUCHDOWN_M                10.0
#define Z_SPEED_STATIONARY_MPS           0.1
#define BOOST_DETECTION_SAMPLE_COUNT     10
#define BURNOUT_DETECTION_SAMPLE_COUNT   10
#define APOGEE_DETECTION_SAMPLE_COUNT    10
#define MAIN_DETECTION_SAMPLE_COUNT      10
#define TOUCHDOWN_DETECTION_SAMPLE_COUNT 10
#define CUTTERS_ON_TIME_MS               3000
#define EJECTION_CO2_DELAY_MS            2000
#define EMATCH_ON_TIME_MS                1000
#define STOP_SD_WRITE_MS                 600000


// Others
#define GROUND_CHECK_TIMER_MS            60*1000
#define DBG_LED_PWM                      255/3
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
