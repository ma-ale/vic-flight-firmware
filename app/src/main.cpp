#include <Adafruit_AHRS.h>
#include <Arduino.h>
#include <FreeRTOS.h>
#include <sbus.h>
#include <ESP32Servo.h>

#include "Bluetooth.hpp"
#include "KalmanFilter.hpp"
#include "barometer.h"
#include "board.h"
#include "gps.h"
#include "imu.h"
#include "logger.h"
#include "lora.h"
#include "sdcard.h"
#include "task.h"


SPIClass SPI2(FSPI);
TwoWire I2C1(0);

// State filter for orientation estimation, used in the IMU task
Adafruit_Mahony orientation;

// Altitude and veritcal velocity state filter, used to sense when to deploy
// the parachute
KalmanFilter altitude;

// sbus to read the radio controller input
bfs::SbusRx sbus(&Serial2, RC_SBUS, -1, false);

// Enables SD card writing
static bool enable_sd_card = true;
static int64_t setup_finished_time_ms;

// flags
static bool imu_healthy = true;
static bool baro_healthy = true;
static bool airbrake_active = false;
bool system_armed = true; // FIXME: SYSTEM SHOULD BE STARTED DISARMED

// calibration
float gx_bias = 0;
float gy_bias = 0;
float gz_bias = 0;

// TX lora packet
LoRaDataPacket lora_tx_packet;
DECLARE_STATIC_SEMAPHORE(lora_tx_packet_semaphore);

DECLARE_STATIC_SEMAPHORE(spi_semaphore);
DECLARE_STATIC_SEMAPHORE(i2c_semaphore);
DECLARE_STATIC_SEMAPHORE(altitude_semaphore);

DECLARE_STATIC_TASK(imu_task);
DECLARE_STATIC_TASK(sbus_task);
DECLARE_STATIC_TASK(barometer_task);
DECLARE_STATIC_TASK_STACK(gps_task, TASK_STACK_2K);
DECLARE_STATIC_TASK(lora_transmitter_task);
DECLARE_STATIC_TASK(lora_formatter_task);
DECLARE_STATIC_TASK_STACK(uart_task, TASK_STACK_2K);
DECLARE_STATIC_TASK(sd_formatter_task);
DECLARE_STATIC_TASK_STACK(sd_writer_task, TASK_STACK_2K);
DECLARE_STATIC_TASK(ble_formatter_task);
DECLARE_STATIC_TASK_STACK(cmd_handler_task, TASK_STACK_2K);

DECLARE_STATIC_QUEUE(parachute_msg_queue, LogMessage, LOG_DEFAULT_QUEUE_SIZE);
DECLARE_STATIC_QUEUE(sd_msg_queue, LogMessage, LOG_DEFAULT_QUEUE_SIZE);
DECLARE_STATIC_QUEUE(uart_msg_queue, LogMessage, LOG_DEFAULT_QUEUE_SIZE);
DECLARE_STATIC_QUEUE(lora_msg_queue, LogMessage, LOG_DEFAULT_QUEUE_SIZE);
DECLARE_STATIC_QUEUE(ble_msg_queue, LogMessage, LOG_DEFAULT_QUEUE_SIZE);

// A ground station command, from either radio. Both the BLE write callback and the LoRa RX callback push into
// gs_command_queue, cmd_handler_task is the only reader.
struct GsCommand {
	SourceSubsystem src; // radio it arrived on: S_BLE or S_LORA
	uint8_t command;     // raw id as received (see LoRaCommand), validated by cmd_handler_task
	uint64_t data;       // LoRaCommandPacket::data, always 0 over BLE
};
DECLARE_STATIC_QUEUE(gs_command_queue, GsCommand, 16);

// Queue a ground station command for cmd_handler_task. Never blocks: it is called from the NimBLE host task and
// from the LoRa state machine. Task context only, not ISR safe.
static void gs_command_enqueue(SourceSubsystem src, uint8_t command, uint64_t data)
{
	GsCommand cmd = {.src = src, .command = command, .data = data};
	if (xQueueSend(gs_command_queue, &cmd, 0) != pdTRUE)
		log(src, T_SYSLOG, "Command queue full, command dropped");
}

bool lora_tx_cb(uint8_t* packet)
{
	xSemaphoreTake(lora_tx_packet_semaphore, portMAX_DELAY);
	u64_to_u48le(now_ms(), lora_tx_packet.header.tx_time);
	*((LoRaDataPacket*)packet) = lora_tx_packet;
	xSemaphoreGive(lora_tx_packet_semaphore);
	return true;
}

void lora_rx_cb(uint8_t* packet)
{
	// Called by lora_fc_state_machine(), i.e. from lora_transmitter_task: task context, not an ISR
	LoRaCommandPacket cmd;
	memcpy(&cmd, packet, sizeof(cmd));
	gs_command_enqueue(S_LORA, (uint8_t)cmd.command, cmd.data);
}


void setup(void)
{
	/* ===================== PERIPHERAL INITIALIZATION ====================== */
	Serial.setTxBufferSize(2048);
	Serial.begin(115200);
	uint32_t t0 = millis();
	while (!Serial && millis() - t0 < 3000) delay(10);
	delay(300);
	Serial.println();
	Serial.println("Initialized");
	Serial.printf("DBG reset reason: %d\n", (int)esp_reset_reason());
/*
	pinMode(PIN_DBG_LED, OUTPUT);
	digitalWrite(PIN_DBG_LED, HIGH);
	delay(200);
	digitalWrite(PIN_DBG_LED, LOW);
*/

	Serial.println("Setting up I2C1");
	I2C1.setPins(I2C1_SDA, I2C1_SCL);
	I2C1.begin();
	I2C1.setClock(400000);

	// put all chip selects high, the individual setup calls will handle them later
	Serial.println("Setting up SPI2");
	pinMode(LORA_CS, OUTPUT);
	digitalWrite(LORA_CS, HIGH);

	SPI2.begin(SPI2_SCK, SPI2_MISO, SPI2_MOSI, -1);


	/* ===================== SEMAPHORES INITIALIZATION ====================== */
	Serial.println("Initializing Semaphores");
	INIT_STATIC_SEMAPHORE(spi_semaphore);
	INIT_STATIC_SEMAPHORE(i2c_semaphore);
	INIT_STATIC_SEMAPHORE(altitude_semaphore);
	INIT_STATIC_SEMAPHORE(lora_tx_packet_semaphore);
	if (spi_semaphore == NULL            ||
	    i2c_semaphore == NULL            ||
	    lora_tx_packet_semaphore == NULL ||
	    altitude_semaphore == NULL) {
		while (true) {
			Serial.println("Error creating semaphore");
			delay(500);
		}
	}


	/* ======================= QUEUES INITIALIZATION ======================== */
	Serial.println("Initializing Queues");
	INIT_STATIC_QUEUE(parachute_msg_queue);
	INIT_STATIC_QUEUE(sd_msg_queue);
	INIT_STATIC_QUEUE(uart_msg_queue);
	INIT_STATIC_QUEUE(lora_msg_queue);
	INIT_STATIC_QUEUE(ble_msg_queue);
	INIT_STATIC_QUEUE(gs_command_queue);
	if (parachute_msg_queue == NULL ||
	    sd_msg_queue == NULL        ||
	    uart_msg_queue == NULL      ||
	    lora_msg_queue == NULL      ||
	    gs_command_queue == NULL    ||
	    ble_msg_queue == NULL) {
		while (true) {
			Serial.println("Error creating queues");
			delay(500);
		}
	}


	/* ================= LOGGING & TELEMETRY INITIALIZATION ================= */
	Serial.println("Initializing Logging Subsystem");
	logger_init();

/*
	Serial.println("Initializing SD Card");
	if (!sdcard_init()) {
		while (true) {
			Serial.println("SD init failed");
			delay(500);
		}
	}
	enable_sd_card = true;
	// creates folder /session_<session_num>
	if (!sdcard_start_session()) {
		while (true) {
			Serial.println("Error creating the SD session folder");
			delay(500);
		}
	}
	sdcard_open_log();
*/
	enable_sd_card = false;

/*
	Serial.println("Initializing Bluetooth");
	ble_init({.on_command =
	              [](uint8_t command, void* context) {
		              (void)context;
		              gs_command_enqueue(S_BLE, command, 0);
	              },
	          .context = nullptr});
*/


	Serial.println("Initializing LoRa Radio");
	lora_setup(BAND_ISM2400_BW812, LORA_CHANNEL, TX_FORCE, LORA_FC_ID, false);
	lora_set_tx_packet_cb(lora_tx_cb);
	lora_set_rx_packet_cb(lora_rx_cb);


	/* ======================= SENSORS INITIALIZATION ======================= */
	Serial.println("Initializing IMU");
	if (imu_setup() == false) {
		// FIXME: telemetry should receive this error state
		while (true) {
			log(S_OTHER, T_SYSLOG, "IMU setup failed");
			delay(1000);
		}
	}
	Serial.println("Calibrating IMU");
	float g_cal;
	if (imu_calibrate(&g_cal, &gx_bias, &gy_bias, &gz_bias, IMU_CALIBRATION_TIME) == false) {
		while (true) {
			log(S_OTHER, T_SYSLOG, "IMU calibration failed");
			delay(1000);
		}
	}
	altitude.setG(g_cal);

	Serial.println("Initializing Barometer");
	if (barometer_setup() == false) {
		while (true) {
			log(S_OTHER, T_SYSLOG, "Barometer setup failed");
			delay(1000);
		}
	}
	Serial.println("Calibrating Barometer");
	if (barometer_calibrate(BARO_CALIBRATION_TIME) == false) {
		while (true) {
			log(S_OTHER, T_SYSLOG, "Barometer calibration failed");
			delay(1000);
		}
	}

	Serial.println("Initializing GPS");
	gps_setup();

	/* ======================== TASKS INITIALIZATION ======================== */
	Serial.println("Initializing Tasks");
	// Core 0 tasks
	INIT_STATIC_TASK(sbus_task, "sbus", NULL, tskIDLE_PRIORITY + 11, 0);
	INIT_STATIC_TASK(imu_task, "imu", NULL, tskIDLE_PRIORITY + 10, 0);
	INIT_STATIC_TASK(barometer_task, "barometer", NULL, tskIDLE_PRIORITY + 9, 0);
	INIT_STATIC_TASK(gps_task, "gps", NULL, tskIDLE_PRIORITY + 7, 0);
	// Core 1 tasks
	INIT_STATIC_TASK(sd_formatter_task, "sd formatter", NULL, tskIDLE_PRIORITY + 10, 1);
	INIT_STATIC_TASK(sd_writer_task, "sd writer", NULL, tskIDLE_PRIORITY + 9, 1);
	INIT_STATIC_TASK(lora_formatter_task, "lora formatter", NULL, tskIDLE_PRIORITY + 8, 1);
	INIT_STATIC_TASK(lora_transmitter_task, "lora transmitter", NULL, tskIDLE_PRIORITY + 7, 1);
	INIT_STATIC_TASK(cmd_handler_task, "cmd handler", NULL, tskIDLE_PRIORITY + 6, 1);
	INIT_STATIC_TASK(ble_formatter_task, "ble formatter", NULL, tskIDLE_PRIORITY + 5, 1);
	INIT_STATIC_TASK(uart_task, "logger", NULL, tskIDLE_PRIORITY, 1);

	if (!TASK_IS_INITIALIZED(sbus_task)             ||
	    !TASK_IS_INITIALIZED(imu_task)              ||
	    !TASK_IS_INITIALIZED(barometer_task)        ||
	    !TASK_IS_INITIALIZED(gps_task)              ||
	    !TASK_IS_INITIALIZED(uart_task)             ||
	    !TASK_IS_INITIALIZED(lora_transmitter_task) ||
	    !TASK_IS_INITIALIZED(lora_formatter_task)   ||
	    !TASK_IS_INITIALIZED(sd_formatter_task)     ||
	    !TASK_IS_INITIALIZED(ble_formatter_task)    ||
	    !TASK_IS_INITIALIZED(cmd_handler_task)      ||
	    !TASK_IS_INITIALIZED(sd_writer_task)) {
		while (true) {
			Serial.println("Error creating tasks");
			delay(500);
		}
	}

	Serial.println("Tasks created");
/*
	digitalWrite(PIN_DBG_LED, HIGH);
	delay(200);
	digitalWrite(PIN_DBG_LED, LOW);
*/


	// Register consumer tasks to the logger, these tasks will get notified when
	// new data is ready to be read
	Serial.println("Registering Logging Consumers");
	logger_register_consumer(lora_formatter_task_descriptor.handle, lora_msg_queue, 0xffff, 0xffff);
	logger_register_consumer(sd_formatter_task_descriptor.handle, sd_msg_queue, 0xffff, 0xffff);
	logger_register_consumer(uart_task_descriptor.handle, uart_msg_queue, 0xffff, 0xffff);
	logger_register_consumer(ble_formatter_task_descriptor.handle, ble_msg_queue, 0xffff, 0xffff);
	Serial.println("DBG A: consumers registered");


	/* ====================== PERIPHERAL INITIALIZATION ===================== */
	// TODO

	/* ======================== SERVO INITIALIZATION ======================== */
	// TODO

	// Blink to confirm end of setup
/*
	digitalWrite(PIN_DBG_LED, HIGH);
	delay(200);
	digitalWrite(PIN_DBG_LED, LOW);
	delay(200);
	digitalWrite(PIN_DBG_LED, HIGH);
	delay(200);
	digitalWrite(PIN_DBG_LED, LOW);
	delay(200);
	digitalWrite(PIN_DBG_LED, HIGH);
	delay(200);
	digitalWrite(PIN_DBG_LED, LOW);
	delay(200);
	digitalWrite(PIN_DBG_LED, HIGH);
	delay(200);
	digitalWrite(PIN_DBG_LED, LOW);
	delay(200);
*/

	setup_finished_time_ms = millis();
}


// Heartbeat
void loop(void)
{
/*
	digitalWrite(PIN_DBG_LED, HIGH);
	delay(1000);
	digitalWrite(PIN_DBG_LED, LOW);
*/
	delay(1000);
}


TASK sbus_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();
	sbus.Begin();

	static Servo s_flap_r;
	static Servo s_flap_l;
	static Servo s_aile_r;
	static Servo s_aile_l;
	static Servo s_tail_r;
	static Servo s_tail_l;

	s_flap_r.attach(SRV_FLAP_R, SERVO_MIN_US, SERVO_MAX_US);
	s_flap_l.attach(SRV_FLAP_L, SERVO_MIN_US, SERVO_MAX_US);
	s_aile_r.attach(SRV_AILE_R, SERVO_MIN_US, SERVO_MAX_US);
	s_aile_l.attach(SRV_AILE_L, SERVO_MIN_US, SERVO_MAX_US);
	s_tail_r.attach(SRV_TAIL_R, SERVO_MIN_US, SERVO_MAX_US);
	s_tail_l.attach(SRV_TAIL_L, SERVO_MIN_US, SERVO_MAX_US);

	#define MAX_SBUS_CHANNELS 16
	static Servo * const SBUS_CHANNEL_MAP[MAX_SBUS_CHANNELS] = {
		[0]  = &s_flap_r,
		[1]  = &s_flap_l,
		[2]  = &s_aile_r,
		[3]  = &s_aile_l,
		[4]  = &s_tail_r,
		[5]  = &s_tail_l,
		[6]  = nullptr,
		[7]  = nullptr,
		[8]  = nullptr,
		[9]  = nullptr,
		[10] = nullptr,
		[11] = nullptr,
		[12] = nullptr,
		[13] = nullptr,
		[14] = nullptr,
		[15] = nullptr,
	};


	while (true) {
		if (sbus.Read()) {
			bfs::SbusData data = sbus.data();
			for (int i = 0; i < 16; i++) {
				if (SBUS_CHANNEL_MAP[i] != nullptr) {
					int angle = map(data.ch[i], SBUS_MIN, SBUS_MAX, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
					//Serial.printf("SERVO %d: %d\n", i, angle);
					SBUS_CHANNEL_MAP[i]->write(angle);
				}
			}
		}
		vTaskDelay(1);
	}
}


TASK imu_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();
	FIFO_Sample sample;
	orientation.begin(IMU_TASK_HZ);

	while (true) {
		if (xSemaphoreTake(i2c_semaphore, portMAX_DELAY) == pdTRUE) {
			if (imu_get_sample(&sample) == 0) {
				float ax = (float)sample.accelerometer[0] / 1000.0f;
				float ay = (float)sample.accelerometer[1] / 1000.0f;
				float az = (float)sample.accelerometer[2] / 1000.0f;
				float gx = ((float)sample.gyroscope[0] - gx_bias) / 1000.0f;
				float gy = ((float)sample.gyroscope[1] - gy_bias) / 1000.0f;
				float gz = ((float)sample.gyroscope[2] - gz_bias) / 1000.0f;

/*
				//first minute after setup check sensor health
				if (millis() - setup_finished_time_ms < GROUND_CHECK_TIMER_MS) {
					imu_healthy = is_imu_healthy_ground(ax, ay, az, gx, gy, gz);
					if (!imu_healthy) 	log(S_OTHER, T_SYSLOG, "IMU not healthy");
				}
*/

				// Mahony filter quaternions
				float qw, qx, qy, qz;

				// Update the relative orientation of the board using
				// the Mahony filter, readings are in mg and mdps, so
				// conversion is needed
				orientation.updateIMU(gx, gy, gz, ax, ay, az);
				orientation.getQuaternion(&qw, &qx, &qy, &qz);

				// Update the altitude and vertical velocity estimation
				// with the inertial data
				float cos_tilt = constrain(1 - 2*(qx*qx + qy*qy), -1.0f, 1.0f);
				float attitude_rad = acosf(cos_tilt); // same as acos(cos p * cos r) but NaN-safe
				float vertical_accel = 2*(qx*qz - qw*qy) * ax + 2*(qy*qz + qw*qx) * ay + (1 - 2*(qx*qx + qy*qy)) * az;
				// TODO: airbrake trigger
				if (xSemaphoreTake(altitude_semaphore, portMAX_DELAY) == pdTRUE) {
					altitude.predict(vertical_accel, attitude_rad, airbrake_active, true);
					xSemaphoreGive(altitude_semaphore);
				}

				Vector3f fs = altitude.getState();
				log(S_IMU, T_ORIENTATION, orientation.getRoll(), orientation.getPitch(), orientation.getYaw());
				log(S_IMU, T_ACCELLERATION, sample.accelerometer[0], sample.accelerometer[1], sample.accelerometer[2]);
				log(S_IMU, T_GYRO, gx, gy, gz);
				log(S_IMU, T_FILTER_STATE, fs[0], fs[1], fs[2]);
			} else {
				// IMU reading was not ok, still predict with constant speed model
				if (xSemaphoreTake(altitude_semaphore, portMAX_DELAY) == pdTRUE) {
					altitude.predict(0, 0, airbrake_active, false);
					xSemaphoreGive(altitude_semaphore);
				}
			}
			xSemaphoreGive(i2c_semaphore);
		}
		TASK_WAIT_HZ(self, IMU_TASK_HZ);
	}
}

TASK barometer_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();
	BaroData sample;

	while (true) {
		if (xSemaphoreTake(i2c_semaphore, portMAX_DELAY) == pdTRUE) {
			sample = barometer_read();

/*
			// first minute after setup check sensor health
			if (millis() - setup_finished_time_ms < GROUND_CHECK_TIMER_MS) {
				baro_healthy = is_baro_healthy_ground(sample1.pressure, sample2.pressure, sample1.temperature, sample2.temperature);
				if (!baro_healthy) 	log(S_OTHER, T_SYSLOG, "Baro not healthy");
			}
*/

			// Update the altitude and vertical velocity estimation with the barometer data
			if (sample.error_status == 0) {
				if (xSemaphoreTake(altitude_semaphore, portMAX_DELAY) == pdTRUE) {
					altitude.update(sample.altitude);
					xSemaphoreGive(altitude_semaphore);
				}
			} else {
				log(S_BARO, T_SYSLOG, "[ERR] Reading barometers");
			}

			Vector3f fs = altitude.getState();
			log(S_BARO, T_FILTER_STATE, fs[0], fs[1], fs[2]);
			log(S_BARO, T_PRESSURE, sample.pressure);
			// FIME: make a barometric altitude and temperature log message
			log(S_BARO, T_SYSLOG, sample.altitude);
			log(S_BARO, T_SYSLOG, sample.temperature, -777.0f);

			xSemaphoreGive(i2c_semaphore);
		}
	TASK_WAIT_HZ(self, BARO_TASK_HZ);
	}
}

TASK gps_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();
	static GPSData data;

	while (true) {
		gps_update(&data);

		if (data.num_sat < GPS_MIN_SATELLITES) {
			log(S_GPS, T_SYSLOG, "Not enough satellites");
		} else {
			// LOG("[GPS]: (%d) pos: (%f, %f), alt: %fm, speed: %fkmh, time:%llu",
			//	data.num_sat, data.lat, data.lon, data.alt, data.kmh, data.unix_time);
			log(S_GPS, T_GPS, data.lat, data.lon);
		}

		TASK_WAIT_HZ(self, GPS_TASK_HZ);
	}
}

TASK ble_formatter_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();

	while (true) {
		// TEST: Blocking receive, if good => apply to all formatter tasks
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		LogMessage msg;

		while (xQueueReceive(ble_msg_queue, &msg, 0) == pdTRUE) {
			//ble_send_log_message(msg);
		}
	}
}

TASK lora_formatter_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();

	while (true) {
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000 / LORA_FMT_TASK_HZ));

		LogMessage msg;

		xSemaphoreTake(lora_tx_packet_semaphore, portMAX_DELAY);
		while (xQueueReceive(lora_msg_queue, &msg, 0) == pdTRUE) {
			switch (msg.type) {
			case T_FILTER_STATE:
				if (msg.payload_type == P_FVEC3) {
					lora_tx_packet.imu.altitude = float16(msg.payload.fv3.x).getBinary();
					lora_tx_packet.imu.vspeed = float16(msg.payload.fv3.y).getBinary();
					lora_tx_packet.imu.dt = msg.timestamp / 1000 - u48le_to_u64(lora_tx_packet.header.tx_time);
				}
				break;
			case T_ORIENTATION: {
				if (msg.payload_type == P_FVEC3) {
					// FIXME: don't repeat this computation here
					float r = msg.payload.fv3.x * 0.0174533;   // roll in radians
					float p = msg.payload.fv3.y * 0.0174533;   // pitch in radians
					float a = acos(cos(p) * cos(r)) * 57.2958; // total pitch from vertical in degrees

					lora_tx_packet.imu.attitude = float16(a).getBinary();
					lora_tx_packet.imu.dt = msg.timestamp / 1000 - u48le_to_u64(lora_tx_packet.header.tx_time);
				}
				break;
			}
			case T_PRESSURE:
				if (msg.payload_type == P_FVEC2) {
					// Pa -> hPa: float16 max is 65504, so Pa would overflow
					lora_tx_packet.baro.p1 = float16(msg.payload.fv2.x / 100.0f).getBinary();
					lora_tx_packet.baro.p2 = float16(msg.payload.fv2.y / 100.0f).getBinary();
					lora_tx_packet.baro.dt = msg.timestamp / 1000 - u48le_to_u64(lora_tx_packet.header.tx_time);
				}
				break;
			case T_GPS:
				if (msg.payload_type == P_FVEC2) {
					lora_tx_packet.gps.latitude = msg.payload.fv2.x;
					lora_tx_packet.gps.longitude = msg.payload.fv2.y;
					lora_tx_packet.gps.dt = msg.timestamp / 1000 - u48le_to_u64(lora_tx_packet.header.tx_time);
				}
				break;
			case T_ROCKET_STATE:
				if (msg.payload_type == P_ROCKET_STATE) {
					lora_tx_packet.state = (uint8_t)msg.payload.state;
				}
				break;
			default:
				break;
			}
		}
		// TODO: this should be better
		lora_tx_packet.state |= system_armed << 7;
		lora_tx_packet.sensor_status = imu_healthy << 0 | baro_healthy << 1;
		xSemaphoreGive(lora_tx_packet_semaphore);
	}
}

TASK lora_transmitter_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();
	LoRaProtoState last_state = STATE_DISCONNECTED;

	while (true) {
/*
		// Run the lora radio state machine
		LoRaProtoState state = lora_fc_state_machine();
		String str;
		switch (state) {
		case STATE_DISCONNECTED:
			str = "STATE_DISCONNECTED";
			break;
		case STATE_CONNECTING:
			str = "STATE_CONNECTING";
			break;
		case STATE_TRANSMIT:
			str = "STATE_TRANSMIT";
			break;
		case STATE_RECEIVE:
			str = "STATE_RECEIVE";
			break;
		default:
			str = "UNKNOWN";
			break;
		}
		if (state != last_state) {
			log(S_LORA, T_SYSLOG, state);
			last_state = state;
		}
*/
		TASK_WAIT_HZ(self, LORA_TX_TASK_HZ);
	}
}

// UART consumer
TASK uart_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();

	while (true) {
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000 / UART_TASK_HZ));
		LogMessage msg;
		char str[LOGGER_STRING_BUFFER_SIZE];
		size_t len = 0;
		while (xQueueReceive(uart_msg_queue, &msg, 0) == pdTRUE) {
			len = logger_message_to_str(str, &msg);
			//Serial.write(str, len);
		}
	}
}

// SD consumer
TASK sd_formatter_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();

	while (true) {
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000 / SD_FMT_TASK_HZ));
		LogMessage msg;

		while (xQueueReceive(sd_msg_queue, &msg, 0) == pdTRUE) {
			if (enable_sd_card == false) {
				continue;
			}
			char str[LOGGER_STRING_BUFFER_SIZE];
			size_t len = logger_message_to_str(str, &msg);
			sdcard_write(str, len);
		}
	}
}

TASK sd_writer_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();

	while (true) {
		if (enable_sd_card == true) {
			sdcard_flush();
		}
		TASK_WAIT_HZ(self, SD_WRITER_TASK_HZ);
	}
}

TASK cmd_handler_task(TaskDescriptor_t* self)
{
	self->last_wake = xTaskGetTickCount();

	while (true) {
		GsCommand cmd;
		if (xQueueReceive(gs_command_queue, &cmd, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		// The log source tells which radio the command came from (S_BLE or S_LORA).
		// Every command is only logged for now: none of them drives an output yet, and in particular
		// the pyro commands must not fire anything until the team decides how they are armed.
		switch (cmd.command) {
		case CMD_NONE:
			log(cmd.src, T_SYSLOG, "Received command: NONE");
			break;
		case CMD_EJECT_A:
			log(cmd.src, T_SYSLOG, "Received command: EJECT_A");
			break;
		case CMD_EJECT_C:
			log(cmd.src, T_SYSLOG, "Received command: EJECT_C");
			break;
		case CMD_CUT_MAIN:
			log(cmd.src, T_SYSLOG, "Received command: CUT_MAIN");
			break;
		case CMD_SENSOR_CALIBRATION:
			log(cmd.src, T_SYSLOG, "Received command: SENSOR_CALIBRATION");
			if (xSemaphoreTake(i2c_semaphore, portMAX_DELAY) == pdTRUE) {
				// IMU calibration
				float g_cal;
				if (imu_calibrate(&g_cal, &gx_bias, &gy_bias, &gz_bias, IMU_CALIBRATION_TIME) == true) {
					if (xSemaphoreTake(altitude_semaphore, portMAX_DELAY) == pdTRUE) {
						altitude.setG(g_cal);
						xSemaphoreGive(altitude_semaphore);
					}
				} else {
					log(cmd.src, T_SYSLOG, "IMU Calibration Failed");
				}

				// Barometer Calibration
				if (barometer_calibrate(BARO_CALIBRATION_TIME) == false) {
					log(cmd.src, T_SYSLOG, "Baro Calibration Failed");
				}
				xSemaphoreGive(i2c_semaphore);
			}
			break;
		case CMD_SYSTEM_ARM:
			log(cmd.src, T_SYSLOG, "Received command: SYSTEM_ARM");
			system_armed = true;
			break;
		case CMD_SYSTEM_DISARM:
			log(cmd.src, T_SYSLOG, "Received command: SYSTEM_DISARM");
			system_armed = false;
			break;
		case CMD_SET_GROUND_TEMP:
			log(cmd.src, T_SYSLOG, "Received command: UPDATE_GROUND_TEMP");
			barometer_set_ground_temp_c((float)cmd.data);
			break;
		default:
			log(cmd.src, T_SYSLOG, "Received unknown command");
			break;
		}
	}
}
