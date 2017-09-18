#define __BTSTACK_FILE__ "spp_counter.c"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "bno055.h"
#include "imu.h"
#include "btstack_run_loop_freertos.h"
#include "btstack.h"

#define RFCOMM_SERVER_CHANNEL 1
#define HEARTBEAT_PERIOD_MS 30
#define ADC1_TEST_CHANNEL0 (0)
#define ADC1_TEST_CHANNEL3 (3)
#define ADC1_TEST_CHANNEL4 (4)
#define ADC1_TEST_CHANNEL5 (5)
#define ADC1_TEST_CHANNEL6 (6)
#define ADC1_TEST_CHANNEL7 (7)
int a0, a3, a4, a5, a6, a7, yaw, pit, rol, countt;
s16 heading, pitch, roll;
float rollDeg, pitchDeg, yawDeg;

static void packet_handler(uint8_t packet_type, uint16_t channel,
		uint8_t *packet, uint16_t size);

static uint16_t rfcomm_channel_id;
static uint8_t spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;

static void initADC(void) {
	// initialize ADC
	adc1_config_width(ADC_WIDTH_12Bit);
	adc1_config_channel_atten(ADC1_TEST_CHANNEL0, ADC_ATTEN_11db);
	adc1_config_channel_atten(ADC1_TEST_CHANNEL3, ADC_ATTEN_11db);
	adc1_config_channel_atten(ADC1_TEST_CHANNEL4, ADC_ATTEN_11db);
	adc1_config_channel_atten(ADC1_TEST_CHANNEL5, ADC_ATTEN_11db);
	adc1_config_channel_atten(ADC1_TEST_CHANNEL6, ADC_ATTEN_11db);
	adc1_config_channel_atten(ADC1_TEST_CHANNEL7, ADC_ATTEN_11db);
}

static void spp_service_setup(void) {
	// register for HCI events
	countt = 0;
	hci_event_callback_registration.callback = &packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);
	l2cap_init();
	initADC();
	rfcomm_init();
	rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff); // reserved channel, mtu limited by l2cap

	// init SDP, create record for SPP and register with SDP
	sdp_init();
	memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
	spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL,
			"SPP Counter");
	sdp_register_service(spp_service_buffer);
	printf("SDP service record size: %u\n", de_get_len(spp_service_buffer));
}

static btstack_timer_source_t heartbeat;
static char lineBuffer[200];
static void heartbeat_handler(struct btstack_timer_source *ts) {
	if (rfcomm_channel_id) {
		a0 = adc1_get_voltage(ADC1_TEST_CHANNEL0);
		a3 = adc1_get_voltage(ADC1_TEST_CHANNEL3);
		a4 = adc1_get_voltage(ADC1_TEST_CHANNEL4);
		a5 = adc1_get_voltage(ADC1_TEST_CHANNEL5);
		a6 = adc1_get_voltage(ADC1_TEST_CHANNEL6);
		a7 = adc1_get_voltage(ADC1_TEST_CHANNEL7);
		sprintf(lineBuffer,
		 "ADC0: %d,ADC3: %d,ADC4: %d,ADC5: %d,ADC6: %d,ADC7: %d,head: %.2f pitch: %.2f roll: %.2f \n", a0,
		 a3, a4, a5, a6, a7,yawDeg, pitchDeg,rollDeg);
//		sprintf(lineBuffer,
//				"ADC0: %d,ADC3: %d,ADC4: %d,ADC5: %d,ADC6: %d,ADC7: %d \n", a0,
//				a3, a4, a5, a6, a7);
		//printf("%s", lineBuffer);

		rfcomm_request_can_send_now_event(rfcomm_channel_id);
	}

	btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
	btstack_run_loop_add_timer(ts);
}

static void one_shot_timer_setup(void) {
	// set one-shot timer
	heartbeat.process = &heartbeat_handler;
	btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
	btstack_run_loop_add_timer(&heartbeat);
}

/* LISTING_START(SppServerPacketHandler): SPP Server - Heartbeat Counter over RFCOMM */
static void packet_handler(uint8_t packet_type, uint16_t channel,
		uint8_t *packet, uint16_t size) {
	UNUSED(channel);

	/* LISTING_PAUSE */
	bd_addr_t event_addr;
	uint8_t rfcomm_channel_nr;
	uint16_t mtu;
	int i;

	switch (packet_type) {
	case HCI_EVENT_PACKET:
		switch (hci_event_packet_get_type(packet)) {
		/* LISTING_RESUME */
		case HCI_EVENT_PIN_CODE_REQUEST:
			// inform about pin code request
			printf("Pin code request - using '0000'\n");
			hci_event_pin_code_request_get_bd_addr(packet, event_addr);
			gap_pin_code_response(event_addr, "0000");
			break;

		case HCI_EVENT_USER_CONFIRMATION_REQUEST:
			// ssp: inform about user confirmation request
			printf(
					"SSP User Confirmation Request with numeric value '%06"PRIu32"'\n",
					little_endian_read_32(packet, 8));
			printf("SSP User Confirmation Auto accept\n");
			break;

		case RFCOMM_EVENT_INCOMING_CONNECTION:
			// data: event (8), len(8), address(48), channel (8), rfcomm_cid (16)
			rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
			rfcomm_channel_nr =
					rfcomm_event_incoming_connection_get_server_channel(packet);
			rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(
					packet);
			printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr,
					bd_addr_to_str(event_addr));
			rfcomm_accept_connection(rfcomm_channel_id);
			break;

		case RFCOMM_EVENT_CHANNEL_OPENED:
			// data: event(8), len(8), status (8), address (48), server channel(8), rfcomm_cid(16), max frame size(16)
			if (rfcomm_event_channel_opened_get_status(packet)) {
				printf("RFCOMM channel open failed, status %u\n",
						rfcomm_event_channel_opened_get_status(packet));
			} else {
				rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(
						packet);
				mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
				printf(
						"RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n",
						rfcomm_channel_id, mtu);
			}
			break;
		case RFCOMM_EVENT_CAN_SEND_NOW:
			rfcomm_send(rfcomm_channel_id, (uint8_t*) lineBuffer,
					strlen(lineBuffer));
			break;

			/* LISTING_PAUSE */
		case RFCOMM_EVENT_CHANNEL_CLOSED:
			printf("RFCOMM channel closed\n");
			rfcomm_channel_id = 0;
			break;

		default:
			break;
		}
		break;

	case RFCOMM_DATA_PACKET:
		printf("RCV: '");
		for (i = 0; i < size; i++) {
			putchar(packet[i]);
		}
		printf("'\n");
		break;

	default:
		break;
	}
	/* LISTING_RESUME */
}

int imu_read(int argc, const char * argv[]);
int imu_read(int argc, const char * argv[]) {
	i2c_master_init();
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	struct bno055_t myBNO = { .bus_write = BNO055_I2C_bus_write, .bus_read =
			BNO055_I2C_bus_read, .dev_addr = BNO055_I2C_ADDR1, .delay_msec =
			BNO055_delay_msek };
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	bno055_init(&myBNO);
	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	struct bno055_euler_float_t eulerData;
	bno055_convert_float_euler_hpr_deg(&eulerData);
	unsigned char accel_calib_status = 0;
	unsigned char gyro_calib_status = 0;
	unsigned char mag_calib_status = 0;
	unsigned char sys_calib_status = 0;
	bno055_get_accel_calib_stat(&accel_calib_status);
	bno055_get_mag_calib_stat(&mag_calib_status);
	bno055_get_gyro_calib_stat(&gyro_calib_status);
	bno055_get_sys_calib_stat(&sys_calib_status);
	while (1) {
		if (bno055_read_euler_h(&heading) == BNO055_SUCCESS
				&& bno055_read_euler_p(&pitch) == BNO055_SUCCESS
				&& bno055_read_euler_r(&roll) == BNO055_SUCCESS) {
			rollDeg = roll / 16;
			pitchDeg = pitch / 16;
			yawDeg = heading / 16;
			//printf("\nhead: %.2f pitch: %.2f roll: %.2f", yawDeg, pitchDeg,
					//rollDeg);
//			yaw = (int) yawDeg;
//			pit = (int) pitchDeg;
//			rol = (int) rollDeg;
		} else {
			printf("\nFalho");
		}
		vTaskDelay(50 / 100);
	}
	return 0;
}

int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]) {
	(void) argc;
	(void) argv;

	one_shot_timer_setup();
	spp_service_setup();

	gap_discoverable_control(1);
	gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
	gap_set_local_name("HandsOnVR");

	// turn on!
	hci_power_control(HCI_POWER_ON);

	return 0;
}

