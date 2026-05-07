#ifndef MAVLINK_RX_H
#define MAVLINK_RX_H

#include <stdint.h>

typedef struct{
	int16_t flow_x;
	int16_t flow_y;
	float altitude_m;
	uint8_t quality;
	uint32_t last_update_ms;
} OpticalFlow_Data_t;

extern OpticalFlow_Data_t optical_flow_data;

void MAVLink_Init(void);
void MAVLink_Task_Process(void);

#endif
