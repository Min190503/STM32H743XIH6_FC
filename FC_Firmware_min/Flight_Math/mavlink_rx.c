#include "mavlink_rx.h"
#include "usart.h"
#include "common/mavlink.h"

#define RX_BUF_SIZE 256
__attribute__((aligned(32))) static uint8_t rx_dma_buf[RX_BUF_SIZE];
static uint16_t old_pos = 0;


OpticalFlow_Data_t optical_flow_data;

void MAVLink_Init(void){
	HAL_UART_Receive_DMA(&huart2, rx_dma_buf, RX_BUF_SIZE);
}


void MAVLink_Task_Process(void){
	SCB_InvalidateDCache_by_Addr((uint32_t*)rx_dma_buf, RX_BUF_SIZE);
	  // Tính toán vị trí con trỏ ghi hiện tại của phần cứng DMA
	  uint16_t curr_pos = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

		if (curr_pos >= RX_BUF_SIZE) curr_pos = 0;


	  // Nếu con trỏ phần cứng chạy vượt con trỏ của phần mềm -> Có data mới
	  while(old_pos != curr_pos){
		  uint8_t c = rx_dma_buf[old_pos];
		  old_pos++;
		  if(old_pos >= RX_BUF_SIZE) old_pos = 0;

		  mavlink_message_t msg;
		  mavlink_status_t status;


		  if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			  // Nhận thành công trọn vẹn 1 gói tin! Kiểm tra xem đúng loại không
			  if (msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW) {
				  mavlink_optical_flow_t flow_msg;
				  mavlink_msg_optical_flow_decode(&msg, &flow_msg);

				  // Rã đông thành công, đẩy ra biến toàn cục
				  optical_flow_data.flow_x = flow_msg.flow_x;
				  optical_flow_data.flow_y = flow_msg.flow_y;
				  optical_flow_data.altitude_m = flow_msg.ground_distance;
				  optical_flow_data.quality = flow_msg.quality;
				  optical_flow_data.last_update_ms = HAL_GetTick();
			  }
		  }
	  }
}
