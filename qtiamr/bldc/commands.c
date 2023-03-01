/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "commands.h"


/*****************************************************************************
* Define motor control frame 
* Read data frame: one byte device address, one byte control command,two bytes read register address,
* two bytes read registered number and two bytes crc check.
* Data return frame:  one byte device address, one byte control command, one byte used save the number of
* register bytes, two bytes crc check.
* Write data frame: two bytes used to express device address control command register address,
* two bytes used to represent data，two bytes for crc check.
* The speed value is a signed 16-bit value, with a range of plus or minus 3000.
*******************************************************************************/

const struct motor_controller_cmd_s mc_command_list[] = {
	{{"debug"}, {0x01, 0x03, 0x20ab, 0x02, 0xbe2b}},
	{{"speed_mode"}, {CONTROLLER_ADDR, W_SINGLE_REG, 0x200d, 0x03, 0x53c8}},
	{{"read_mode"}, {CONTROLLER_ADDR, R_MULTI_REG, 0x200d, 0x01, 0x00}},
	{{"mc_en"}, {CONTROLLER_ADDR, W_SINGLE_REG, 0x200E, 0x08, 0x00}},
	{{"mc_disable"}, {CONTROLLER_ADDR, W_SINGLE_REG, 0x200E, 0x07, 0x00}},
	{{"read_speed"}, {CONTROLLER_ADDR, R_MULTI_REG, 0x20ab, 0x02, 0xbe2b}},
	{{"set_speed_100r"}, {CONTROLLER_ADDR, W_SINGLE_REG, 0x2088, 0x64, 0x03cb}},
};



/*****************************************************************************
* Define motor driver control code.
*******************************************************************************/

enum mc_control_code_e {
	DEBUG_CC = 0,
	SET_SPEED_MODE,
	GET_MODE,
	SET_MC_ENABLE,
	SET_MC_DISABLE,
	GET_SPEED,
	SET_SPEED_100,
};

/*****************************************************************************
* Function: CRC_16 check 
* Description: Calculate CRC16.
* Input: data addres, data length. 
* Return: CRC value
******************************************************************************/
static uint16_t crc16_485bus(const uint8_t *data, uint8_t data_len){
	
	uint16_t ucrc = 0xffff;//CRC register 
	uint8_t num =0;


	for(uint8_t num=0; num<data_len; num++){
		ucrc = (*data++)^ucrc;
		for(uint8_t x=0;x<8;x++){
			if(ucrc&0x0001){
				ucrc = ucrc>>1;
				ucrc = ucrc^0xA001;
			}else{
				ucrc = ucrc>>1;
			}
		}
	}
	return ucrc;
}


/*****************************************************************************
* Function: controller frame format copy
* Description:  Motor driver data frame coding.
* Input: @mc_buff, motor control command . 
* Output: @buff,encoded data 
* Return: null
******************************************************************************/
static void controller_frame_coding(char* buff, uint8_t buff_len, struct mc_cmd_buffer_s * mc_buff) {
	
	int16_t data;
	uint8_t current = 0;
	uint8_t i;
	if (buff ==NULL || mc_buff == NULL)
		return;
	
	memcpy(buff, mc_buff, 2); //addr &id
	current = 2;

	data = ((mc_buff->mc_register & 0xff) << 0x8) + ((mc_buff->mc_register & 0xff00) >> 0x8);
	memcpy(&buff[current], &data, 2);

	current = current + 2;
	data = ((mc_buff->mc_reg_len & 0xff) << 0x8) + ((mc_buff->mc_reg_len & 0xff00) >> 0x8);
	memcpy(&buff[current], &data, 2);
}


static char *controller_frame_decoding(void *mc_buff, char* rc_buff, uint8_t rc_size) {
	
	return 0;
}

/*****************************************************************************
* Function: Robot motor controller frame read/write
* Description:  Send motor controller frame to motor driver and receive it.
* Input: @frame,send frame. 
* Output: @frame_return, recevice frame from motor driver.  
* Return: EEROR NUMBER
******************************************************************************/
static int mc_frame_sync(void *frame, uint8_t frame_size, char *frame_return,uint8_t return_size, int fd)
{
	uint8_t send_buff[RS485_MSG_LEN];
	uint8_t read_size,count, i;
	uint16_t crc16;
	char * receive_buff;
	uint32_t current,time;

	receive_buff = frame_return;
	controller_frame_coding(send_buff, frame_size, frame);
	current = frame_size;

	/* calculate crc */
	crc16 = crc16_485bus(send_buff, frame_size);

	memcpy(&send_buff[current], &crc16, 2);
	current = current + 2;
	/* send fame to MC */
	write(fd, send_buff, current);
	printf("DEBUG send mc_fame  to controller \n\n");

	/* Data frame exception not considered.  */
	current= 0;
	count =0;
	while (count < return_size && current< RS485_RECEIVE_TIME_MS)
	{
		read_size = read(fd, &receive_buff[count], return_size-count);
		if (read_size > 0)
		{
			count = read_size +count;
		}
		usleep(RS485_RECEIVE_TIME_MS/4);
		current = current + RS485_RECEIVE_TIME_MS/4;
	}

	printf("read count =%d \n",count);
	
	if (count == return_size)
	{
		printf("get data %x %x %x %x %x %x \n",receive_buff[0],
												receive_buff[1],
												receive_buff[2],
												receive_buff[3],
												receive_buff[4],
												receive_buff[5]);
	}
	return OK;

}



/*****************************************************************************
* Function: Robot motor controller init
* Description:  Init motor controller struct.
* Input: @fd,motor driver file description. 
* Output: 
* Return: EEROR NUMBER
******************************************************************************/
int mc_init(int fd)
{
	uint8_t rec_buff[RS485_MSG_LEN];
	char rec_size;
	struct mc_cmd_buffer_s mc_buff;

	printf(" mc_init enable controller \n\n");
	/* enable controller */
	mc_buff.mc_addr = CONTROLLER_ADDR;
	mc_buff.mc_cmd_id = W_SINGLE_REG;
	mc_buff.mc_register =MCREG_CONTROL_MODE;
	mc_buff.reg_data = CONTROL_MODE_EN;

	rec_size = RETURN_RAME_WRITE_SIZE;
	mc_frame_sync(&mc_buff, 6, rec_buff, rec_size, fd);

	printf(" mc_init set speed mode \n\n");

	mc_buff.mc_register =MCREG_RUNNING_MODE;
	mc_buff.reg_data = RUNNING_MODE_SPEED;

	rec_size = RETURN_RAME_WRITE_SIZE;
	mc_frame_sync(&mc_buff, 6, rec_buff, 8, fd);

	/* set speed mode */

	return OK;
}


/*****************************************************************************
* Function: Set motor speed.
* Description: Write robot left wheel speed and robot right speed.
* Input: @speed_left, left motor speed; @speed_right,right motor speed;
* @fd, motor driver file description.
* Output: NULL
* Return: EEROR NUMBER
******************************************************************************/
int mc_set_speed(int16_t speed_left, int16_t speed_right, int fd)
{
	uint8_t rec_buff[RS485_MSG_LEN];
	char rec_size;
	struct mc_cmd_buffer_s mc_buff;

	printf(" mc_set_speed : left %d right %d \n\n",speed_left, speed_right);
	/* enable controller */
	mc_buff.mc_addr = CONTROLLER_ADDR;
	mc_buff.mc_cmd_id = W_SINGLE_REG;
	mc_buff.mc_register =MCREG_LEFT_SET_SPEED;
	mc_buff.reg_data = speed_left;

	rec_size = RETURN_RAME_WRITE_SIZE;
	mc_frame_sync(&mc_buff, 6, rec_buff, rec_size, fd);

	mc_buff.mc_register =MCREG_RIGHT_SET_SPEED;
	mc_buff.reg_data = speed_right;

	rec_size = RETURN_RAME_WRITE_SIZE;
	mc_frame_sync(&mc_buff, 6, rec_buff, rec_size, fd);

	return OK;
	
}


uint16_t mc_get_speed(int fd)
{
	
	return OK;
}


/*****************************************************************************
* Function: disable motor controller.
* Description: Send motor driver disable function.
* Input: @fd, motor driver file description.
* Output: NULL
* Return: NULL
******************************************************************************/
void mc_disable(int fd)
{
	uint8_t buff[RS485_MSG_LEN];
	uint8_t current = 0;
	uint16_t crc16;
	int16_t read_data;
	struct mc_cmd_buffer_s *mc_cmd_buff;

	printf(" mc_set_speed_100 start \n\n");
	
	/* set driver disable frame */
	mc_cmd_buff = &mc_command_list[SET_MC_DISABLE].command;
	controller_frame_copy(buff, 6, mc_cmd_buff);
	current = 6;
	
	/* calculate crc */
	crc16 = crc16_485bus(buff, 6);
	sleep(2);

	memcpy(&buff[current], &crc16, 2);
	write(fd, buff, 8);
	printf("send disable  to controller \n\n");
	sleep(2);
	
	read(fd, buff, 8);
	if (buff[0] == mc_cmd_buff->mc_addr)
	{
		read_data = buff[2]<<8 + buff[3];
		printf("disable mc ok  send_d=%x read_d=%x \n\n",mc_cmd_buff->mc_register,read_data);
	}
	else{
		printf("error: read frame header %x %x ",buff[0] ,buff[1]);
		return;
	}
}
