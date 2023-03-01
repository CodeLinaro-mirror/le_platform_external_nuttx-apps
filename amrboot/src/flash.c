/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>


#include "flash.h"


FAR static struct boot_flash_s g_flash; 


/* Erase function not ready in low driver */
enum boot_flash_status boot_flash_erase(void)
{
  return FLASH_OK;
}

static  void char2word(uint16_t * buffer, char * data, size_t bytes)
{
	uint32_t num_bytes = bytes;
	uint32_t i,k;
	
	if ( buffer ==NULL || data == NULL)
		return;
	
	k =0;
	for (i = 0; i < num_bytes / sizeof(uint16_t);i++)
	{
		buffer[i] = data[k] + (data[k+1]<< 8u);
		k = k + sizeof(uint16_t);
	}

	printf("debug half word =  %x \t %x \n",buffer[0],buffer[1]);
}

/* flash data to mcu flash */
enum boot_flash_status boot_flash_write(void *data, uint32_t length,bool last)
{
	enum boot_flash_status ret = FLASH_OK;
	ssize_t nbytes;
	uint32_t data_index;
	FAR uint32_t *buffer;
	  

	int fd;

	if ((data == NULL) || (length > BOOT_SECTOR_SIZE) || (g_flash.flash_inited != true))
		return FLASH_ERROR_SIZE;

	buffer = (FAR uint32_t *)malloc(g_flash.geo.blocksize);

	if (buffer == NULL){
		printf("ERROR: buffer malloc failed \n" );
		return FLASH_ERROR_WRITE;
	}

	/* add  data in  buffer */
	if (g_flash.buff_available <= length)
	{
		memcpy(&g_flash.buffer[g_flash.buff_index], data, g_flash.buff_available);
		
		//char2word(buffer, g_flash.buffer, BOOT_SECTOR_SIZE);
		//memcpy(buffer,g_flash.buffer,BOOT_SECTOR_SIZE);

		nbytes = write(g_flash.fd, g_flash.buffer, BOOT_SECTOR_SIZE);
		if (nbytes < 0)
		{
			printf("ERROR: boot_flash_write failed: %d  return =%d \n", errno,nbytes );
			fflush(stdout);
			free(buffer);
			return FLASH_ERROR_WRITE;
		}

		g_flash.sector = g_flash.sector +1;
		memset(g_flash.buffer, 0, BOOT_SECTOR_SIZE);
		memcpy(g_flash.buffer, data+g_flash.buff_available, length - g_flash.buff_available);
		g_flash.buff_index = length - g_flash.buff_available;
		g_flash.buff_available = BOOT_SECTOR_SIZE - g_flash.buff_index;
	
		printf("DEBUG: write flash done sector = %d nbytes=%d \n",g_flash.sector,nbytes);
		free(buffer);
		return ret;

	} else {
		memcpy(&g_flash.buffer[g_flash.buff_index], data, length);
		g_flash.buff_index = g_flash.buff_index + length;
		g_flash.buff_available = BOOT_SECTOR_SIZE - g_flash.buff_index;
	}

	if (true == last)
	{
		//char2word(buffer, g_flash.buffer, BOOT_SECTOR_SIZE);
		nbytes = write(g_flash.fd, g_flash.buffer, BOOT_SECTOR_SIZE);
		if (nbytes < 0)
		{
			printf("ERROR: boot_flash_write failed: %d\n", errno);
			fflush(stdout);
			free(buffer);
			return FLASH_ERROR_WRITE;
		}
		g_flash.buff_index = 0;
		g_flash.buff_available = BOOT_SECTOR_SIZE;
		g_flash.sector = g_flash.sector +1;
	}

	printf("debug: boot_flash_write flash buffer index= %d \n",g_flash.buff_index);
	free(buffer);
	return ret;
}

static void  do_jump(uint32_t stacktop, uint32_t entrypoint)
{
        asm volatile(
                "msr msp, %0  \n"
                "bx %1  \n"
                : : "r"(stacktop), "r"(entrypoint) :);

        // just to keep noreturn happy
        for (;;) ;
}

void boot_flash_jump_to_app(void)
{
	const uint32_t *app_base = (const uint32_t *)FLASH_APP_START_ADDRESS;
	uint8_t data = 0x0;
	
	/* flash the last buffer data */
	if (g_flash.buff_index != 0)
		boot_flash_write(&data, 1,true);

	/*
	* We refuse to program the first word of the app until the upload is marked
	* complete by the host.  So if it's not 0xffffffff, we should try booting it.
	*/
	printf("jump_to_app:  =  %x ; %p \n",*app_base,app_base);
	printf("jump_to_app-2:  =  %x\n",*(app_base+4));
	
	if (app_base[0] == 0x0) {
		printf("error:jump_to_app failed app_base_0\n");
		return;
	}

	/*
	* The second word of the app is the entrypoint; it must point within the
	* flash area (or we have a bad flash).
	*/
	if (app_base[1] < FLASH_APP_START_ADDRESS) {
		printf("error:jump_to_app failed app_base_1\n");
		return;
	}

	/* extract the stack and entrypoint from the app vector table and go */
	do_jump(app_base[0], app_base[1]);
	return;
}


int boot_flash_init(void)
{
	int ret;
	int fd;
	struct mtd_geometry_s *geo;

	fd = open(FLASH_DEV,  O_WRONLY);

	if (fd < 0)
    {
        printf(" open flash device fd=%d    failed: %d\n",fd, errno);
		return ERROR;
    }
	printf(" opened  flash device  OK \n");
	g_flash.fd = fd;
	
	/* Get the geometry of the FLASH device */
	geo = &g_flash.geo;
	ret = ioctl(fd, MTDIOC_GEOMETRY,(unsigned long)((uintptr_t)geo));

	if (ret < 0)
    {
        printf("MDT: ioctl(MTDIOC_GEOMETRY) failed: %d\n", errno);
        return ERROR;
    }
	
	printf("  blocksize:      %lu\n", (unsigned long)g_flash.geo.blocksize);
	printf("  erasesize:      %lu\n", (unsigned long)g_flash.geo.erasesize);
	printf("  neraseblocks:   %lu\n", (unsigned long)g_flash.geo.neraseblocks);


	if ((geo->blocksize != BOOT_SECTOR_SIZE) && (geo->neraseblocks != NUM_USER_SECTORS))
	{
		printf("MDT: error sector configuration %d ,%d \n", geo->blocksize, geo->neraseblocks);
        return ERROR;
	}
	
	/* init parameters */
	g_flash.write_size = 0;
	g_flash.sector = 0;
	g_flash.buff_index = 0;
	g_flash.buff_available = BOOT_SECTOR_SIZE;	
	g_flash.flash_inited = true;


	return OK;
}

void boot_flash_deinit(void)
{
	close(g_flash.fd);
}


