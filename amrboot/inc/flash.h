/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef FLASH_H_
#define FLASH_H_

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/ioctl.h>

/* size(24*16kb) */
#define NUM_USER_SECTORS (24)

#define COMMAND_TIMEOUT ((uint16_t)3000u)


/* ASC_2 code */
#define UL_a   ((uint8_t)0x61u)
#define UL_1   ((uint8_t)0x31u)
#define UL_2   ((uint8_t)0x32u)
#define UL_3   ((uint8_t)0x33u)
#define UL_4   ((uint8_t)0x34u)


/* bootlaoder size is 128kb, user 384kb */
#define FLASH_APP_START_ADDRESS	(0x08100000)  /* 1MB */
#define BOOT_SECTOR_SIZE		(0x4000)
#define NUM_USER_SECTORS		(24)
#define FLASH_APP_END_ADDRESS	((uint32_t)FLASH_APP_START_ADDRESS+ \
								((uint32_t)BOOT_SECTOR_SIZE * (uint32_t)NUM_USER_SECTORS))

#define FLASH_DEV  "/dev/mtd0"
enum boot_flash_status{
  FLASH_OK              = 0x00u, /* The action was successful. */
  FLASH_ERROR_SIZE      = 0x01u, /* The binary is too big. */
  FLASH_ERROR_WRITE     = 0x02u, /* Writing failed. */
  FLASH_ERROR_READBACK  = 0x04u, /* Writing was successful, but the content of the memory is wrong. */
  FLASH_ERROR           = 0xFFu  /* Generic error. */
};

struct boot_flash_s 
{
	int fd;
	bool flash_inited;
	uint32_t write_size;
	uint8_t  sector;   /* present sector, */
	uint8_t buffer[BOOT_SECTOR_SIZE];
	uint32_t buff_index;	/* buffer present pointer */
	uint32_t buff_available;  /* buffer available size */
	
	struct mtd_geometry_s geo;

};

int boot_flash_init(void);
enum boot_flash_status boot_flash_erase(void);
enum boot_flash_status boot_flash_write(void *data, uint32_t length,bool last);
void boot_flash_jump_to_app(void);

void boot_flash_deinit(void);

#endif /* FLASH_H_ */
