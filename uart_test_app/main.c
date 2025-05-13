/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <syslog.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define CIRC_BUF_LEN 256
#define SERIAL_FD ("/dev/ttyS2")
//#define DEBUG_PRINT
struct serial_s
{
  int fd;
};



/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct serial_s g_serial;

/*cirbuffer*/
static uint8_t cir_buffer_g[CIRC_BUF_LEN];
static int r_point_g, w_point_g;
static bool buffer_is_full_g;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*cic buffer*/

static int get_empty_buffer_len(void)
{
  if(buffer_is_full_g)
  {
    return 0;
  }
  else if (r_point_g == w_point_g)
  {
    return CIRC_BUF_LEN;
  }
  else if (r_point_g > w_point_g)
  {
    return r_point_g - w_point_g;
  }
  else
  {
	return (r_point_g + CIRC_BUF_LEN) - w_point_g;
  }
}


static int get_data_buffer_len(void)
{
  if(buffer_is_full_g)
  {
    return CIRC_BUF_LEN;
  }
  else if (r_point_g == w_point_g)
  {
    return 0;
  }
  else if (w_point_g > r_point_g)
  {
    return w_point_g - r_point_g;
  }
  else
  {
    return (w_point_g + CIRC_BUF_LEN) - r_point_g;
  }
}

static int save_data_into_cirbuf(uint8_t *data, int count)
{
  int i;

  if (get_empty_buffer_len() < count)
  {
    return -1;
  }

  if (((count + w_point_g) % CIRC_BUF_LEN) == r_point_g)
  {
    buffer_is_full_g = true;
  }

  for (i = 0; i < count; i++)
  {
    cir_buffer_g[w_point_g] = data[i];
    w_point_g = (w_point_g + 1) % CIRC_BUF_LEN;
  }

  return 0;
}

static int put_cirbuf_into_buff(uint8_t *data, int count)
{
  int read_cur = r_point_g;
  int i;

  for (i=0; i < count; i++)
  {
    data[i] = cir_buffer_g[read_cur];
    read_cur = (read_cur + 1) % CIRC_BUF_LEN;
  }
  return 0;
}


static bool read_buffe_len(int count)
{
  r_point_g = (r_point_g + count)%CIRC_BUF_LEN;
  if (r_point_g != w_point_g)
  {
    buffer_is_full_g=false;
  }
  return true;
}



static void read_buffer(void)
{
  int readable_len = 0;
  int free_buf = 0;

  if(ioctl(g_serial.fd, FIONREAD, &readable_len) < 0)
  {
    printf("\nERROR: serial get readable size fail!\n");
    return;
   }
  if (readable_len > 0)
  {
    uint8_t *buf = malloc(readable_len * sizeof(uint8_t));
    int read_len = read(g_serial.fd, buf, readable_len);
#ifdef DEBUG_PRINT
    if (readable_len)
    {
      printf("\nread buffer len: %d %d,!\n",readable_len, read_len);
      printf("read buffer: %s .\n", buf);
    }
#endif
    while(read_len > 0) 
    {
      free_buf = get_empty_buffer_len();
      if (read_len <= free_buf)
      {
        save_data_into_cirbuf((uint8_t*)buf, read_len);
	read_len = 0;
      }
      else
      {
        save_data_into_cirbuf((uint8_t*)buf, free_buf);
        printf("\nERROR: circ buffer is full, lose serial data!\n");
      }
    }
    free(buf);
  }
  return;
}

static void write_buffer(void)
{
  int writeable_len = 0;
  int write_able = 0;
  int write_cnt = 0;

  if(ioctl(g_serial.fd, FIONSPACE, &writeable_len) < 0)
  {
    printf("\nERROR: serial get writeable size fail!\n");
    return;
  }
  if (writeable_len > 0)
  {
    write_able = get_data_buffer_len();
#ifdef DEBUG_PRINT
    if (write_able)
    {
      printf("\nneed_write buffer len: %d,free buff:%d,!\n",write_able,writeable_len);
    }
#endif
    if (write_able == 0)
    {
      return;
    }
    if (writeable_len > write_able)
    {
      uint8_t *buf = malloc(write_able * sizeof(uint8_t));
      put_cirbuf_into_buff(buf, write_able);
      write_cnt = write(g_serial.fd, buf, write_able);
#ifdef DEBUG_PRINT
      printf("\nwrite_cnt1 : %d,!\n",write_cnt);
      printf("write buffer1: %s .\n", buf);
#endif
      free(buf);
      read_buffe_len(write_cnt);
    }
    else
    {
      uint8_t *buf = malloc(write_able * sizeof(uint8_t));
      put_cirbuf_into_buff(buf, writeable_len);
      write_cnt = write(g_serial.fd, buf, writeable_len);
#ifdef DEBUG_PRINT
      printf("\nwrite_cnt2 : %d,!\n",write_cnt);
      printf("write buffer2: %s .\n", buf);
#endif
      free(buf);
      read_buffe_len(write_cnt);
    }
  }
  return;
}


static void loop_writeback(void)
{
  int count = 0;

  while (1)
  {
    if (!(count % 10000))
    {
      count = 1;
      printf("##############loop###############\n\n");
    }
    read_buffer();
    write_buffer();
    usleep(100);
  }
  return;
}

static int serial_init(void)
{
  g_serial.fd = open(SERIAL_FD, O_RDWR);
  if(g_serial.fd == -1)
  {
    printf("ERROR: %s open failed!\n", SERIAL_FD);
    close(g_serial.fd);
    return false;
  }
  return true;
}

int main(int argc, FAR char *argv[])
{

  /* main function */
  printf("\n\n\n#############################\n");
  printf("#start uart test #\n");
  printf("#############################\n\n");

  syslog(LOG_INFO, "main: uart  main start\n");

  /* init serial */
  if(serial_init())
  {
    loop_writeback();
  }


  return EXIT_SUCCESS;
}
