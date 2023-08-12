/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include "xmodem.h"

static struct boot_xmodem_s  g_boot_xmodem;

static uint16_t xmodem_calc_crc(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0u;
    while (length)
    {
        length--;
        crc = crc ^ ((uint16_t)*data++ << 8u);
        for (uint8_t i = 0u; i < 8u; i++)
        {
            if (crc & 0x8000u)
            {
                crc = (crc << 1u) ^ 0x1021u;
            }
            else
            {
                crc = crc << 1u;
            }
        }
    }
    return crc;
}

static uint8_t xmodem_handle_packet(uint8_t header)
{
    uint8_t status = XMODEM_OK;
    uint16_t size = 0u;
    ssize_t recevic_pn;
    ssize_t receive_data;
    ssize_t receive_crc;
    uint16_t crc_received;
    uint16_t crc_calculated;
    enum boot_flash_status flash_status;
    uint8_t data_len =0;
    uint8_t retry = 0;
    uint8_t package_len = 0;

    g_boot_xmodem.received_packet_number[X_PACKET_NUMBER_SIZE];
    g_boot_xmodem.received_packet_data[X_PACKET_1024_SIZE];
    g_boot_xmodem.received_packet_crc[X_PACKET_CRC_SIZE];

    /* Get the size of the data. */
    if (X_SOH == header)
    {
        size = X_PACKET_128_SIZE;
    }
    else if (X_STX == header)
    {
        size = X_PACKET_1024_SIZE;
    }
    else
    {
        /* Wrong header type. This shoudn't be possible... */
        status |= XMODEM_ERROR;
    }

    package_len =X_PACKET_NUMBER_SIZE + size + X_PACKET_CRC_SIZE;
    while (retry < 10)
    {
        ioctl(g_boot_xmodem.fd, FIONREAD, (unsigned long)&data_len);
        if (data_len >= package_len)
        {
            break;
        }

        usleep(8000);
        retry ++;
    }
    if (retry ==10)
    {
        status |= XMODEM_ERROR_UART;
        return status;
    }

    /* Get the packet number, data and CRC from UART. */
    recevic_pn = read(g_boot_xmodem.fd, &g_boot_xmodem.received_packet_number[0u], X_PACKET_NUMBER_SIZE);
    receive_data = read(g_boot_xmodem.fd, &g_boot_xmodem.received_packet_data[0u], size);
    receive_crc = read(g_boot_xmodem.fd, &g_boot_xmodem.received_packet_crc[0u], X_PACKET_CRC_SIZE);

    if (recevic_pn != X_PACKET_NUMBER_SIZE && receive_data !=size && receive_crc != X_PACKET_CRC_SIZE)
    {
        printf("error:xmodem_handle_packet read data failed \n");
        status |= XMODEM_ERROR_UART;
    }

    crc_received = ((uint16_t)g_boot_xmodem.received_packet_crc[X_PACKET_CRC_HIGH_INDEX] << 8u) | \
                   ((uint16_t)g_boot_xmodem.received_packet_crc[X_PACKET_CRC_LOW_INDEX]);

    crc_calculated = xmodem_calc_crc(&g_boot_xmodem.received_packet_data[0u], size);

    /* If it is the first packet, then erase the memory. */
    // TODO: Move this outside of the file transfer routine
    if ((XMODEM_OK == status) && (false == g_boot_xmodem.first_packet_received))
    {
        g_boot_xmodem.first_packet_received = true;
    }
    printf("debug xmodem_handle_packet g_packet_number= %d packet_number = %d \n",g_boot_xmodem.xmodem_packet_number,g_boot_xmodem.received_packet_number[0u]);

    /* Error handling and flashing. */
    if (XMODEM_OK == status)
    {
        if (g_boot_xmodem.xmodem_packet_number != g_boot_xmodem.received_packet_number[0u])
        {
            /* Packet number counter mismatch. */
            if (g_boot_xmodem.xmodem_packet_number > g_boot_xmodem.received_packet_number[0u])
            {
                //have been received this package
                status |= XMODEM_ERROR_NUM_MIN;
            }
            else
            {
                status |= XMODEM_ERROR_NUMBER;
            }
        }
        if (255u != (g_boot_xmodem.received_packet_number[X_PACKET_NUMBER_INDEX] + \
            g_boot_xmodem.received_packet_number[X_PACKET_NUMBER_COMPLEMENT_INDEX]))
        {
            /* The sum of the packet number and packet number complement aren't 255. */
            /* The sum always has to be 255. */
            status |= XMODEM_ERROR_NUMBER;
        }
        if (crc_calculated != crc_received)
        {
            /* The calculated and received CRC are different. */
            status |= XMODEM_ERROR_CRC;
        }
    }

    /* Do the actual flashing (if there weren't any errors). */
    if (XMODEM_OK == status) 
    {
        flash_status = boot_flash_write(&g_boot_xmodem.received_packet_data[0u], size, false);
        if (flash_status != FLASH_OK)
        {
            status |= XMODEM_ERROR_FLASH;
        }
    }

    /* Raise the packet number and the address counters (if there weren't any errors). */
    if (XMODEM_OK == status)
    {
        g_boot_xmodem.xmodem_packet_number++;
    }

    return status;
}

static uint8_t xmodem_error_handler(uint8_t *error_number, uint8_t max_error_number)
{
    uint8_t status = XMODEM_OK;
    uint8_t protocol;

    /* Raise the error counter. */
    (*error_number)++;
    /* If the counter reached the max value, then abort. */
    if ((*error_number) >= max_error_number)
    {
        /* Graceful abort. */
        protocol = X_CAN;
        write(g_boot_xmodem.fd, &protocol, 1);
        write(g_boot_xmodem.fd, &protocol, 1);
        status = XMODEM_ERROR;
    }
    /* Otherwise send a NAK for a repeat. */
    else
    {
        protocol = X_NAK;
        write(g_boot_xmodem.fd, &protocol, 1);
        status = XMODEM_OK;
    }

    return status;
}

static void xmodem_receive(void)
{
    volatile uint8_t status = XMODEM_OK;
    uint8_t packet_status;
    uint8_t error_number = 0u;
    uint8_t header = 0x00u;
    ssize_t recevic_n;
    uint8_t protocol;
    uint8_t data_len =0;
    uint8_t retry = 0;

    /* Loop until there isn't any error (or until we jump to the user application). */
    while (XMODEM_OK == status)
    {
        header = 0x00u;
        while (retry < 10)
        {
            ioctl(g_boot_xmodem.fd, FIONREAD, (unsigned long)&data_len);
            if (data_len >= 1)
            {
                break;
            }

            usleep(500);
            retry ++;
        }

    recevic_n = read(g_boot_xmodem.fd, &header, 1u);
    /* Spam the host (until we receive something) with ACSII "C", to notify it, we want to use CRC-16. */
    if ((recevic_n != 1) && (false == g_boot_xmodem.first_packet_received))
    {
        protocol = X_C;
        write(g_boot_xmodem.fd, &protocol, 1);
        printf("debug xmodem_receive write X_C \n");
        usleep(200);
        write(g_boot_xmodem.fd, &protocol, 1);
    }
    /* Uart timeout or any other errors. */
    else if ((recevic_n != 1) && (true == g_boot_xmodem.first_packet_received))
    {
        //status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
        //printf(" xmodem_receive xmodem_error_handler \n");
        continue;
    }
    else
    {
        /* Do nothing. */
    }

    /* The header can be: SOH, STX, EOT and CAN. */
    switch(header)
    {
        packet_status = XMODEM_ERROR;
        /* 128 or 1024 bytes of data. */
        case X_SOH:
        case X_STX:
            /* If the handling was successful, then send an ACK. */
            packet_status = xmodem_handle_packet(header);
            if (XMODEM_OK == packet_status)
            {
                protocol = X_ACK;
                write(g_boot_xmodem.fd, &protocol, 1);
                //printf("debug xmodem_receive write X_ACK  \n");
            }
            /* If the error was flash related, then immediately set the error counter to max (graceful abort). */
            else if (XMODEM_ERROR_FLASH == packet_status)
            {
                error_number = X_MAX_ERRORS;
                status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
                printf("debug xmodem_receive xmodem_error_handler ERROR_FLASH  \n");
            }
            else if (XMODEM_ERROR_NUM_MIN == packet_status)
            {
                //this package have been received.
                //protocol = X_ACK;
                //write(g_boot_xmodem.fd, &protocol, 1);
                printf("debug xmodem_receive XMODEM_ERROR_NUM_MIN \n");
            }
            else if (XMODEM_ERROR_UART == packet_status)
            {
                printf("debug xmodem_receive XMODEM_ERROR_UART \n");
            }
            else
            {
                status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
                printf(" xmodem_receive xmodem_error_handler status =%d \n",packet_status);
            }
            break;

            /* End of Transmission. */
            case X_EOT:
                /* ACK, feedback to user (as a text), then jump to user application. */
                printf("debug xmodem_receive data_len = %d \n",data_len);
                protocol = X_ACK;
                write(g_boot_xmodem.fd, &protocol, 1);
                printf("debug xmodem_receive X_EOT start  jump_to_app \n");
                boot_flash_jump_to_app();
                break;

            case X_CAN:
                status = XMODEM_ERROR;
                break;

            default:
                /* Wrong header. */
                if (recevic_n != 1)
                {
                    status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
                }
                break;
        }
    }
}

void bootloader_workflow(void)
{
    uint8_t resp = 0;
    uint8_t protocol;
    int recevic_n;
    uint8_t count = 15;
    uint8_t data_len =0;

    while(1)
    {
        usleep(500000);
        protocol = X_a;
        write(g_boot_xmodem.fd, &protocol, 1);
        usleep(500000);
        recevic_n = read(g_boot_xmodem.fd, &resp, 1);

        printf("bootloader_workflow receive cmd = %d \n",resp);

        if (1 == recevic_n)
        {
            switch(resp)
            {
                case FLASH:
                    xmodem_receive();
                    break;
                case APP:
                    boot_flash_jump_to_app();
                    break;
                case EXIT:
                    break;
                default:
                    break;
            }
        }
        else
        {
            count = count -1;
        }
        /* jump to app  */
        if (count < 1)
        {
            boot_flash_jump_to_app();
        }
    }
}

int xmodem_init(void)
{
    int fd;
    int ret;
    fd = open(XMODEM_DEV, O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        printf("error: xmodem_init: open %s failed: %d\n", XMODEM_DEV, errno);
        return ERROR;
    }

    printf(" opened %s\n",XMODEM_DEV);
    /* init parameters */
    g_boot_xmodem.fd = fd;
    g_boot_xmodem.first_packet_received = false;
    g_boot_xmodem.xmodem_packet_number = 1;

    /* init flash */
    ret = boot_flash_init();
    if (ret != OK)
    {
        printf("error: xmodem_init init boot failed \n");
    }

    return ret;
}

void xmodem_deinit(void)
{
    close(g_boot_xmodem.fd);
    boot_flash_deinit();
}
