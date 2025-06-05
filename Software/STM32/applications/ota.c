/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-30     armink       the first version
 * 2018-08-27     Murphy       update log
 */

#include <rtthread.h>
#include <stdio.h>
#include <stdbool.h>
#include <finsh.h>
#include <fal.h>
#include <ymodem.h>
#include <rtdevice.h>
#include "./WS2812/ws2812b.h"
#include "main.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME               "ymodem"
#ifdef YMODEM_DEBUG
#define DBG_LEVEL                      DBG_LOG
#else
#define DBG_LEVEL                      DBG_INFO
#endif
#define DBG_COLOR
#include <rtdbg.h>

static size_t update_file_total_size, update_file_cur_size;

static const struct fal_partition * dl_part = RT_NULL;

static void set_ota_flag() {
    // RTC 初始化
    hrtc.Instance = RTC;
    HAL_RTC_Init(&hrtc);

    // 写入 OTA 状态标志
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0xA5A5);
//    rt_kprintf("BKP->0xA5A5\n");
}

static enum rym_code ymodem_on_begin(struct rym_ctx *ctx, rt_uint8_t *buf, rt_size_t len)
{
    char *file_name, *file_size;

    /* calculate and store file size */
    file_name = (char *)&buf[0];
    file_size = (char *)&buf[rt_strlen(file_name) + 1];
    update_file_total_size = atol(file_size);

    update_file_cur_size = 0;

    /* Get download partition information and erase download partition data */
    if ((dl_part = fal_partition_find("download")) == RT_NULL)
    {
        LOG_E("Firmware download failed! Partition (%s) find error!", "download");
        return RYM_CODE_CAN;
    }

    if (update_file_total_size > dl_part->len)
    {
        LOG_E("Firmware is too large! File size (%d), '%s' partition size (%d)", update_file_total_size, "download", dl_part->len);
        return RYM_CODE_CAN;
    }

    LOG_I("Start erase. Size (%d)", update_file_total_size);

    /* erase DL section */
    if (fal_partition_erase(dl_part, 0, update_file_total_size) < 0)
    {
        LOG_E("Firmware download failed! Partition (%s) erase error!", dl_part->name);
        return RYM_CODE_CAN;
    }

    return RYM_CODE_ACK;
}

static enum rym_code ymodem_on_data(struct rym_ctx *ctx, rt_uint8_t *buf, rt_size_t len)
{
    /* write data of application to DL partition  */
    if (fal_partition_write(dl_part, update_file_cur_size, buf, len) < 0)
    {
        LOG_E("Firmware download failed! Partition (%s) write data error!", dl_part->name);
        return RYM_CODE_CAN;
    }

    update_file_cur_size += len;

    return RYM_CODE_ACK;
}

void update_driver()
{
    struct rym_ctx rctx;

    rt_kprintf("Waring: Ymodem has started! This operator will not recovery.\n");
    rt_kprintf("Please select the ota firmware file and use Ymodem to send 22.\n");

    static rt_device_t serial;
    serial = rt_device_find("uart3");
    if (!serial)
    {
        rt_kprintf("find uart3 failed");
        return;
    }

    // XXX: reduce bitrate?
    //struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    //config.baud_rate = BAUD_RATE_9600;
    //rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    if (!rym_recv_on_device(&rctx, serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                            ymodem_on_begin, ymodem_on_data, NULL, RT_TICK_PER_SECOND))
    {
        rt_kprintf("Download firmware to flash success.\n");
        rt_kprintf("System now will restart...\r\n");
        set_ota_flag();
        ws2812_clearn_all(2);
        /* wait some time for terminal response finish */
        rt_thread_delay(rt_tick_from_millisecond(200));
        /* Reset the device, Start new firmware */
        extern void rt_hw_cpu_reset(void);
        rt_hw_cpu_reset();
        /* wait some time for terminal response finish */
        rt_thread_delay(rt_tick_from_millisecond(200));
    }
    else
    {
        /* wait some time for terminal response finish */
        rt_thread_delay(RT_TICK_PER_SECOND);
        rt_kprintf("Update firmware fail.\n");
        set_ota_flag();
        ws2812_clearn_all(2);

        extern void rt_hw_cpu_reset(void);
        rt_hw_cpu_reset();
        //config.baud_rate = BAUD_RATE_115200;
        //rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    }

    return;
}
