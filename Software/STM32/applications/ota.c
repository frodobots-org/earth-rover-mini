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
 
 /* Debug configuration for YMODEM OTA update */
 #define DBG_ENABLE
 #define DBG_SECTION_NAME               "ymodem"
 #ifdef YMODEM_DEBUG
 #define DBG_LEVEL                      DBG_LOG
 #else
 #define DBG_LEVEL                      DBG_INFO
 #endif
 #define DBG_COLOR
 #include <rtdbg.h>
 
 /* Track OTA firmware update size */
 static size_t update_file_total_size, update_file_cur_size;
 
 /* Pointer to FAL (Flash Abstraction Layer) partition where firmware is downloaded */
 static const struct fal_partition * dl_part = RT_NULL;
 
 /* 
  * Set OTA (Over-The-Air update) flag in RTC backup register.
  * This ensures that after reset, the bootloader knows a new firmware was downloaded.
  */
 static void set_ota_flag() {
     // Initialize RTC (Real-Time Clock)
     hrtc.Instance = RTC;
     HAL_RTC_Init(&hrtc);
 
     // Write OTA status flag into backup register
     HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0xA5A5);
 //    rt_kprintf("BKP->0xA5A5\n");  // Debug print (disabled)
 }
 
 /* 
  * YMODEM callback: called at the beginning of the transfer.
  * Parses the file header, determines size, and prepares flash partition for writing.
  */
 static enum rym_code ymodem_on_begin(struct rym_ctx *ctx, rt_uint8_t *buf, rt_size_t len)
 {
     char *file_name, *file_size;
 
     /* Extract file name and size from YMODEM header packet */
     file_name = (char *)&buf[0];
     file_size = (char *)&buf[rt_strlen(file_name) + 1];
     update_file_total_size = atol(file_size);
 
     update_file_cur_size = 0;
 
     /* Locate the "download" partition in flash */
     if ((dl_part = fal_partition_find("download")) == RT_NULL)
     {
         LOG_E("Firmware download failed! Partition (%s) find error!", "download");
         return RYM_CODE_CAN; // Cancel transfer
     }
 
     /* Validate firmware size against partition size */
     if (update_file_total_size > dl_part->len)
     {
         LOG_E("Firmware is too large! File size (%d), '%s' partition size (%d)", 
               update_file_total_size, "download", dl_part->len);
         return RYM_CODE_CAN;
     }
 
     LOG_I("Start erase. Size (%d)", update_file_total_size);
 
     /* Erase partition area before writing new firmware */
     if (fal_partition_erase(dl_part, 0, update_file_total_size) < 0)
     {
         LOG_E("Firmware download failed! Partition (%s) erase error!", dl_part->name);
         return RYM_CODE_CAN;
     }
 
     return RYM_CODE_ACK; // Acknowledge ready to receive data
 }
 
 /* 
  * YMODEM callback: called for each data packet.
  * Writes received firmware data into flash partition.
  */
 static enum rym_code ymodem_on_data(struct rym_ctx *ctx, rt_uint8_t *buf, rt_size_t len)
 {
     /* Write firmware chunk to flash */
     if (fal_partition_write(dl_part, update_file_cur_size, buf, len) < 0)
     {
         LOG_E("Firmware download failed! Partition (%s) write data error!", dl_part->name);
         return RYM_CODE_CAN;
     }
 
     /* Update total written size */
     update_file_cur_size += len;
 
     return RYM_CODE_ACK;
 }
 
 /* 
  * Firmware update driver function:
  * - Starts YMODEM file transfer over UART3
  * - Writes received firmware into "download" flash partition
  * - Sets OTA flag in RTC
  * - Resets system to boot into new firmware
  */
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
 
     // NOTE: Optionally adjust baud rate if needed (commented out)
     //struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
     //config.baud_rate = BAUD_RATE_9600;
     //rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
 
     /* Start receiving firmware using YMODEM protocol */
     if (!rym_recv_on_device(&rctx, serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                             ymodem_on_begin, ymodem_on_data, NULL, RT_TICK_PER_SECOND))
     {
         /* Successful transfer */
         rt_kprintf("Download firmware to flash success.\n");
         rt_kprintf("System now will restart...\r\n");
         set_ota_flag();             // Mark OTA flag for bootloader
         ws2812_clearn_all(2);       // Clear WS2812 LEDs as visual indicator
         rt_thread_delay(rt_tick_from_millisecond(200));
         
         /* Reset CPU to boot into new firmware */
         extern void rt_hw_cpu_reset(void);
         rt_hw_cpu_reset();
 
         rt_thread_delay(rt_tick_from_millisecond(200));
     }
     else
     {
         /* Failed transfer */
         rt_thread_delay(RT_TICK_PER_SECOND);
         rt_kprintf("Update firmware fail.\n");
         set_ota_flag();             // Still set OTA flag
         ws2812_clearn_all(2);
 
         extern void rt_hw_cpu_reset(void);
         rt_hw_cpu_reset();
 
         // Optional: restore baud rate to default
         //config.baud_rate = BAUD_RATE_115200;
         //rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
     }
 
     return;
 }
 