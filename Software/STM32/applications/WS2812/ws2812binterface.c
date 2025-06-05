/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      stm32f407_driver_ws2812b_interface.c
 * @brief     stm32f407 driver ws2812b interface source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2021-11-13
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/11/13  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "ws2812binterface.h"
#include "board.h"
#include <stdarg.h>
#include "drv_spi.h"

rt_device_t spi_dev = NULL;

//RT—thread SPI框架初始化
static int rt_hw_spi_ws2812_init(void)
{
    rt_hw_spi_device_attach("spi2", "spi20", GPIOB, GPIO_PIN_9);//把spi2的0号设备挂载在spi总线上
    spi_dev = rt_device_find("spi20");
    if (NULL == spi_dev)
        return 1;

    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.max_hz = 25 * 1000 * 1000; /* 20M */

    rt_spi_configure(spi_dev, &cfg);
    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(rt_hw_spi_ws2812_init);

/**
 * @brief  interface spi 10mhz bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init 10mhz failed
 * @note   none
 */
uint8_t ws2812b_interface_spi_10mhz_init(void)
{

    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t ws2812b_interface_spi_deinit(void)
{
    return 0;
}

/**
 * @brief     interface spi bus write command
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t ws2812b_interface_spi_write_cmd(uint8_t *buf, uint16_t len)
{
    if (len == rt_spi_send(spi_dev, buf, len))
        return 0;
    return 1;
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void ws2812b_interface_delay_ms(uint32_t ms)
{
    rt_thread_mdelay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void ws2812b_interface_debug_print(const char *const fmt, ...)
{
    char str[256];
    uint16_t len;
    va_list args;

    va_start(args, fmt);
    vsnprintf((char *)str, 255, (char const *)fmt, args);
    va_end(args);

    len = strlen((char *)str);
    printf("%s\r\n", str);
}
