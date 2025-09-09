/*
 * 程序清单：这是一个独立看门狗设备使用例程
 * Program description: This is a standalone watchdog device usage example
 *
 * 例程导出了 wdt_sample 命令到控制终端
 * The example exports the `wdt_sample` command to the control terminal
 *
 * 命令调用格式：wdt_sample wdt
 * Command format: wdt_sample wdt
 *
 * 命令解释：命令第二个参数是要使用的看门狗设备名称，为空则使用例程默认的看门狗设备。
 * Command explanation: The second argument is the name of the watchdog device to use. 
 * If empty, the example will use the default watchdog device.
 *
 * 程序功能：程序通过设备名称查找看门狗设备，然后初始化设备并设置看门狗设备溢出时间。
 * Program function: The program looks up the watchdog device by name, initializes it, 
 * and sets its timeout period.
 *
 * 然后设置空闲线程回调函数，在回调函数里会喂狗。
 * Then it sets an idle thread callback function, where the watchdog is fed ("keepalive").
 */

 #include <rtthread.h>
 #include <rtdevice.h>
 
 #define WDT_DEVICE_NAME    "wdt"    /* 看门狗设备名称 | Watchdog device name */
 
 static rt_device_t wdg_dev;         /* 看门狗设备句柄 | Handle to watchdog device */
 
 /* Idle hook function: called when CPU is idle */
 static void idle_hook(void)
 {
     /* 在空闲线程的回调函数里喂狗 | Feed the watchdog inside the idle thread callback */
     rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
 }
 
 /**
  * @brief Initialize the watchdog device
  * 
  * Steps:
  * 1. Find watchdog device by name
  * 2. Initialize device
  * 3. Set watchdog timeout
  * 4. Start watchdog
  * 5. Register idle hook for periodic feeding
  *
  * @return RT_EOK (0) on success, negative error code on failure
  */
 int wdt_init(void)
 {
     rt_err_t ret = RT_EOK;
     rt_uint32_t timeout = 3;        /* 溢出时间，单位：秒 | Timeout in seconds */
     char device_name[RT_NAME_MAX] = WDT_DEVICE_NAME;
 
     /* 根据设备名称查找看门狗设备，获取设备句柄 
      * Find watchdog device by name and get handle */
     wdg_dev = rt_device_find(WDT_DEVICE_NAME);
     if (!wdg_dev)
     {
         rt_kprintf("find %s failed!\n", device_name);
         return RT_ERROR;
     }
 
     /* 初始化设备 | Initialize the watchdog device */
     rt_device_init(wdg_dev);
 
     /* 设置看门狗溢出时间 | Set watchdog timeout */
     ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
     if (ret != RT_EOK)
     {
         rt_kprintf("set %s timeout failed!\n", device_name);
         return RT_ERROR;
     }
 
     /* 启动看门狗 | Start the watchdog */
     ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
     if (ret != RT_EOK)
     {
         rt_kprintf("start %s failed!\n", device_name);
         return -RT_ERROR;
     }
 
     /* 设置空闲线程回调函数 | Set idle thread callback to feed the watchdog */
     rt_thread_idle_sethook(idle_hook);
 
     return ret;
 }
 