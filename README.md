本项目为 基于 ESP32C6 的太阳能追踪系统，使用 ESP-IDF 开发。  
系统可通过 WiFi 获取 Stellarium API 的太阳位置数据，通过 BLE 蓝牙 与手机同步 GPS 和时间，同时支持手动模式切换。  

双步进电机精确追踪太阳，提高太阳能采集效率。  
通过合理调度 WiFi、BLE 和步进电机模块的休眠，实现低功耗与续航最大化。  
内置磁力计支持系统校准，无需精确放置即可使用。  


ESP32C6 Solar Tracking System

This project is a solar tracking system based on ESP32C6, developed using the ESP-IDF framework.  
It can fetch sun position data via the Stellarium API over WiFi, connect via BLE for GPS and time updates from a mobile phone, and switch to manual mode when needed.  

The system uses dual stepper motors for precise solar tracking, maximizing solar energy harvesting.  
Power consumption is optimized by carefully scheduling WiFi, BLE, and stepper motor sleep modes.  
A built-in magnetometer allows the system to calibrate automatically and be placed anywhere without manual alignment.  
