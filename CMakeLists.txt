cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cellular_fundamentals)

# NORDIC SDK APP START
target_include_directories(app PRIVATE include/)
target_include_directories(app PRIVATE tasks/MQTT_TASK/)
target_include_directories(app PRIVATE tasks/GNSS_TASK/)
target_include_directories(app PRIVATE tasks/RS485_TASK/)
target_include_directories(app PRIVATE tasks/RTC_TASK/)
target_sources(app PRIVATE src/mqtt_connection.c)
target_sources(app PRIVATE src/gnss_connection.c)
target_sources(app PRIVATE src/rtc.c)
target_sources(app PRIVATE src/rs485.c)
target_sources(app PRIVATE src/leds.c)
target_sources(app PRIVATE src/adc.c)
target_sources(app PRIVATE tasks/MQTT_TASK/mqtt_task.c)
target_sources(app PRIVATE tasks/GNSS_TASK/gnss_task.c)
target_sources(app PRIVATE tasks/RS485_TASK/rs485_task.c)
target_sources(app PRIVATE tasks/RTC_TASK/rtc_task.c)

target_sources(app PRIVATE src/main.c)


# NORDIC SDK APP END