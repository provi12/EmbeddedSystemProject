# EmbeddedSystemProject

My Project includes 
- W25Q64 NOR based Flash Memory accessed using SPI communication
- DHT11 Temperature and humidity sensor
- LCD 20*2 Display

I have used FreeRTOS for schdeuling and Task control. 
Following are details about my software - 
1 - vSensorDataWriteTask - Used to read dht11 sensor data and write them into flash memory
2 - vSensorDataReadTask - Used to read user input from UART and read data from flash memory like Temperature or Humidity then display it at LCD20*2 display.
3 - used mutex to avoid data curruption at flash memory.
4 - I have used opensource library named libopencm3 used for stm32.

libopencm3 - https://github.com/libopencm3/libopencm3.git

Note : Not able to test it as did not had required components but compilation successfully done in my system.

