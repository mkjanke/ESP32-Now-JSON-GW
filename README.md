# ESP-NOW to Serial JSON message handler

Listens for JSON formatted broadcast ESP-NOW messages, forwards to serial port. 

* Messages received via ESP-NOW via task_read_ESP_NOW_Write_Serial are queued to send_to_Serial_queue.
* Messages dequeued from send_to_Serial_queue are pre-pended with "JSON>> " and forwarded serial port.
* Messages received via serial port are forwarded to ESP-NOW via task_read_Serial_Write_ESP_NOW. 
* Messages are assumed to be JSON formatted, terminated with newline "\n".

This project is a companion to https://github.com/mkjanke/ESP32-Fanspeed-NOW, https://github.com/mkjanke/ESP32-SeeLevel-NOW, https://github.com/mkjanke/ESP32-NOW-Govee, and https://github.com/mkjanke/ESP32-Nextion-NOW , acting as the communication bridge between the remote ESP's and a Pi or other serial device.

This is built assuming that this ESP32 is attached via serial port to a Node-Red instance or other application that processes serial messages from remote ESP32's and formats messages destined to remote ESP32's.

The remote ESP32's (referenced above) listen for ESP-NOW broadcasts, discard any messages where JSON "D" does not match compiled-in device name, and act on remaining based on the device's code. Message formats are described in the above links.

The remote ESP's broadcast JSON formatted data, which is received by this ESP and promiscuously forwarded out to this ESP's serial port.