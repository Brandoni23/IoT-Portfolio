# Project 2: Rainmaker I2C Temperature Reading

This is my code for adding a widget to the Rainmaker cloud server in app_main.c. It also accesses the temperature data from the app_driver.c code file which uses I2C to get the data. The app_priv.h header file transfers the temperature's global float variable from the driver file to the main file. The readings can be monitored by the Rainmaker mobile app which scans the barcode from the ESP32-S2 when building the app_driver.c code.
