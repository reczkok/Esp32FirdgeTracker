# A simple device for fridge tracking made for a uni course
Its main purpose was to get familiar with embedded programming and simple electronics.\
It uses an analogue **temperature sensor** and an **ultrasonic distance sensor** to gather information about the fridge (temperature and whether it is open).
It is designed to be used with [an app](https://github.com/reczkok/Esp32FirdgeTracker-App) which was made using React Native and Expo and uses Bluetooth to send SSID and password to the device (for it to then connect to a wireless network and send data using MQTT). The app also gathers MAC information from the device to send it to a server - so the device can be associated with the appropriate user.
