<div id="home" align="center">
  <h1>GY-271 (HMC5883L) 3-Axis Magnetometer</h1>
  
  ![license](https://badgen.net/github/license/helviorubens/ESP8266-IMU)
  ![commits](https://badgen.net/github/commits/helviorubens/ESP8266-IMU/)
  ![last-commit](https://badgen.net/github/last-commit/helviorubens/ESP8266-IMU)
  [![twitter](https://badgen.net/badge/icon/helviorubens?icon=twitter&label)](https://twitter.com/helviorubens)
    
  <h3>Library for magnetometer as digital compass, to get the geographic direction of the X-axis (0° NORTH)</h3>
  
  | <a href="#disclaimer">DISCLAIMER</a>
  | <a href="#how-to-use">HOW TO USE</a>|
  
</div>

# DISCLAIMER

## About Development

**THIS LIBRARY AND EXAMPLES ARE UNDER DEVELOPMENT!**

I used only *NodeMCU/ESP8266* to perform the tests. There is compatibility with other boards, *e.g.*, Arduino UNO and ESP32. Sometimes minor code changes are necessary, but this is not in the scope of this project.

## About Original Library

I did use the **[Grove_3Axis_Digital_Compass_HMC5883L library](https://github.com/Seeed-Studio/Grove_3Axis_Digital_Compass_HMC5883L)** by *Seeed Studio* to implement my own library, with new reviewed methods and functionalities. The original author is *Frankie* Chu, and *Yihui Xiong* as collaborator. The original library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.

## About Code Editor

This library and examples have been tested with [**Arduino IDE**](https://www.arduino.cc/) and written with [*Visual Studio Code*](https://code.visualstudio.com/) as source code editor, using the extension *Arduino for Visual Studio Code* by Microsoft. If you want to install it, I used the Paul Lunn tutorial on his website [*Maker Pro*](https://maker.pro/arduino/tutorial/how-to-use-visual-studio-code-for-arduino).

If you prefer not to use the *Visual Studio Code*, the library and examples are **100% compatible with Arduino IDE**.

<br/>
<div align="right">
    <b><a href="#home">▲ back to HOME</a></b>
</div>
<br/>

# HOW TO USE

There are 3 global variables to setting sensor parameters: `SENSOR_GAIN`, `MEASUREMENT` and `DECLINATION`.

![HowToUse](https://drive.google.com/uc?export=view&id=1L-K5pJV8laFztU87CuPY1b60U7e13Ihr)

**If you choose not to use them**, the default values are `SENSOR_GAIN = 1.3 Ga`, `MEASUREMENT = MEASUREMENT_CONTINUOUS` and `DECLINATION = 0.0 radians`. 

## SENSOR GAIN

The *sensor gain* has 8 predefined values: `(0.88 / 1.3 / 1.9 / 2.5 / 4.0 / 4.7 / 5.6 / 8.1) Gauss`.

Choose one of them to set the scale of compass and get a proper values measurement. *Lower values indicates more accuracy*, but if you have others objects interfering in the compass, you may need to choose a bigger value.

## MEASUREMENT MODE

The *measurement mode* has 3 predefined values: `MEASUREMENT_CONTINUOUS`, `MEASUREMENT_SINGLE` and `MEASUREMENT_IDLE`. These operating modes are related to the power management of the device.

* `MEASUREMENT_CONTINUOUS`: the device continuously makes measurements, at user selectable rate, and places measured data in data output registers.

* `MEASUREMENT_SINGLE`: the device makes a single measurement and places the measured data in data output registers. After that the device is placed in idle mode.

* `MEASUREMENT_IDLE`: during this mode the device is accessible through the I2C bus, but major sources of power consumption are disabled.

## DECLINATION ANGLE

The *declination angle* can be found [HERE](http://www.magnetic-declination.com/). By default, the displayed value is the declination about your current location.

![declination-angle](https://drive.google.com/uc?export=view&id=1TmYVQeT8nF58HJG3bRKtE7ZwprCa5h1Z)

This value is important for accurate measurements, because the magnetic field is not perfect around the Earth. The core of the Earth changes constantly and affects the measured values.

**HOW TO SET DECLINATION VARIABLE?** If your declination angle is, for example, -21°36' (*21 degrees and 36 minutes WEST*) then you set the variable like `-21.36`. Otherwise, the declination angle will be 0.0.

<br/>
<div align="right">
    <b><a href="#home">▲ back to HOME</a></b>
</div>
<br/>

