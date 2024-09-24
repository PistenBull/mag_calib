# Magnetometer calibration

### Relevance of the work
- In the era of rapid development of unmanned transportation technologies, the issue of increasing the level of their functional safety is particularly relevant.
- In this context, magnetometer sensors play an important role as they help to ensure accurate determination of an object's orientation in space, which is critical for navigation and control of an unmanned vehicle.
- This need emphasizes the importance of calibrating magnetometers to maximize the accuracy and reliability of the system.

### Problems and purpose of the research

Reasons for using magnetometers in navigation systems of unmanned vehicles:
1. This is the need to determine the azimuth or course of an object, relative to the magnetic poles of the Earth;
2. Addition to GNSS systems for the possibility of realizing such tasks as: functions of autonomous flight (autopilot) and keeping the set position in difficult weather conditions of the environment;
3. Compensation of gyroscope angular orientation drift. This allows to maintain accurate orientation of the drone in space.

**Hard Iron distortions** - cause the magnetometer readings to shift, adding a fixed value to the measured magnetic field regardless of the orientation of the devices.

**Soft Iron distortions**- change the shape and direction of the magnetic field, which significantly affects the accuracy of the magnetometer readings.

![Pic. 1 Hard and Soft Iron distortion](https://github.com/PistenBull/magnetometer_calibration/blob/master/docs/images/%D0%92%D0%BB%D0%B8%D1%8F%D0%BD%D0%B8%D0%B5%20hard%20%D0%B8%20soft%20iron%20%D0%B8%D1%81%D0%BA%D0%B0%D0%B6%D0%B5%D0%BD%D0%B8%D0%B9%20%D0%BD%D0%B0%20%D0%BF%D0%BE%D0%BA%D0%B0%D0%B7%D0%B0%D0%BD%D0%B8%D1%8F%20%D0%BC%D0%B0%D0%B3%D0%BD%D0%B8%D1%82%D0%BE%D0%BC%D0%B5%D1%82%D1%80%D0%B0%20%E2%84%961.png "Pic. 1 Hard and Soft Iron distortion")

![Pic. 2 Hard and Soft Iron distortion](https://github.com/PistenBull/magnetometer_calibration/blob/master/docs/images/%D0%92%D0%BB%D0%B8%D1%8F%D0%BD%D0%B8%D0%B5%20hard%20%D0%B8%20soft%20iron%20%D0%B8%D1%81%D0%BA%D0%B0%D0%B6%D0%B5%D0%BD%D0%B8%D0%B9%20%D0%BD%D0%B0%20%D0%BF%D0%BE%D0%BA%D0%B0%D0%B7%D0%B0%D0%BD%D0%B8%D1%8F%20%D0%BC%D0%B0%D0%B3%D0%BD%D0%B8%D1%82%D0%BE%D0%BC%D0%B5%D1%82%D1%80%D0%B0%20%E2%84%962.jpg "Pic. 2 Hard and Soft Iron distortion")

**The purpose of this research** - is to investigate and develop magnetometer calibration techniques to minimize environmental distortion and improve the navigation accuracy of an unmanned vehicle.

### The essence of the calibration method

- **Transformation matrix** - Determined by collecting data at different sensor positions and then analyzing by least squares to compensate for anisotropic distortion.
- **Bias matrix** - The bias matrix is calculated from the average of the steady-state sensor readings to correct for permanent offsets.

![Formula for calculating calibrated magnetometer data](https://github.com/PistenBull/magnetometer_calibration/blob/master/docs/images/%D0%A4%D0%BE%D1%80%D0%BC%D1%83%D0%BB%D0%B0%20%D0%BF%D1%80%D0%B8%D0%BC%D0%B5%D0%BD%D0%B5%D0%BD%D0%B8%D1%8F%20%D0%BA%D0%B0%D0%BB%D0%B8%D0%B1%D1%80%D0%BE%D0%B2%D0%BE%D1%87%D0%BD%D1%8B%D1%85%20%D0%BC%D0%B0%D1%82%D1%80%D0%B8%D1%86%20%E2%84%961.PNG "Formula for calculating calibrated magnetometer data")

### Experimental setup and research procedure

- **Arduino MEGA 2560** - It is Arduino's flagship development platform based on the ATmega2560 microcontroller.

![Arduino MEGA 2560](https://github.com/PistenBull/magnetometer_calibration/blob/master/docs/images/%D0%9E%D0%B1%D0%BE%D1%80%D1%83%D0%B4%D0%BE%D0%B2%D0%B0%D0%BD%D0%B8%D0%B5.%20Arduino%20MEGA.png "Arduino MEGA 2560")

- **Spurkfun 9DOF SEN10724** - This is a magnetic-inertial module that includes the following sensors:
1. The ADXL345 is a three-axis accelerometer;
2. ITG3200 is a three-axis gyroscope;
3. HMC5883L – трехосевой магнитометр.

![Spurkfun 9DOF SEN10724](https://github.com/PistenBull/magnetometer_calibration/blob/master/docs/images/%D0%9E%D0%B1%D0%BE%D1%80%D1%83%D0%B4%D0%BE%D0%B2%D0%B0%D0%BD%D0%B8%D0%B5.%20Spurkfun%209DOF%20SEN10724.png "Spurkfun 9DOF SEN10724")

### Experimental setup and research procedure

![Schematic diagram of the experimental setup](https://github.com/PistenBull/magnetometer_calibration/blob/master/docs/images/%D0%A1%D1%85%D0%B5%D0%BC%D0%B0%20%D1%8D%D0%BA%D1%81%D0%BF%D0%B5%D1%80%D0%B8%D0%BC%D0%B5%D0%BD%D1%82%D0%B0%D0%BB%D1%8C%D0%BD%D0%BE%D0%B9%20%D1%83%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B8.png "Schematic diagram of the experimental setup")

![Experimental setup](https://github.com/PistenBull/magnetometer_calibration/blob/master/docs/images/%D0%AD%D0%BA%D1%81%D0%BF%D0%B5%D1%80%D0%B8%D0%BC%D0%B5%D0%BD%D1%82%D0%B0%D0%BB%D1%8C%D0%BD%D0%B0%D1%8F%20%D1%83%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0.jpg "Экспериментальная установка")

In the course of the study, a series of experiments were conducted to collect data before and after magnetometer calibration.

![Visualization of magnetometer readings before calibration](https://github.com/PistenBull/magnetometer_calibration/blob/master/docs/images/%D0%9F%D0%BE%D0%BA%D0%B0%D0%B7%D0%B0%D0%BD%D0%B8%D1%8F%20%D0%BC%D0%B0%D0%B3%D0%BD%D0%B8%D1%82%D0%BE%D0%BC%D0%B5%D1%82%D1%80%D0%B0%20%D0%B4%D0%BE%20%D0%BA%D0%B0%D0%BB%D0%B8%D0%B1%D1%80%D0%BE%D0%B2%D0%BA%D0%B8%20%E2%84%961.jpg "Visualization of magnetometer readings before calibration")

Statistical analysis showed a significant increase in the accuracy of magnetometer readings after calibration, which confirms the effectiveness of the proposed calibration technique.

![Visualization of magnetometer readings after calibration](https://github.com/PistenBull/magnetometer_calibration/blob/master/docs/images/%D0%9F%D0%BE%D0%BA%D0%B0%D0%B7%D0%B0%D0%BD%D0%B8%D1%8F%20%D0%BC%D0%B0%D0%B3%D0%BD%D0%B8%D1%82%D0%BE%D0%BC%D0%B5%D1%82%D1%80%D0%B0%20%D0%BF%D0%BE%D1%81%D0%BB%D0%B5%20%D0%BA%D0%B0%D0%BB%D0%B8%D0%B1%D1%80%D0%BE%D0%B2%D0%BA%D0%B8.png "Visualization of magnetometer readings after calibration")

### Conclusion

The research showed a significant improvement in the accuracy of magnetometer data after calibration, which confirms its necessity to improve the safety, accuracy and reliability of unmanned vehicle navigation systems. Thus, this study makes an important contribution to the development and improvement of functional safety systems for unmanned vehicles.

------------

### Example 

The [example](https://github.com/PistenBull/magnetometer_calibration/tree/master/example) tab considers a special case of using the calibration method. The functions are named differently, but the essence of the method does not change.

------------

### How to start using

To perform the calibration it is necessary to collect data by rotating the device in 6 planes (in the faces of the cube), and in each plane it is necessary to fix the coordinates of 2 points, the second of which is obtained by rotation by 180 degrees relative to the first point. As a result, 12 points are obtained.

Then you just need to run the calibration function and everything should work.