#include "MPU9250.h"

#include "Adafruit_BMP280.h"

 

MPU9250 mpu = MPU9250();

Adafruit_BMP280 bme; // I2C

 

void setup(void) {

  Serial.begin(9600); 

  uint8_t temp = mpu.begin();

  if (!bme.begin())

  { 

    Serial.println("Could not find a valid BMP280 sensor, check wiring!");

    while (1);

  }

}

 

void loop() {

  // MPU925 Acceleromter:

  mpu.set_accel_range(RANGE_4G);

  mpu.get_accel();

  Serial.print("X: ");  Serial.print(mpu.x);

  Serial.print(" Y: "); Serial.print(mpu.y);

  Serial.print(" Z: "); Serial.print(mpu.z);

 

  mpu.get_accel_g();

  Serial.print(" X_g: "); Serial.print(mpu.x_g,2);

  Serial.print(" Y_g: "); Serial.print(mpu.y_g,2);

  Serial.print(" Z_g: "); Serial.print(mpu.z_g,2);  Serial.println(" G");

 

  // MPU9250 Gyro:

  mpu.set_gyro_range(RANGE_GYRO_250);

  mpu.get_gyro();

  Serial.print("GX: ");  Serial.print(mpu.gx);

  Serial.print(" GY: "); Serial.print(mpu.gy);

  Serial.print(" GZ: "); Serial.print(mpu.gz);

 

  mpu.get_gyro_d();

  Serial.print(" GX_g: "); Serial.print(mpu.gx_d,2);

  Serial.print(" GY_g: "); Serial.print(mpu.gy_d,2);

  Serial.print(" GZ_g: "); Serial.print(mpu.gz_d,2); Serial.println(" º/s");

 

  // MPU9250 Magnetometer:

  mpu.set_mag_scale(SCALE_14_BITS);

  mpu.set_mag_speed(MAG_8_Hz);

  if(!mpu.get_mag())

  {

    Serial.print("MX: ");  Serial.print(mpu.mx);

    Serial.print(" MY: "); Serial.print(mpu.my);

    Serial.print(" MZ: "); Serial.print(mpu.mz);

 

    mpu.get_mag_t();

    Serial.print(" MX_t: "); Serial.print(mpu.mx_t,2);

    Serial.print(" MY_t: "); Serial.print(mpu.my_t,2);

    Serial.print(" MZ_t: "); Serial.print(mpu.mz_t,2); Serial.println(" uT");

  }

  else{

    // |X|+|Y|+|Z| must be < 4912 microT to sensor measure correctly

    Serial.println("Overflow no magnetometer");

  }

 

  // MPU9250 Temperature:    

  Serial.print("MPU9250 Temperature is "); 

  Serial.print((((float) mpu.get_temp()) / 333.87 + 21.0), 1); 

  Serial.println(" degrees C");

 

  // BMP280 Temperature:

  Serial.print("BMP280 Temperature = ");

  Serial.print(bme.readTemperature());

  Serial.println(" degrees C");

 

  // BMP280 Pressure: 

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure());

  Serial.println(" Pa");

 

  // BMP280 Approximate Altitude:

  Serial.print("Approx altitude = ");

  Serial.print(bme.readAltitude(1005.00)); // This should be adjusted to your local forcast of barometric pressure (hPA) at sea level.

  Serial.println(" m");

  Serial.println();

 

  delay(1000);

}
