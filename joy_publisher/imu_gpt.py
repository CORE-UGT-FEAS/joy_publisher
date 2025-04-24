from __future__ import print_function
import qwiic_icm20948
import time
import sys
import math
import numpy as np

class IMUSensor:
    # Sensitivity values based on full-scale range settings
    ACCEL_SENSITIVITY = {0x00: 16384, 0x01: 8192, 0x02: 4096, 0x03: 2048}  # LSB per G
    GYRO_SENSITIVITY = {0x00: 131.072, 0x01: 65.536, 0x02: 32.768, 0x03: 16.384}  # LSB per °/s

    def __init__(self, accel_mode=0x00, gyro_mode=0x00, alpha=0.8):
        self.imu = qwiic_icm20948.QwiicIcm20948()
        self.accel_mode = accel_mode  # gpm2 (±2G)
        self.gyro_mode = gyro_mode  # dps250 (±250°/s)
        self.alpha = alpha  # Weight for complementary filter

        if not self.imu.connected:
            raise Exception("The Qwiic ICM20948 device isn't connected to the system. Please check your connection.")

        self.imu.begin()

        # Rotation tracking (degrees)
        self.rotation = {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
        self.last_time = time.time()

        # Initialize pitch and roll using gravity-based estimation
        self.initialize_orientation()

    def convert_accel(self, raw_value):
        """ Convert raw accelerometer value to Gs """
        sensitivity = self.ACCEL_SENSITIVITY.get(self.accel_mode, 16384)
        return raw_value / sensitivity

    def convert_gyro(self, raw_value):
        """ Convert raw gyroscope value to degrees per second """
        sensitivity = self.GYRO_SENSITIVITY.get(self.gyro_mode, 131.072)
        return raw_value / sensitivity

    def initialize_orientation(self):
        """ Estimate initial pitch and roll from accelerometer readings """
        self.imu.getAgmt()
        accel_x = self.convert_accel(self.imu.axRaw)
        accel_y = self.convert_accel(self.imu.ayRaw)
        accel_z = self.convert_accel(self.imu.azRaw)
        my = self.imu.myRaw
        mx = self.imu.mxRaw
        self.yaw = np.arctan2(my, mx)

        self.rotation["pitch"] = math.degrees(math.atan2(accel_y, accel_z))
        self.rotation["roll"] = math.degrees(math.atan2(accel_x, accel_z))
        self.rotation["yaw"] = 0 # Yaw always starts at zero

    def complementary_filter(self, accel_data, gyro_data, dt):
        """ Apply complementary filter to combine accelerometer and gyroscope data """
        # Estimate angles from accelerometer
        accel_pitch = math.degrees(math.atan2(accel_data["y"], accel_data["z"]))
        accel_roll = math.degrees(math.atan2(accel_data["x"], accel_data["z"]))

        # Update the angles using the gyroscope data for pitch and roll
        self.rotation["pitch"] = self.alpha * (self.rotation["pitch"] + gyro_data["x"] * dt) + (1 - self.alpha) * accel_pitch
        self.rotation["roll"] = self.alpha * (self.rotation["roll"] - gyro_data["y"] * dt) + (1 - self.alpha) * accel_roll

        # For yaw, we rely only on the gyroscope data (integration of gyro.z)
        self.rotation["yaw"] += gyro_data["z"] * dt  # Integrating yaw directly from the gyroscope

    def read_data(self):
        """ Read sensor data, update rotation, and return processed values """
        if self.imu.dataReady():
            self.imu.getAgmt()  # Read all axis and temp from sensor

            accel = {
                "x": self.convert_accel(self.imu.axRaw),
                "y": self.convert_accel(self.imu.ayRaw),
                "z": self.convert_accel(self.imu.azRaw)
            }

            gyro = {
                "x": self.convert_gyro(self.imu.gxRaw),
                "y": self.convert_gyro(self.imu.gyRaw),
                "z": self.convert_gyro(self.imu.gzRaw)
            }

            # Time delta for integration
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            self.complementary_filter(accel, gyro, dt)

            return {"accel": accel, "gyro": gyro, "rotation": self.rotation}
        else:
            return None


def runExample():
    print("\nSparkFun 9DoF ICM-20948 Sensor Example - Tracking Rotation\n")
    imu_sensor = IMUSensor(alpha=0.98)  # alpha controls the blend between accelerometer and gyroscope data

    last_print_time = time.time()  # Time to track when to print
    print_interval = 0.5  # Print every 0.5 seconds

    while True:
        data = imu_sensor.read_data()  # Integrating data as quickly as possible
        current_time = time.time()

        if data and current_time - last_print_time >= print_interval:
            # Print acceleration and rotation data every 0.5 seconds
            print(f"Accel (G): X={data['accel']['x']:.3f}, Y={data['accel']['y']:.3f}, Z={data['accel']['z']:.3f} | ",
                  f"Rotation (deg): Pitch={data['rotation']['pitch']:.2f}, Yaw={data['rotation']['yaw']:.2f}, Roll={data['rotation']['roll']:.2f}")

            last_print_time = current_time  # Update the time for the next print

        time.sleep(0.001)  # Sleep just a tiny bit to avoid maxing out the processor's clock

if __name__ == '__main__':
    try:
        runExample()
    except (KeyboardInterrupt, SystemExit):
        print("\nEnding Example")
        sys.exit(0)