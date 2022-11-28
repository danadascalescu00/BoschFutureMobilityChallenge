#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import sys
sys.path.append('.')
import os.path
import time
import math

import rospy

from utils.msg import IMU

sensorType = "BNO055"

if sensorType == "BNO055":
    import RTIMU
elif sensorType == "MPU6050":
    from mpu6050 import mpu6050

class imuNODE():
    def __init__(self): 
        if sensorType == "BNO055":
            import RTIMU
            self.SETTINGS_FILE = "RTIMULib"
            print("Using settings file " + self.SETTINGS_FILE + ".ini")
            if not os.path.exists(self.SETTINGS_FILE + ".ini"):
                print("Settings file does not exist, will be created")
            self.s = RTIMU.Settings(self.SETTINGS_FILE)
        else:
            from mpu6050 import mpu6050
        
        rospy.init_node('imuNODE', anonymous=False)
        
        # BNO publisher object
        self.BNO_publisher = rospy.Publisher("/automobile/imu", IMU, queue_size=1)
        
    #================================ RUN ========================================
    def run(self):
        rospy.loginfo("starting imuNODE")
        self._initIMU()
        self._getting()
    
    #================================ INIT IMU ========================================
    def _initIMU(self):
        if sensorType == "BNO055":
            self.imu = RTIMU.RTIMU(self.s)
            
            if (not self.imu.IMUInit()):
                sys.exit(1)
            print("IMU Name: " + self.imu.IMUName())
            self.imu.setSlerpPower(0.02)
            self.imu.setGyroEnable(True)
            self.imu.setAccelEnable(True)
            self.imu.setCompassEnable(True)
            self.poll_interval = self.imu.IMUGetPollInterval()
        else:
            self.imu = mpu6050(0x68)
            self.poll_interval = 100

    #================================ GETTING ========================================
    def _getting(self):
        while not rospy.is_shutdown():
            imudata = IMU()
            if sensorType == "BNO055":
                if self.imu.IMURead():
                    data = self.imu.getIMUData()
                    fusionPose = data["fusionPose"]
                    accel = data["accel"]
                    
                    imudata.roll   =  math.degrees(fusionPose[0])
                    imudata.pitch  =  math.degrees(fusionPose[1])
                    imudata.yaw    =  math.degrees(fusionPose[2])
                    imudata.accelx =  accel[0]
                    imudata.accely =  accel[1]
                    imudata.accelz =  accel[2]
            else:
                data = self.imu.get_accel_data()
                dataa = self.imu.get_gyro_data()

                imudata.accelx = data["x"]
                imudata.accely = data["y"]
                imudata.accelz = data["z"]
                
                imudata.roll  =  math.degrees(dataa["x"])
                imudata.pitch =  math.degrees(dataa["y"])
                imudata.yaw   =  math.degrees(dataa["z"])
            self.BNO_publisher.publish(imudata)
            
            time.sleep(self.poll_interval*1.0/1000.0)
        
if __name__ == "__main__":
    imuNod = imuNODE()
    imuNod.run()
