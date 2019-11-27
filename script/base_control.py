#!/usr/bin/python
# coding=gbk
# Copyright 2019 Wechange Tech.
# Developer: FuZhi, Liu (liu.fuzhi@wechangetech.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import rospy
import tf
import time
import sys
import math
import serial
import string
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
import ctypes

base_type = os.getenv('BASE_TYPE')

#class queue is design for uart receive data cache
class queue:
    def __init__(self, capacity = 1024*4):
        self.capacity = capacity
        self.size = 0
        self.front = 0
        self.rear = 0
        self.array = [0]*capacity
 
    def is_empty(self):
        return 0 == self.size
 
    def is_full(self):
        return self.size == self.capacity
 
    def enqueue(self, element):
        if self.is_full():
            raise Exception('queue is full')
        self.array[self.rear] = element
        self.size += 1
        self.rear = (self.rear + 1) % self.capacity
 
    def dequeue(self):
        if self.is_empty():
            raise Exception('queue is empty')
        self.size -= 1
        self.front = (self.front + 1) % self.capacity
 
    def get_front(self):
        return self.array[self.front]
    
    def get_front_second(self):
        return self.array[((self.front + 1) % self.capacity)]

    def get_queue_length(self):
        return (self.rear - self.front + self.capacity) % self.capacity

    def show_queue(self):
        for i in range(self.capacity):
            print self.array[i],
        print(' ')
#class BaseControl:
class BaseControl:
    def __init__(self):
        #Get params
        self.baseId = rospy.get_param('~base_id','base_footprint')
        self.odomId = rospy.get_param('~odom_id','odom')
        self.device_port = rospy.get_param('~port','/dev/ttyUSB0')
        self.baudrate = int(rospy.get_param('~baudrate','115200'))
        self.odom_freq = int(rospy.get_param('~odom_freq','50'))
        self.odom_topic = rospy.get_param('~odom_topic','/odom')
        self.battery_topic = rospy.get_param('~battery_topic','battery')
        self.battery_freq = float(rospy.get_param('~battery_freq','1'))
        self.cmd_vel_topic= rospy.get_param('~cmd_vel_topic','/cmd_vel')
        self.ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic','/ackermann_cmd_topic')

        self.pub_imu = bool(rospy.get_param('~pub_imu','False'))
        if(self.pub_imu == True):
            self.imuId = rospy.get_param('~imu_id','imu')
            self.imu_topic = rospy.get_param('~imu_topic','imu')
            self.imu_freq = float(rospy.get_param('~imu_freq','50'))
        self.sub_ackermann = bool(rospy.get_param('~sub_ackermann','False'))


        #define param
        self.current_time = rospy.Time.now()
        self.previous_time = self.current_time
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.serialIDLE_flag = 0
        self.trans_x = 0.0
        self.trans_y = 0.0
        self.rotat_z = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.sendcounter = 0
        self.ImuErrFlag = False
        self.EncoderFlag = False
        self.BatteryFlag = False
        self.OdomTimeCounter = 0
        self.BatteryTimeCounter = 0
        self.Circleloop = queue(capacity = 1024*4)
        self.Vx = 0
        self.Vy = 0
        self.Vyaw = 0
        self.Yawz = 0
        self.Vvoltage = 0
        self.Icurrent = 0
        self.Gyro = [0,0,0]
        self.Accel = [0,0,0]
        self.Quat = [0,0,0,0]
        self.movebase_firmware_version = [0,0,0]
        self.movebase_hardware_version = [0,0,0]
        self.movebase_type = ["NanoCar","NanoRobot","4WD_OMNI","4WD"]
        self.motor_type = ["25GA370","37GB520"]
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_ackermann_cmd_time = rospy.Time.now()
        # Serial Communication
        try:
            self.serial = serial.Serial(self.device_port,self.baudrate,timeout=10)
            rospy.loginfo("Opening Serial")
            try:
                if self.serial.in_waiting:
                    self.serial.readall()
            except:
                rospy.loginfo("Opening Serial Try Faild")
                pass
        except:
            rospy.logerr("Can not open Serial"+self.device_port)
            self.serial.close
            sys.exit(0)
        rospy.loginfo("Serial Open Succeed")
        #if move base type is ackermann car like robot and use ackermann msg ,sud ackermann topic,else sub cmd_vel topic
        if((base_type == 'NanoCar') & (self.sub_ackermann == True)):
            from ackermann_msgs.msg import AckermannDriveStamped
            self.sub = rospy.Subscriber(self.ackermann_cmd_topic,AckermannDriveStamped,self.ackermannCmdCB,queue_size=20)
        else:
            self.sub = rospy.Subscriber(self.cmd_vel_topic,Twist,self.cmdCB,queue_size=20)
        self.pub = rospy.Publisher(self.odom_topic,Odometry,queue_size=10)
        self.battery_pub = rospy.Publisher(self.battery_topic,BatteryState,queue_size=3)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.odom_freq),self.timerOdomCB)
        self.timer_battery = rospy.Timer(rospy.Duration(1.0/self.battery_freq),self.timerBatteryCB)  
        self.timer_communication = rospy.Timer(rospy.Duration(5.0/1000),self.timerCommunicationCB)
        #inorder to compatibility old version firmware,imu topic is NOT pud in default
        if(self.pub_imu):
            self.timer_imu = rospy.Timer(rospy.Duration(1.0/self.imu_freq),self.timerIMUCB) 
            self.imu_pub = rospy.Publisher(self.imu_topic,Imu,queue_size=10)
        self.getVersion()
        #move base imu initialization need about 2s,during initialization,move base system is blocked
        #so need this gap
        time.sleep(2.2)
        self.getSN()
        time.sleep(0.01)
        self.getInfo()
    #CRC-8 Calculate
    def crc_1byte(self,data):
        crc_1byte = 0
        for i in range(0,8):
            if((crc_1byte^data)&0x01):
                crc_1byte^=0x18
                crc_1byte>>=1
                crc_1byte|=0x80
            else:
                crc_1byte>>=1
            data>>=1
        return crc_1byte
    def crc_byte(self,data,length):
        ret = 0
        for i in range(length):
            ret = self.crc_1byte(ret^data[i])
        return ret               
    #Subscribe vel_cmd call this to send vel cmd to move base
    def cmdCB(self,data):
        self.trans_x = data.linear.x
        self.trans_y = data.linear.y
        self.rotat_z = data.angular.z
        self.last_cmd_vel_time = rospy.Time.now()
        output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x01) + \
            chr((int(self.trans_x*1000.0)>>8)&0xff) + chr(int(self.trans_x*1000.0)&0xff) + \
            chr((int(self.trans_y*1000.0)>>8)&0xff) + chr(int(self.trans_y*1000.0)&0xff) + \
            chr((int(self.rotat_z*1000.0)>>8)&0xff) + chr(int(self.rotat_z*1000.0)&0xff) + \
            chr(0x00)
        outputdata = [0x5a,0x0c,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        outputdata[4] = (int(self.trans_x*1000.0)>>8)&0xff
        outputdata[5] = int(self.trans_x*1000.0)&0xff
        outputdata[6] = (int(self.trans_y*1000.0)>>8)&0xff
        outputdata[7] = int(self.trans_y*1000.0)&0xff
        outputdata[8] = (int(self.rotat_z*1000.0)>>8)&0xff
        outputdata[9] = int(self.rotat_z*1000.0)&0xff
        crc_8 = self.crc_byte(outputdata,len(outputdata)-1)
        output += chr(crc_8)
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Vel Command Send Faild")
        self.serialIDLE_flag = 0
    #Subscribe ackermann Cmd call this to send vel cmd to move base
    def ackermannCmdCB(self,data):
        self.speed = data.drive.speed
        self.steering_angle = data.drive.steering_angle
        self.last_ackermann_cmd_time = rospy.Time.now()
        output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x15) + \
            chr((int(self.speed*1000.0)>>8)&0xff) + chr(int(self.speed*1000.0)&0xff) + \
            chr(0x00) + chr(0x00) + \
            chr((int(self.steering_angle*1000.0)>>8)&0xff) + chr(int(self.steering_angle*1000.0)&0xff) + \
            chr(0x00)
        outputdata = [0x5a,0x0c,0x01,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        outputdata[4] = (int(self.speed*1000.0)>>8)&0xff
        outputdata[5] = int(self.speed*1000.0)&0xff
        outputdata[8] = (int(self.steering_angle*1000.0)>>8)&0xff
        outputdata[9] = int(self.steering_angle*1000.0)&0xff
        crc_8 = self.crc_byte(outputdata,len(outputdata)-1)
        output += chr(crc_8)
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Vel Command Send Faild")
        self.serialIDLE_flag = 0
    #Communication Timer callback to handle receive data
    #depend on communication protocol
    def timerCommunicationCB(self,event):
        length = self.serial.in_waiting
        if length:
            reading = self.serial.read_all()
            if len(reading)!=0:
                for i in range(0,len(reading)):
                    data = (int(reading[i].encode('hex'),16)) 
                    try:
                        self.Circleloop.enqueue(data)
                    except:
                        rospy.logerr("Circleloop.enqueue Faild")
        else:
            pass
        if self.Circleloop.is_empty()==False:
            if self.Circleloop.is_empty()==False:
                data = self.Circleloop.get_front()
            else:
                pass
            if data == 0x5a:
                length = self.Circleloop.get_front_second()
                if length > 1 :
                    if self.Circleloop.get_front_second() <= self.Circleloop.get_queue_length():
                        databuf = []
                        for i in range(length):
                            databuf.append(self.Circleloop.get_front())
                            self.Circleloop.dequeue()
                        
                        if (databuf[length-1]) == self.crc_byte(databuf,length-1):
                            pass
                        else:
                            # print databuf
                            # print "Crc check Err %d"%self.crc_byte(databuf,length-1)
                            pass
                        #parse receive data
                        if(databuf[3] == 0x04):
                            self.Vx =    databuf[4]*256
                            self.Vx +=   databuf[5]
                            self.Vy =    databuf[6]*256
                            self.Vy +=   databuf[7]
                            self.Vyaw =  databuf[8]*256
                            self.Vyaw += databuf[9]
                        elif(databuf[3] == 0x06):
                            self.Yawz =  databuf[8]*256
                            self.Yawz += databuf[9]
                        elif (databuf[3] == 0x08):
                            self.Vvoltage = databuf[4]*256
                            self.Vvoltage += databuf[5]
                            self.Icurrent = databuf[6]*256
                            self.Icurrent += databuf[7]
                        elif (databuf[3] == 0x0a):
                            self.Vx =    databuf[4]*256
                            self.Vx +=   databuf[5]
                            self.Yawz =  databuf[6]*256
                            self.Yawz += databuf[7]
                            self.Vyaw =  databuf[8]*256
                            self.Vyaw += databuf[9]
                        elif (databuf[3] == 0x12):
                            self.Vx =    databuf[4]*256
                            self.Vx +=   databuf[5]
                            self.Vy =  databuf[6]*256
                            self.Vy += databuf[7]
                            self.Yawz =  databuf[8]*256
                            self.Yawz += databuf[9]    
                            self.Vyaw =  databuf[10]*256
                            self.Vyaw += databuf[11]                          
                        elif (databuf[3] == 0x14):
                            self.Gyro[0] = int(((databuf[4]&0xff)<<24)|((databuf[5]&0xff)<<16)|((databuf[6]&0xff)<<8)|(databuf[7]&0xff))
                            self.Gyro[1] = int(((databuf[8]&0xff)<<24)|((databuf[9]&0xff)<<16)|((databuf[10]&0xff)<<8)|(databuf[11]&0xff))
                            self.Gyro[2] = int(((databuf[12]&0xff)<<24)|((databuf[13]&0xff)<<16)|((databuf[14]&0xff)<<8)|(databuf[15]&0xff))

                            self.Accel[0] = int(((databuf[16]&0xff)<<24)|((databuf[17]&0xff)<<16)|((databuf[18]&0xff)<<8)|(databuf[19]&0xff))
                            self.Accel[1] = int(((databuf[20]&0xff)<<24)|((databuf[21]&0xff)<<16)|((databuf[22]&0xff)<<8)|(databuf[23]&0xff))
                            self.Accel[2] = int(((databuf[24]&0xff)<<24)|((databuf[25]&0xff)<<16)|((databuf[26]&0xff)<<8)|(databuf[27]&0xff))

                            self.Quat[0] = int((databuf[28]&0xff)<<8|databuf[29])
                            self.Quat[1] = int((databuf[30]&0xff)<<8|databuf[31])
                            self.Quat[2] = int((databuf[32]&0xff)<<8|databuf[33])
                            self.Quat[3] = int((databuf[34]&0xff)<<8|databuf[35])
                        elif(databuf[3] == 0xf2):
                            self.movebase_hardware_version[0] = databuf[4]
                            self.movebase_hardware_version[1] = databuf[5]
                            self.movebase_hardware_version[2] = databuf[6]
                            self.movebase_firmware_version[0] = databuf[7]
                            self.movebase_firmware_version[1] = databuf[8]
                            self.movebase_firmware_version[2] = databuf[9]
                            version_string = "Move Base Hardware Ver %d.%d.%d,Firmware Ver %d.%d.%d"\
                                %(self.movebase_hardware_version[0],self.movebase_hardware_version[1],self.movebase_hardware_version[2],\
                                self.movebase_firmware_version[0],self.movebase_firmware_version[1],self.movebase_firmware_version[2])
                            rospy.loginfo(version_string)
                        elif(databuf[3] == 0xf4):
                            sn_string = "SN:"
                            for i in range(4,16):
                                sn_string = "%s%02x"%(sn_string,databuf[i])
                            rospy.loginfo(sn_string)                            

                        elif(databuf[3] == 0x22):
                            fRatio = float(databuf[6]<<8|databuf[7])/10
                            fDiameter = float(databuf[8]<<8|databuf[9])/10
                            info_string = "Type:%s Motor:%s Ratio:%.01f WheelDiameter:%.01f"\
                                %(self.movebase_type[databuf[4]-1],self.motor_type[databuf[5]-1],fRatio,fDiameter)
                            rospy.loginfo(info_string)
                        else:
                            self.timer_odom.shutdown()
                            self.timer_battery.shutdown()
                            self.timer_imu.shutdown()
                            self.timer_communication.shutdown()
                            rospy.logerr("Invalid Index")
                            rospy.logerr()
                            pass
                else:
                    pass
            else:
                self.Circleloop.dequeue()
        else:
            # rospy.loginfo("Circle is Empty")
            pass
    #get move base hardware & firmware version    
    def getVersion(self):
        #Get version info
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0xf1) + chr(0x00) + chr(0xd7) #0xd7 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Get Version Command Send Faild")
        self.serialIDLE_flag = 0 
    #get move base SN
    def getSN(self):
        #Get version info
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0xf3) + chr(0x00) + chr(0x46) #0x46 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Get SN Command Send Faild")
        self.serialIDLE_flag = 0  
    #get move base info
    def getInfo(self):
        #Get version info
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x21) + chr(0x00) + chr(0x8f) #0x8f is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Get info Command Send Faild")
        self.serialIDLE_flag = 0               
    #Odom Timer call this to get velocity and imu info and convert to odom topic
    def timerOdomCB(self,event):
        #Get move base velocity data
        if self.movebase_firmware_version[1] == 0: 
            #old version firmware have no version info and not support new command below
            output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x09) + chr(0x00) + chr(0x38) #0x38 is CRC-8 value
        else:
            #in firmware version new than v1.1.0,support this command      
            output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x11) + chr(0x00) + chr(0xa2) 
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Odom Command Send Faild")
        self.serialIDLE_flag = 0   
        #calculate odom data
        Vx = float(ctypes.c_int16(self.Vx).value/1000.0)
        Vy = float(ctypes.c_int16(self.Vy).value/1000.0)
        Vyaw = float(ctypes.c_int16(self.Vyaw).value/1000.0)

        self.pose_yaw = float(ctypes.c_int16(self.Yawz).value/100.0)
        self.pose_yaw = self.pose_yaw*math.pi/180.0
  
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.previous_time).to_sec()
        self.previous_time = self.current_time
        self.pose_x = self.pose_x + Vx * (math.cos(self.pose_yaw))*dt - Vy * (math.sin(self.pose_yaw))*dt
        self.pose_y = self.pose_y + Vx * (math.sin(self.pose_yaw))*dt + Vy * (math.cos(self.pose_yaw))*dt

        pose_quat = tf.transformations.quaternion_from_euler(0,0,self.pose_yaw)        
        msg = Odometry()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.odomId
        msg.child_frame_id =self.baseId
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = pose_quat[0]
        msg.pose.pose.orientation.y = pose_quat[1]
        msg.pose.pose.orientation.z = pose_quat[2]
        msg.pose.pose.orientation.w = pose_quat[3]
        msg.twist.twist.linear.x = Vx
        msg.twist.twist.linear.y = Vy
        msg.twist.twist.angular.z = Vyaw
        self.pub.publish(msg)
        self.tf_broadcaster.sendTransform((self.pose_x,self.pose_y,0.0),pose_quat,self.current_time,self.baseId,self.odomId)
    #Battery Timer callback function to get battery info
    def timerBatteryCB(self,event):
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x07) + chr(0x00) + chr(0xe4) #0xe4 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Battery Command Send Faild")
        self.serialIDLE_flag = 0
        msg = BatteryState()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.baseId
        msg.voltage = float(self.Vvoltage/1000.0)
        msg.current = float(self.Icurrent/1000.0)
        self.battery_pub.publish(msg)
    #IMU Timer callback function to get raw imu info
    def timerIMUCB(self,event):
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x13) + chr(0x00) + chr(0x33) #0x33 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Imu Command Send Faild")

        self.serialIDLE_flag = 0
        msg = Imu()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.imuId

        msg.angular_velocity.x = float(ctypes.c_int32(self.Gyro[0]).value/100000.0)
        msg.angular_velocity.y = float(ctypes.c_int32(self.Gyro[1]).value/100000.0)
        msg.angular_velocity.z = float(ctypes.c_int32(self.Gyro[2]).value/100000.0)

        msg.linear_acceleration.x = float(ctypes.c_int32(self.Accel[0]).value/100000.0)
        msg.linear_acceleration.y = float(ctypes.c_int32(self.Accel[1]).value/100000.0)
        msg.linear_acceleration.z = float(ctypes.c_int32(self.Accel[2]).value/100000.0)

        msg.orientation.w = float(ctypes.c_int16(self.Quat[0]).value/10000.0)
        msg.orientation.x = float(ctypes.c_int16(self.Quat[1]).value/10000.0)
        msg.orientation.y = float(ctypes.c_int16(self.Quat[2]).value/10000.0)
        msg.orientation.z = float(ctypes.c_int16(self.Quat[3]).value/10000.0)

        self.imu_pub.publish(msg)  
        # TF value calculate from mechanical structure
        if(base_type == 'NanoRobot'):
            self.tf_broadcaster.sendTransform((-0.062,0.0235,0.08),(0.0,0.0,0.0,1.0),self.current_time,self.imuId,self.baseId)   
        elif(base_type == 'NanoCar'):
            self.tf_broadcaster.sendTransform((-0.056,0.0285,0.09),(0.0,0.0,0.0,1.0),self.current_time,self.imuId,self.baseId) 
        elif(base_type == '4WD'):
            self.tf_broadcaster.sendTransform((-0.065,0.0167,0.02),(0.0,0.0,0.0,1.0),self.current_time,self.imuId,self.baseId)     
        else:
            pass

#main function
if __name__=="__main__":
    try:
        rospy.init_node('base_control',anonymous=True)
        if base_type != None:
            rospy.loginfo('%s base control ...'%base_type)
        else:
            rospy.loginfo('base control ...')
            rospy.logwarn('PLEASE SET BASE_TYPE ENV')

        bc = BaseControl()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")
