#!/usr/bin/python
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

import ctypes
class BaseControl:
    def __init__(self):
        #Get params
        self.baseId = rospy.get_param('~base_id','base_footprint')
        self.odomId = rospy.get_param('~odom_id','odom')
        self.device_port = rospy.get_param('~port','/dev/ttyUSB1')
        self.baudrate = int(rospy.get_param('~baudrate','115200'))
        self.odom_freq = int(rospy.get_param('~odom_freq','50'))
        self.odom_topic = rospy.get_param('~odom_topic','/odom')
        self.battery_topic = rospy.get_param('~battery_topic','battery')
        self.battery_freq = float(rospy.get_param('~battery_freq','0.1'))
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

        self.sub = rospy.Subscriber("cmd_vel",Twist,self.cmdCB,queue_size=20)
        self.pub = rospy.Publisher(self.odom_topic,Odometry,queue_size=10)
        self.baudrate_pub = rospy.Publisher(self.battery_topic,BatteryState,queue_size=3)

        self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.odom_freq),self.timerOdomCB)
        self.timer_battery = rospy.Timer(rospy.Duration(1.0/self.battery_freq),self.timerBatteryCB)  #20Hz
        self.timer_cmd = rospy.Timer(rospy.Duration(2.0/self.odom_freq),self.timerCmdCB)
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.current_time = rospy.Time.now()
        self.previous_time = self.current_time
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.serialIDLE_flag = True
        self.trans_x = 0.0
        self.rotat_z = 0.0
    #Subscribe vel_cmd call this to send vel cmd to move base
    def cmdCB(self,data):
        self.trans_x = data.linear.x
        self.rotat_z = data.angular.z
        # print("cmdCB")

        # while self.serialIDLE_flag==False:
        #     # time.sleep(1)
        #     rospy.loginfo("CmdCB Serial Busy")
        #     time.sleep(0.01)
        # output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x01) + \
        #     chr((int(self.trans_x*1000.0)>>8)&0xff) + chr(int(self.trans_x*1000.0)&0xff) + \
        #     chr(0x00) + chr(0x00) + \
        #     chr((int(self.rotat_z*1000.0)>>8)&0xff) + chr(int(self.rotat_z*1000.0)&0xff) + \
        #     chr(0x00) + chr(0x00)
        # # rospy.logdebug()

        # self.serialIDLE_flag = False
        # try:
        #     while self.serial.out_waiting:
        #         pass
        #     self.serial.write(output)
        # except:
        #     rospy.logerr("Vel Command Send Faild")
        # self.serialIDLE_flag = True
        # rospy.loginfo("Send Vel Cmd")
        # # self.serial.read_all()

        
    #Odom Timer call this to get velocity and imu info and convert to odom topic
    def timerOdomCB(self,event):
        #Get move base velocity data
        # rospy.loginfo("Send Encoder Cmd")
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x03) + chr(0x00) + chr(0x00)
        while(self.serialIDLE_flag == False):
            # rospy.loginfo("OdomCB Serial Busy")
            time.sleep(0.01)
        self.serialIDLE_flag = False
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Odom Command Send Faild")
        # self.serialIDLE_flag = True   
        try:
            # data = self.data
            while self.serial.in_waiting != 12:
                pass
            data = self.serial.read(12)
            self.serialIDLE_flag = True
            #Normal mode
            if(int(data[3].encode('hex'),16) == 0x04):
                Vx = int(data[4].encode('hex'),16)*256
                Vx += int(data[5].encode('hex'),16)
                Vywa = int(data[8].encode('hex'),16)*256
                Vywa += int(data[9].encode('hex'),16)
                # rospy.loginfo("Valid Data")
            else:
                print len(data),int(data[3].encode('hex'),8)
                rospy.logerr("Invalid Encoder Data")
            # self.current_time = rospy.Time.now()
            # dt = (self.previous_time - self.current_time).to_sec
            # self.previous_time = self.current_time
        except :
            rospy.logerr("Encoder Try Faild")
            # print data + str(len(data))
            pass
        Vx = float(ctypes.c_int16(Vx).value/1000.0)
        Vywa = float(ctypes.c_int16(Vywa).value/1000.0)
        # rospy.loginfo("Send Imu Cmd")
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x05) + chr(0x00) + chr(0x00)
        while(self.serialIDLE_flag == False):
            # rospy.loginfo("OdomCB Serial Busy")
            time.sleep(0.01)
        self.serialIDLE_flag = False
        while self.serial.out_waiting:
            pass
        self.serial.write(output)
 
        try:
            while self.serial.in_waiting != 12:
                pass
            data = self.serial.read(12)
            self.serialIDLE_flag = True 
            #Normal mode
            if(int(data[3].encode('hex'),16) == 0x06):
                Yawz = int(data[8].encode('hex'),16)*256
                Yawz += int(data[9].encode('hex'),16)
                # print int(Yawz,16)
                # rospy.loginfo("Valid Data")
            else:
                # print len(data),int(data[3].encode('hex'),8)
                rospy.logerr("Invalid Imu Data")

        except :
            rospy.logerr("IMU Try Faild")
            # print data + str(len(data))
            pass
        self.pose_yaw = float(ctypes.c_int16(Yawz).value/100.0)
        self.pose_yaw = self.pose_yaw*math.pi/180.0
        self.current_time = rospy.Time.now()
        dt = (self.previous_time - self.current_time).to_sec()
        self.previous_time = self.current_time

        self.pose_x = self.pose_x + Vx * (math.cos(self.pose_yaw))*dt
        self.pose_y = self.pose_y + Vx * (math.sin(self.pose_yaw))*dt

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
        msg.twist.twist.angular.z = Vywa
        self.pub.publish(msg)

        self.tf_broadcaster.sendTransform((self.pose_x,self.pose_y,0.0),pose_quat,self.current_time,self.baseId,self.odomId)
    #Battery Timer callback function to get battery info
    def timerBatteryCB(self,event):
        #Send Velocity to move base
        # while self.serialIDLE_flag==False:
        #     # time.sleep(1)
        
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x07) + chr(0x00) + chr(0x00)
        while(self.serialIDLE_flag == False):
            # rospy.loginfo("BatteryCB Serial Busy")
            time.sleep(0.01)
        self.serialIDLE_flag = False
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Battery Command Send Faild")
        rospy.loginfo("Get Battery Info")
        try:
            # data = self.data
            while self.serial.in_waiting != 10:
                pass
            data = self.serial.read(10)
            self.serialIDLE_flag = True
            #Normal mode
            if(int(data[3].encode('hex'),16) == 0x08):
                Vvoltage = int(data[4].encode('hex'),16)*256
                Vvoltage += int(data[5].encode('hex'),16)
                Icurrent = int(data[6].encode('hex'),16)*256
                Icurrent += int(data[7].encode('hex'),16)
                # rospy.loginfo("Valid Data")
            else:
                print len(data),int(data[3].encode('hex'),8)
                rospy.logerr("Invalid Encoder Data")
            # self.current_time = rospy.Time.now()
            # dt = (self.previous_time - self.current_time).to_sec
            # self.previous_time = self.current_time
        except :
            rospy.logerr("Encoder Try Faild")
            # print data + str(len(data))
            pass
        msg = BatteryState()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.baseId
        msg.voltage = float(Vvoltage/1000.0)
        msg.current = float(Icurrent/1000.0)
        self.baudrate_pub.publish(msg)
        #     time.sleep(0.05)
        # self.serialIDLE_flag = False
        # # rospy.logdebug()
        # output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x01) + chr(0x00) + chr(0x00) + chr(0x00) + chr(0x00) + chr(0x00) + chr(0x00) + chr(0x00) + chr(0x00)
        # self.serial.write(output)
        # rospy.loginfo("Send Vel Cmd")
        # # self.serial.read_all()
        # self.serialIDLE_flag = True
    #timerCmdCB callback function
    def timerCmdCB(self,ecent):
        trans_x = self.trans_x
        rotat_z = self.rotat_z
        output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x01) + \
            chr((int(trans_x*1000.0)>>8)&0xff) + chr(int(trans_x*1000.0)&0xff) + \
            chr(0x00) + chr(0x00) + \
            chr((int(rotat_z*1000.0)>>8)&0xff) + chr(int(rotat_z*1000.0)&0xff) + \
            chr(0x00) + chr(0x00)
        while self.serialIDLE_flag==False:
            # time.sleep(1)
            # rospy.loginfo("CmdCB Serial Busy")
            time.sleep(0.01)
        self.serialIDLE_flag = False
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Vel Command Send Faild")
        self.serialIDLE_flag = True
        # rospy.loginfo("Send Vel Cmd")

if __name__=="__main__":
    try:
        rospy.init_node('base_control',anonymous=True)
        rospy.loginfo('Nano base control ...')
        bc = BaseControl()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")
