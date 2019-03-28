#!/usr/bin/python
# coding=gbk
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

import Queue
import ctypes

class queue:
    def __init__(self, capacity = 100):
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

class BaseControl:
    def __init__(self):
        #Get params
        self.baseId = rospy.get_param('~base_id','base_footprint')
        self.odomId = rospy.get_param('~odom_id','odom')
        self.device_port = rospy.get_param('~port','/dev/ttyUSB0')
        self.baudrate = int(rospy.get_param('~baudrate','115200'))
        self.odom_freq = int(rospy.get_param('~odom_freq','25'))
        self.odom_topic = rospy.get_param('~odom_topic','/odom')
        self.battery_topic = rospy.get_param('~battery_topic','battery')
        self.battery_freq = float(rospy.get_param('~battery_freq','0.1'))
        #define param
        self.current_time = rospy.Time.now()
        self.previous_time = self.current_time
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.serialIDLE_flag = 0
        self.trans_x = 0.0
        self.rotat_z = 0.0
        self.sendcounter = 0
        self.ImuErrFlag = False
        self.EncoderFlag = False
        self.BatteryFlag = False
        self.OdomTimeCounter = 0
        self.BatteryTimeCounter = 0
        self.Circleloop = queue(capacity = 500)
        self.Vx =0
        self.Vyaw = 0
        self.Yawz = 0
        self.Vvoltage = 0
        self.Icurrent = 0
        self.last_cmd_vel_time = rospy.Time.now()

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
        # self.timer_cmd = rospy.Timer(rospy.Duration(1.0/self.odom_freq),self.timerCmdCB)
        self.timer_communication = rospy.Timer(rospy.Duration(1.0/1000),self.timerCommunicationCB)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.serialQ = Queue.Queue(200)
        # self.serial.flush()
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
        self.rotat_z = data.angular.z
        self.last_cmd_vel_time = rospy.Time.now()
        output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x01) + \
            chr((int(self.trans_x*1000.0)>>8)&0xff) + chr(int(self.trans_x*1000.0)&0xff) + \
            chr(0x00) + chr(0x00) + \
            chr((int(self.rotat_z*1000.0)>>8)&0xff) + chr(int(self.rotat_z*1000.0)&0xff) + \
            chr(0x00)
        outputdata = [0x5a,0x0c,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        outputdata[4] = (int(self.trans_x*1000.0)>>8)&0xff
        outputdata[5] = int(self.trans_x*1000.0)&0xff
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
    #
    def timerCommunicationCB(self,event):
        length = self.serial.in_waiting
        if length:
            reading = self.serial.read_all()
            if len(reading)!=0:
                # print "Recv Len:%d--"%len(reading),
                for i in range(0,len(reading)):
                    data = (int(reading[i].encode('hex'),16)) 
                    # print " %x"%data,
                    try:
                        self.Circleloop.enqueue(data)
                    except:
                        rospy.logerr("Circleloop.enqueue Faild")
                # print ''
                # self.Circleloop.show_queue()
                # print self.Circleloop.front,self.Circleloop.rear
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
                            print databuf
                            print "Crc check Err %d"%self.crc_byte(databuf,length-1)
                            pass
                        # self.serialQ.put(databuf)
                        if(databuf[3] == 0x04):
                            self.Vx =    databuf[4]*256
                            self.Vx +=   databuf[5]
                            self.Vyaw =  databuf[8]*256
                            self.Vyaw += databuf[9]
                            print "Vx %x"%self.Vx
                            print "Vyaw %x"%self.Vyaw
                        elif(databuf[3] == 0x06):
                            self.Yawz =  databuf[8]*256
                            self.Yawz += databuf[9]
                            print "Yawz %x"%self.Yawz
                        elif (databuf[3] == 0x08):
                            self.Vvoltage = databuf[4]*256
                            self.Vvoltage += databuf[5]
                            self.Icurrent = databuf[6]*256
                            self.Icurrent += databuf[7]
                            # print ctypes.c_int16(self.Vvoltage),ctypes.c_int16(self.Icurrent)
                        elif (databuf[3] == 0x0a):
                            self.Vx =    databuf[4]*256
                            self.Vx +=   databuf[5]
                            self.Yawz =  databuf[6]*256
                            self.Yawz += databuf[7]
                            self.Vyaw =  databuf[8]*256
                            self.Vyaw += databuf[9]
                            # print "Vx %x"%self.Vx
                            # print "Yawz %x"%self.Yawz
                            # print "Vyaw %x"%self.Vyaw
                        else:
                            self.timer_odom.shutdown()
                            self.timer_battery.shutdown()
                            self.timer_cmd.shutdown()
                            rospy.logerr("Invalid Index")
                            rospy.logerr()
                            pass
                else:
                    print "CircleLoop Length %d"%length
                    # self.Circleloop.dequeue()
            else:
                print "Data:%x"%data
                self.Circleloop.dequeue()
        else:
            # rospy.loginfo("Circle is Empty")
            pass
    #                   
    #Odom Timer call this to get velocity and imu info and convert to odom topic
    def timerOdomCB(self,event):
        #Get move base velocity data
        # rospy.loginfo("Send Encoder Cmd")
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x09) + chr(0x00) + chr(0x38) #0x38 is CRC-8 value
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
        # rospy.loginfo("Send Imu Cmd")
        # output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x05) + chr(0x00) + chr(0x00)
        # while(self.serialIDLE_flag):
        #     time.sleep(0.01)
        # self.serialIDLE_flag = 2
        # while self.serial.out_waiting:
        #     pass
        # self.serial.write(output)
        # self.serialIDLE_flag = 0 
        #calculate odom data
        Vx = float(ctypes.c_int16(self.Vx).value/1000.0)
        Vyaw = float(ctypes.c_int16(self.Vyaw).value/1000.0)
        # print "Yawz------------ %d"%ctypes.c_int16(self.Yawz).value
        self.pose_yaw = float(ctypes.c_int16(self.Yawz).value/100.0)
        self.pose_yaw = self.pose_yaw*math.pi/180.0
        # print "pose_yaw------------ %f"%pose_yaw
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.previous_time).to_sec()
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
        msg.twist.twist.angular.z = Vyaw
        self.pub.publish(msg)
        self.tf_broadcaster.sendTransform((self.pose_x,self.pose_y,0.0),pose_quat,self.current_time,self.baseId,self.odomId)
    #
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
        # rospy.loginfo("Get Battery Info")

        self.serialIDLE_flag = 0
        msg = BatteryState()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.baseId
        msg.voltage = float(self.Vvoltage/1000.0)
        msg.current = float(self.Icurrent/1000.0)
        self.baudrate_pub.publish(msg)
    #timerCmdCB callback function TODO:delete
    def timerCmdCB(self,ecent):
        if(rospy.Time.now()-self.last_cmd_vel_time).to_sec > 0.2:
            self.trans_x = 0
            self.rotat_z = 0
        else:
            pass
        trans_x = self.trans_x
        rotat_z = self.rotat_z
        output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x01) + \
            chr((int(trans_x*1000.0)>>8)&0xff) + chr(int(trans_x*1000.0)&0xff) + \
            chr(0x00) + chr(0x00) + \
            chr((int(rotat_z*1000.0)>>8)&0xff) + chr(int(rotat_z*1000.0)&0xff) + \
            chr(0x00)
        outputdata = [0x5a,0x0c,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        outputdata[4] = (int(trans_x*1000.0)>>8)&0xff
        outputdata[5] = int(trans_x*1000.0)&0xff
        outputdata[8] = (int(rotat_z*1000.0)>>8)&0xff
        outputdata[9] = int(rotat_z*1000.0)&0xff
        crc_8 = self.crc_byte(outputdata,len(outputdata)-1)
        # print "CRC-8 = %02x"%crc_8
        output += chr(crc_8)
        while self.serialIDLE_flag:
            # time.sleep(1)
            # rospy.loginfo("CmdCB Serial Busy %d"%self.serialIDLE_flag)
            time.sleep(0.01)
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Vel Command Send Faild")
        self.serialIDLE_flag = 0
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
