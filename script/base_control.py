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

class BaseControl:
    serialIDLE_flag = True
    def __init__(self):
        #Get params
        self.baseId = rospy.get_param('~base_id','base_footprint')
        self.odomId = rospy.get_param('~odom_id','odom')
        self.device_port = rospy.get_param('~port','/dev/ttyUSB1')
        self.baudrate = int(rospy.get_param('~baudrate','115200'))
        self.odom_freq = int(rospy.get_param('~odom_freq','1'))
        self.odom_topic = rospy.get_param('~odom_topic','/odom')
        # Serial Communication
        try:
            self.serial = serial.Serial(self.device_port,self.baudrate,timeout=10)
            rospy.loginfo("Opening Serial")
            # try:
            #     self.serial.read()
            # except:
            #     pass
            # for x in range(0,50):
            #     data = self.serial.read()
            #     time.sleep(0.01)
        except:
            rospy.logerr("Can not open Serial"+self.device_port)
            self.serial.close
            sys.exit(0)
        rospy.loginfo("Serial Open Succeed")

        self.sub = rospy.Subscriber("cmd_vel",Twist,self.cmdCB,queue_size=20)
        self.pub = rospy.Publisher(self.odom_topic,Odometry,queue_size=10)
        self.timer_odom = rospy.Timer(rospy.Duration(1/self.odom_freq),self.timerOdomCB)
        # self.timer_cmd = rospy.Timer(rospy.Duration(1),self.timerCmdCB)  #20Hz
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.current_time = rospy.Time.now()
        self.previous_time = self.current_time
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0

        while True:
            reading = self.serial.read_all()
            if reading != '':
                if((int(reading[0].encode('hex'),16)== 0x5a) & (int(reading[1].encode('hex'),16)==len(reading))):
                    self.data = reading
                else:
                    rospy.logerr("Invalid Data")
                    pass
            else:
                pass
    def cmdCB(self,data):
        self.trans_x = data.linear.x
        self.rotat_z = data.angular.z
        print("cmdCB")
        print(self.trans_x,self.rotat_z)

        while self.serialIDLE_flag==False:
            # time.sleep(1)
            rospy.loginfo("CmdCB Serial Busy")
            time.sleep(0.05)
        self.serialIDLE_flag = False
        # rospy.logdebug()
        output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x01) + \
            chr((int(self.trans_x*1000.0)>>8)&0xff) + chr(int(self.trans_x*1000.0)&0xff) + \
            chr(0x00) + chr(0x00) + \
            chr((int(self.rotat_z*1000.0)>>8)&0xff) + chr(int(self.rotat_z*1000.0)&0xff) + \
            chr(0x00) + chr(0x00)
        int(self.trans_x*1000.0)>>8
        self.serial.write(output)
        rospy.loginfo("Send Vel Cmd")
        # self.serial.read_all()
        self.serialIDLE_flag = True
        
    
    def timerOdomCB(self,event):
        #Serial read & write
        rospy.loginfo("Send Encoder Cmd")
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x03) + chr(0x00) + chr(0x00)
        while(self.serialIDLE_flag == False):
            rospy.loginfo("OdomCB Serial Busy")
            time.sleep(0.05)
        self.serialIDLE_flag = False
        self.serial.write(output)
        self.serialIDLE_flag = True   
        try:
            data = self.data
            #Normal mode
            if(len(data) == int(data[1].encode('hex'),16)):
                print len(data)
                rospy.loginfo("Valid Data")
                # Vx = float(int(int(data[4].encode('hex'),16)<<8 + int(data[5].encode('hex'),16),16))
                # Vx =int( (int(data[4].encode('hex'),8)<<8 + int(data[5].encode('hex'),16))&0xffff,16)
                Vx = int(data[4].encode('hex'),8)*256
                Vx += int(data[5].encode('hex'),16)
                
                
            else:
                rospy.logerr("Invalid Data")
            self.current_time = rospy.Time.now()
            dt = (self.previous_time - self.current_time).to_sec
            self.previous_time = self.current_time
        except :
            # rospy.logerr("Try Faild")
            # print data + str(len(data))
            pass
        
    def timerCmdCB(self,event):
        #Send Velocity to move base
        while self.serialIDLE_flag==False:
            # time.sleep(1)
            rospy.loginfo("CmdCB Serial Busy")
            time.sleep(0.05)
        self.serialIDLE_flag = False
        # rospy.logdebug()
        output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x01) + chr(0x00) + chr(0x00) + chr(0x00) + chr(0x00) + chr(0x00) + chr(0x00) + chr(0x00) + chr(0x00)
        self.serial.write(output)
        rospy.loginfo("Send Vel Cmd")
        # self.serial.read_all()
        self.serialIDLE_flag = True

if __name__=="__main__":
    try:
        rospy.init_node('base_control',anonymous=True)
        rospy.loginfo('Nano base control ...')
        bc = BaseControl()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")
