# Copyright (c) Microsoft. All rights reserved.
# Licensed under the MIT license.

import iotc
from iotc import IOTConnectType, IOTLogLevel
from random import randint
import httplib, socket, time
import sys
import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist



#Azure IoT Central settings
deviceId = "yourdeviceid"
scopeId = "yourscopeid"
mkey = "yoursaskey"

iotc = iotc.Device(scopeId, mkey, deviceId, IOTConnectType.IOTC_CONNECT_SYMM_KEY)
iotc.setLogLevel(IOTLogLevel.IOTC_LOGGING_API_ONLY)

gCanSend = False
gCounter = 0
sttenabled = False

def sttCallback(data):
    rospy.loginfo(rospy.get_caller_id() + '===I heard %s', data.data)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    cmd = Twist()
    #linear:x: 0.0 y: 0.0 z: 0.0 angular: x: 0.0 y: 0.0 z: -0.282429536481
    global sttenabled
    print(sttenabled)
    if sttenabled:
      if "Forward" in data.data or "forward" in data.data:
          print('Forward')
          cmd.linear.x = 0.215233605
          pub.publish(cmd)

      if "Backward" in data.data or "backward" in data.data:
          print('Backward')
          cmd.linear.x = -0.215233605
          pub.publish(cmd)

      if "Left" in data.data or "left" in data.data:
          print('Left')
          cmd.angular.z = -0.254186582833
          pub.publish(cmd)

      if "Right" in data.data or "right" in data.data:
          print('Right')
          cmd.angular.z = 0.254186582833
          pub.publish(cmd)

	
		
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '===CMD_VEL %s', data)

    #pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    #cmd = Twist()
    iotc.sendTelemetry("{ \
    \"linear-x\": " + str(data.linear.x) +" , \
    \"linear-y\": " + str(data.linear.y) +" , \
    \"linear-z\": " + str(data.linear.z) +" , \
    \"angular-x\": " + str(data.angular.x) +" , \
    \"angular-y\": " + str(data.angular.y) +" , \
    \"angular-z\": " + str(data.angular.z) +" }")

    #rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        #rate.sleep()
    #linear:x: 0.0 y: 0.0 z: 0.0 angular: x: 0.0 y: 0.0 z: -0.282429536481
    #if "pinky" in data.data or "Pinky" in data.data:
    #    print('momo')
    #    cmd.angular.z = -0.282429536481
    #    pub.publish(cmd)

    #if "yellow" in data.data:
    #    print('qiaohu')
    #    cmd.angular.z = 0.282429536481
    #    pub.publish(cmd)



    #if "Hi" in data.data or "Who are" in data.data or "What's your name" in data.data:
    #    print('hello,im mebo2 robot')
    #    pub.publish('hello_robot')



def onconnect(info):
  global gCanSend
  print("- [onconnect] => status:" + str(info.getStatusCode()))
  if info.getStatusCode() == 0:
     if iotc.isConnected():
       gCanSend = True
       rospy.init_node('botctrl', anonymous=True)
       rospy.Subscriber('cmd_vel', Twist, callback)
       rospy.Subscriber('stt', String, sttCallback)
       # spin() simply keeps python from exiting until this node is stopped
       rospy.spin()

def onmessagesent(info):
  print("\t- [onmessagesent] => " + str(info.getPayload()))

def oncommand(info):
  print("- [oncommand] => " + info.getTag() + " => " + str(info.getPayload()))
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  cmd = Twist()
  #handle_mebo_command(info.getTag())
  if info.getTag()=='left':
    cmd.angular.z = -0.254186582833
    pub.publish(cmd)
  if info.getTag()=='right':
    cmd.angular.z = 0.254186582833
    pub.publish(cmd)
  if info.getTag()=='forward':
    cmd.linear.x = 0.215233605
    pub.publish(cmd)
  if info.getTag()=='backward':
    cmd.linear.x = -0.215233605
    pub.publish(cmd)
  # if info.getTag()=='lightoff':
  #   handle_mebo_command("LN")


def onsettingsupdated(info):
  global sttenabled
  print("- [onsettingsupdated] => " + info.getTag() + " => " + info.getPayload())
  infoobj = json.loads(info.getPayload())
  print('device settings=='+str(infoobj['value']))
  sttenabled = infoobj['value']


  #if info.getTag() =='COMMAND_DURATION':
  #  mebo_constants.COMMAND_DURATION=infoobj['value']

iotc.on("ConnectionStatus", onconnect)
iotc.on("MessageSent", onmessagesent)
iotc.on("Command", oncommand)
iotc.on("SettingsUpdated", onsettingsupdated)

iotc.connect()


