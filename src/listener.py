#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import time
import threading
import datetime
import os
import requests

import sys
import numpy as np
from PIL import Image
import json
#from object_detection import ObjectDetection

#ROS import
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


MODEL_FILENAME = 'model.pb'
LABELS_FILENAME = 'labels.txt'

clawTarget = 'momo'


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    global clawTarget
    if "pinky" in data.data or "Pinky" in data.data:
        os.system('d:\\ffmpeg-4.0.2-win64-static\\bin\\ffmpeg -i okgotit-short.wav -vol 1000 -f alaw -ar 8000 udp://192.168.99.1:8828?connect=1')
        print('momo')
        clawTarget = 'momo'

    if "yellow" in data.data:
		print('qiaohu')
		clawTarget = 'qiaohu'
    if "stop" in data.data:
		print('stop')
		clawTarget = ''
    if "Hi" in data.data or "Who are" in data.data or "What's your name" in data.data:
        print('hello,im mebo2 robot')
        os.system('d:\\ffmpeg-4.0.2-win64-static\\bin\\ffmpeg -i immebo.wav -vol 1000 -f alaw -ar 8000 udp://192.168.99.1:8828?connect=1')
		
		
		
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('stt', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spinOnce()

class TFObjectDetection(ObjectDetection):
    """Object Detection class for TensorFlow
    """
    def __init__(self, graph_def, labels):
        super(TFObjectDetection, self).__init__(labels)
        self.graph = tf.Graph()
        with self.graph.as_default():
            tf.import_graph_def(graph_def, name='')
            
    def predict(self, preprocessed_image):
        inputs = np.array(preprocessed_image, dtype=np.float)[:,:,(2,1,0)] # RGB -> BGR

        with tf.Session(graph=self.graph) as sess:
            output_tensor = sess.graph.get_tensor_by_name('model_outputs:0')
            outputs = sess.run(output_tensor, {'Placeholder:0': inputs[np.newaxis,...]})
            return outputs[0]

class ipcamCapture:
    def __init__(self, URL):
        self.Frame = []
        self.status = False
        self.isstop = False
		
        self.capture = cv2.VideoCapture(URL)

    def start(self):
        print('ipcam started!')
        threading.Thread(target=self.queryframe, args=()).start()
        listener()

    def stop(self):
        self.isstop = True
        print('ipcam stopped!')
   
    def getframe(self):
        return self.Frame
        
    def queryframe(self):
        while (not self.isstop):
            self.status, self.Frame = self.capture.read()
        
        self.capture.release()

def __sendFrameForProcessing(frame):
        headers = {'Content-Type': 'application/octet-stream'}
        response = requests.post("http://127.0.0.1/image", headers = headers, params = "", data = frame)
        try:
            print("Response from external processing service: (" + str(response.status_code) + ") " + json.dumps(response.json()))
        except Exception:
            print("Response from external processing service (status code): " + str(response.status_code))
        try:
            return response.json()
        except:
            print('error')
            return ""

#URL = "rtsp://stream:video@192.168.99.1:554/media/stream2"
URL = 0
# 連接攝影機
ipcam = ipcamCapture(URL)
ipcam.start()
# 暫停1秒，確保影像已經填充
time.sleep(1)

while True:
    # 使用 getframe 取得最新的影像
    I = ipcam.getframe()
    if np.shape(I) == ():
        continue

    # elif:
    try:
        image = cv2.resize(I,(512, 512))
    except cv2.error as e:
        print('ignore error')
        print(e)
    (h,w) = image.shape[:2]

    # capture training image file
    #suffix = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
    #filename="_".join(["toy",suffix])
    #cv2.imwrite(filename+".jpg",image)

    encodedFrame = cv2.imencode(".jpg", image)[1].tostring()
                                # Send over HTTP for processing
    response = __sendFrameForProcessing(encodedFrame)
    if len(response['predictions']) >0 :
        print(response['predictions'][0]['boundingBox']['top'])                            
        # task = loop.create_task(self.__sendFrameForProcessing( encodedFrame ))
        # tasks.append(task)
        # loop.run_until_complete(asyncio.wait(tasks))
                                    
        
        for i in range(len(response['predictions'])):
            if response['predictions'][i]['tagName'] == clawTarget or clawTarget=='' :
                x_center = response['predictions'][i]['boundingBox']['left'] * w
                y_center = response['predictions'][i]['boundingBox']['top'] * h
                width_box = response['predictions'][i]['boundingBox']['width'] * w
                height_box = response['predictions'][i]['boundingBox']['height'] * h

                x1 = int( x_center)
                y1 = int( y_center)
                x2 = int( x_center + width_box)
                y2 = int( y_center + height_box)
                x0 = int( x_center + width_box * 0.5 )
                y0=  int( y_center + height_box * 0.5)
                print(x1,y1,x2,y2)
                framecolor = (255, 0, 0)
                #print(labels[selected_classes[i]])
                
                #print(clawTarget)        
                cv2.rectangle( image, (x1, y1), (x2, y2), framecolor, 2 )
                cv2.putText( image, response['predictions'][i]['tagName'] + " " + "{0:.1f}".format( response['predictions'][i]['probability'] ),
                                                            (x1, y1),
                                                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA )
                cv2.circle(image,(x0, y0), 3, (0, 255, 0), -1)
                cv2.putText( image, str(x0)+','+str(y0),
                                                            (x0, y0),
                                                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA )
            if clawTarget !='' and response['predictions'][i]['probability'] > 0.5:
            # Path Finder based on target (x=160-240,y=160-240)...
                    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
                    cmd = Twist()
                    if x0 >= 210 and x0 <= 285:
                        #if y0 >= 275 and y0 <=310:
                        #    print('claw')
                        if y0 <=410:
                            print('forward')
                            cmd.linear.x = 0.255233605
                            pub.publish(cmd)
                        elif y0 > 395:
                            print('backward')
                            cmd.linear.x = -0.255233605
                            pub.publish(cmd)
                    elif x0 < 210:
                        print('Left')
                        cmd.angular.z = 0.154186582833
                        pub.publish(cmd)
                    elif x0 > 285:
                        print('Right')
                        cmd.angular.z = -0.154186582833
                        pub.publish(cmd)
            
    cv2.imshow('Image', image)
    if cv2.waitKey(100) == 27:
        cv2.destroyAllWindows()
        ipcam.stop()
        break
