#!/usr/bin/env python3
# Switcher.py
# By Andrew Grindstaff '21
# Maintained by Alex Bertran '24 (06/27/22 - Present)
# Rewritten by James Randall '24

# Cameras run a script that streams raspivid to 192.168.1.100:5000 (Topside IP)
# This program runs a flask app that listens on 192.168.1.100:12345
# When cameras start up, they ping this IP address. This program detects the IP that pings that address
  # and adds it to a list of known camera IPs.


import cv2
import numpy as np       
import time
import threading
import flask
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Float32, Int32

from sensor_msgs.msg import Joy, Image
from camera_view.msg import camData 
from copilot_interface.msg import controlData
from copilot_interface.msg import autoControlData
from rov_control.msg import autoDock
from rov_control.msg import thrusterPercents

# Class for holding all the camera logic. Switches and reads the camera, adding an overlay to it.
class CameraSwitcher:

    def __init__(self):
        # self.verified is a dictionary that has keys of camera numbers and values of ip addresses -
        # self.verified = { 1: "192.168.1.101", 2: "192.168.1.102" }
        self.verified = {}
        # self.num is an integer that represents the current camera number and is the key for self.verified
        self.num = 0
        self.change = False
        self.cap = None

        # targeting color values
        self.camera_data = camData()
        self.image = None
        self.bridge = CvBridge()
        
        self.lower_red_first = np.array([0,70,10])
        self.upper_red_first = np.array([10,255,255])
        self.lower_red_second = np.array([170,70,10])
        self.upper_red_second = np.array([180,255,255])
        self.lower_white = np.array([0,3,240])
        self.upper_white = np.array([255,5,255])
        # enable auto dock
        self.auto_dock = False

        # Create Subscribers
        self.auto_control_sub = rospy.Subscriber('/auto_control', autoControlData, self.enable_auto_dock)
        self.camera_sub = rospy.Subscriber('joystick', Joy, self.change_camera_callback)
        self.image_pub = rospy.Publisher('screenshots', Image, queue_size = 1)
        self.depth_sub = rospy.Subscriber('rov/depth_sensor', Float32, self.change_depth_callback)
        self.auto_dock_pub = rospy.Publisher('auto_dock_data', autoDock, queue_size = 3)
        self.throttle_sub = rospy.Subscriber('thrusters', thrusterPercents, self.throttle_callback)
        
        self.depth = 0
        self.target_depth = 0
        self.dh_enable = False
        
        # Auto Docking Data (for publishing)
        self.ad_msg = autoDock()
        self.ad_msg.ad_x_error = 0
        self.ad_msg.ad_y_error = 0
        self.ad_msg.ad_proximity = 0
        
        self.rov_throttle = 0

        self.config = {}

        # Initialize multithreading
        self.camera_thread = threading.Thread(target=self.find_cameras)
        self.camera_thread.setDaemon(True)
        self.camera_thread.start()
        
    @property
    def ip(self):
        #"""Ensures that the IP of the camera is always the correct number
        #without sacraficing redability. Otherwise, returns False
        #"""
        try:
            return self.verified[self.num]
        except KeyError:
            rospy.logerr("camera_viewer: passed a camera number that doesn't exist")
            if len(self.verified.keys()) == 0:
                return ""
            return self.verified[list(self.verified.keys())[0]]
    
    # Depth Bar Overlay Code
    def depth_calibration(self):
        return (self.depth * 3.281) + 1.95
        
    def depth_bar(self, frame, depthLevel):
        # Preparing the bar
        cv2.line(frame, (1240, 128), (1240, 640), (48, 18, 196), 5)
        cv2.line(frame, (1240, 128), (1190, 128), (48, 18, 196), 5)
        cv2.line(frame, (1240, 640), (1190, 640), (48, 18, 196), 5)

        # Intervals (32 pixels)
        for i in range(16):
            if i % 2 == 0:
                cv2.line(frame, (1240, 608 - (i * 32)), (1215, 608 - (i * 32)), (48, 18, 196), 5)
            else:
                cv2.line(frame, (1240, 608 - (i * 32)), (1190, 608 - (i * 32)), (48, 18, 196), 5)

        # Draw target depth
        if self.dh_enable:
            cv2.line(frame, (1190, int(self.target_depth * 32) + 128), (1240, int(self.target_depth * 32) + 128), (255,0,0), 5)
            cv2.putText(frame, "Target", (1085, int(self.target_depth * 32) + 135), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)


        # Draw pointer
        pt1 = (1240, (depthLevel * 32) + 128)
        pt2 = (1190, (depthLevel * 32) + 153)
        pt3 = (1190, (depthLevel * 32) + 103)

        pointer = np.array([pt1, pt2, pt3])
        cv2.drawContours(frame, [pointer.astype(int)], 0, (19,185,253), -1)

        return frame
    
    def throttle_bar(self, frame):
      tempThrottle = abs(self.rov_throttle + 1000)
      
      cv2.line(frame, (40,128), (40, 640), (48, 18, 196), 5)
      cv2.line(frame, (40,128), (90, 128), (48, 18, 196), 5)
      cv2.line(frame, (40,640), (90, 640), (48, 18, 196), 5)
      cv2.line(frame, (40,384), (90, 384), (48, 18, 196), 5)
      
      pt1 = (40, int(tempThrottle * 0.256) + 128)
      pt2 = (90, int(tempThrottle * 0.256) + 153)
      pt3 = (90, int(tempThrottle * 0.256) + 103)
      pointer = np.array([pt1, pt2, pt3])
      
      if not self.dh_enable:
        cv2.drawContours(frame, [pointer.astype(int)], 0, (19,185,253), -1)
      else:
        cv2.drawContours(frame, [pointer.astype(int)], 0, (255,0,0), -1)
      return frame
    
    def docking_targeting(self, frame):
        inverted = cv2.bitwise_not(frame)
        hsv_white_frame = cv2.cvtColor(inverted, cv2.COLOR_BGR2HSV)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red1 = cv2.inRange(hsv_frame, self.lower_red_first, self.upper_red_first)
        mask_red2 = cv2.inRange(hsv_frame, self.lower_red_second, self.upper_red_second)
        mask_white = cv2.inRange(hsv_white_frame, self.lower_white, self.upper_white)
        mask_combo = mask_red1 + mask_red2 + mask_white
        contours, heirarchy = cv2.findContours(mask_combo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        biggestC = 0
        biggestX = 0
        biggestY = 0
        for c in contours:
          if cv2.contourArea(c) > 1:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            if cX >= 320 and cX <= 960 and cY >= 180 and cY <= 540:
                if cv2.contourArea(c) > biggestC:
                    biggestC = cv2.contourArea(c)
                    biggestX = cX
                    biggestY = cY
        if biggestC > 0:
          # Proximity calculated with line of best fit from multiple data points (area vs distance)
          self.ad_msg.ad_proximity = int(((261267000 / ((1000 * int(biggestC)) + 376387)) ** (7235 / 12894)) * 2.54)
          self.ad_msg.ad_x_error = -(int((640 - biggestX) * (self.ad_msg.ad_proximity / 100)))
          self.ad_msg.ad_y_error = int((360 - biggestY) * (self.ad_msg.ad_proximity / 100))
          
          cv2.circle(frame, (biggestX, biggestY), int(biggestC ** 0.45), (0,0,255), 5)
          cv2.putText(frame, "Proximity: {} centimeters".format(self.ad_msg.ad_proximity), (320, 170), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)
          cv2.putText(frame, "Error: {}X {}Y".format(self.ad_msg.ad_x_error, self.ad_msg.ad_y_error), (320, 570), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)
          
          self.auto_dock_pub.publish(self.ad_msg)
        frame = cv2.rectangle(frame, (320, 180), (960, 540), (19,185,253), 2)
        return frame
    
    # Get vertical thrust from throttle
    def throttle_callback(self, data):
      self.rov_throttle = data.t5
    
    # Code for displaying most recent frame + overlay
    def read(self):
        depthLevel = self.depth_calibration()
        if self.change:
            self.cap.release()
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))
            self.change = False

        # Read the most recent frame from the video stream into ret and frame
        ret, frame = self.cap.read()
        net, self.image = self.cap.read()
        if frame is None:
            self.change = True
            return False
        if ret is None:       
            rospy.logwarn('camera_viewer: ret is None, can\'t display new frame')
            return False
        else: # If there is no error reading the last frame, add the text
            # Targeting System
            if self.auto_dock:
              frame = self.docking_targeting(frame)
            
            # Camera number
            cv2.putText(frame, str(self.num), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)

            # Add depth reading
            if depthLevel < 0.5: # Displays 0 ft when surfaced rather than weird number
                depthLevel = 0
            
            textSize = cv2.getTextSize("{:.2f} ft".format(abs(depthLevel)), cv2.FONT_HERSHEY_COMPLEX, 1, 2)[0]
            cv2.putText(frame, "{:.2f} ft".format(abs(depthLevel)), (1260 - textSize[0], 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            
            # Depth Bar
            frame = self.depth_bar(frame, depthLevel)
            frame = self.throttle_bar(frame)

            return frame

    # Delay until camera IP is added to verified
    def wait(self):
        rospy.loginfo('camera_viewer: waiting for cameras - None connected')
        while not self.verified:
            if rospy.is_shutdown():
                return
            rospy.logdebug('camera_viewer: still no cameras connected')
            time.sleep(1)

        # Initializes first camera
        self.num = 1
        rospy.loginfo("camera_viewer: loading capture from camera {}".format(self.num))
        if self.ip:
            self.camera_data.screenshot = False
            self.camera_data.ip = self.ip
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))
        else:
            rospy.logerr("camera_viwer: there is no camera at spot 1 after waiting.")


    # Changes cameras callback
    def change_camera_callback(self, joy_data):
        # Checks to make sure it hasn't already selected that camera
        cam_select = self.num
        change = False
        self.camera_data.screenshot = False

        if joy_data.axes[4] != 0 or joy_data.axes[5] != 0:
            self.camera_data.screenshot = True

        if joy_data.buttons[2]:
            cam_select = 1
        elif joy_data.buttons[3]:
            cam_select = 2
        elif joy_data.buttons[4]:
            cam_select = 3
        elif joy_data.buttons[5]:
            cam_select = 4

        try: self.camera_data.ip = self.verified[cam_select]  
        except: 
            if self.camera_data.screenshot: pass
            else: return

        if self.camera_data.screenshot:
            try:
                self.image = self.bridge.cv2_to_imgmsg(self.image, encoding="passthrough")
                self.image_pub.publish(self.image)
            except: pass

        if self.num != cam_select and self.num != 0:
            if self.ip:
                self.num = cam_select
                self.change= True
                rospy.loginfo("camera_viewer: changing to camera {}".format(self.num))
                
    def enable_auto_dock(self, auto_control_data):
        self.auto_dock = auto_control_data.auto_dock
        self.dh_enable = auto_control_data.dh_status
        self.target_depth = auto_control_data.target_depth

    # ROSPY subscriber to change depth
    def change_depth_callback(self, depth):
        self.depth = depth.data

    # Creates a web server on port 12345 and waits until it gets pinged
    # Then it adds the camera IP to self.verified
    def find_cameras(self):
        # Create the app
        app = flask.Flask(__name__)

        # Function that runs when the app gets a connection

        @app.route('/', methods=["POST", "GET"])
        def page():
            # If the IP that pinged flask is not already connected, and the length of request form is not 0
            if flask.request.remote_addr not in self.verified.values() and len(flask.request.form) > 0:
                rospy.loginfo(flask.request.remote_addr)
                # Add the IP to self.verified
                self.verified[self.give_num(flask.request.remote_addr)] = flask.request.remote_addr
                rospy.loginfo('camera_viewer: cameras currently connected: {}'.format(self.verified))
            return ""

        # Run the app
        rospy.loginfo('camera_viewer: camera web server online')
        app.run(host='0.0.0.0', port=12345)

    # Assign number (lower possible) to new cameras in self.verified
    def give_num(self, ip):
        if ip in self.config:
            return self.config[ip]
        else:
            try:
                available = [num for num in range(1, 8) if num not in self.verified and num not in self.config.values()][0]
            except IndexError:
                rospy.logerr('camera_viewer: camera detected, but there are no available numbers')
            return available

    # Closes the program nicely
    def cleanup(self):
        flask.request.environ.get('werkzeug.server.shutdown')()
        self.camera_thread.terminate()
        self.camera_thread.join()

    # Renders the window
def main():
    rospy.init_node('camera_feed')
    switcher = CameraSwitcher()
    switcher.wait()

    cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
    fullscreen = True
    while not rospy.is_shutdown():
        frame = switcher.read()
        if cv2.waitKey(1) == ord('f'):
            if fullscreen == True:
                cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                fullscreen = False
            elif fullscreen == False:
                cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                fullscreen = True
        if frame is not False:
            cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
