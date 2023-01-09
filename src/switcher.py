#!/usr/bin/env python3
# Switcher.py
# By Andrew Grindstaff
# Maintained by Alex Bertran (06/27/22 - Present)
# ROS Package to switch between cameras streaming to port 5000
# Streamer code in streamer directory at streamer.py

import cv2
import numpy as np
import json
import time
import threading
import flask
import rospy
from std_msgs.msg import UInt8, Float32, Int32
from sensor_msgs.msg import Image
from gpio_control.srv import gpio_status

# NEED TO ADD SENSOR DATA
# Overlay and timer stack
# Task specific visuals - overlay


class CameraSwitcher:
    """The class for handling all the camera logic. Switches and reads the
    camera, adding an overlay to it.
    """
    def __init__(self):
        # self.verified is a dictionary that has keys of camera numbers and
        # values of ip addresses -
        # self.verified = {
        #  1: "192.168.1.101",
        #  2: "192.168.1.102"
        # }
        self.verified = {}
        # self.num is an integer that represents the current camera number and
        # is the key for self.verified
        self.num = 0
        self.change = False
        self.cap = None

        try:
            self.config = json.load(open("/home/jhsrobo/config.json"))
        except IOError:
            rospy.logwarn("camera_viewer: please make config.json if you want to save camera settings")
            self.config = {}

        self.camera_sub = rospy.Subscriber('rov/camera_select', UInt8, self.change_camera_callback)

        self.temp_sub = rospy.Subscriber('rov/temp_sensor', Float32, self.change_temp_callback)
        self.temp = 0

        self.depth_sub = rospy.Subscriber('rov/depth_sensor', Float32, self.change_depth_callback)
        self.depth = 0

        self.gpio_sub = rospy.Subscriber('rov/gpio_control', Int32, self.change_gpio_callback)
        try:
          self.gpio_service = rospy.ServiceProxy('gpio_status_server', gpio_status)
        except rospy.ServiceException as e:
            rospy.logerr("camera_viewer: GPIO status server not initialized. Will retry.")
            self.gpio_service = None
        self.electromags = {11: [False, "Left pad"],
                           15: [False, "Right pad"]
                           }

        self.camera_thread = threading.Thread(target=self.find_cameras)
        self.camera_thread.setDaemon(True)

        self.camera_thread.start()

    @property
    def ip(self):
        """Ensures that the IP of the camera is always the correct number
        without sacraficing redability. Otherwise, returns False
        """
        try:
            return self.verified[self.num]
        except KeyError:
            rospy.logerr("camera_viewer: passed a camera number that doesn't exist")
            if len(self.verified.keys()) == 0:
                return ""
            return self.verified[self.verified.keys()[0]]
        
    def depth_calibration(self):
        return abs((self.depth - 198.3) / (893.04 / 149))
    
    def depth_bar(self, frame):
        depthLevel = self.depth_calibration()
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

        # Draw pointer
        pt1 = (1240, (depthLevel * 32) + 128)
        pt2 = (1190, (depthLevel * 32) + 153)
        pt3 = (1190, (depthLevel * 32) + 103)

        pointer = np.array([pt1, pt2, pt3])
        cv2.drawContours(frame, [pointer.astype(int)], 0, (19,185,253), -1)
        return frame

    def read(self):
        """Reads a frame from the cv2 video capture and adds the overlay to it"""
        depthLevel = self.depth_calibration()
        if self.change:
            self.cap.release()
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))
            self.change = False

        ret, frame = self.cap.read()
        if frame is None:
            rospy.logerr('camera_viewer: camera failed - please wait for it to refresh or switch cameras')
            self.change = True
            return False
        elif ret is None:
            rospy.logwarn('camera_viewer: ret is None, can\'t display new frame')
            return False
        else:
            cv2.putText(frame, str(self.num), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            if depthLevel < 0.5: # Displays 0 ft when surfaced rather than weird number
                depthLevel = 0
            textSize = cv2.getTextSize("{:.2f} ft".format(-abs(depthLevel)), cv2.FONT_HERSHEY_COMPLEX, 1, 2)[0]
            cv2.putText(frame, "{:.2f} ft".format(-abs(depthLevel)), (1260 - textSize[0], 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            textSize = cv2.getTextSize("{:.2f} C".format(self.temp), cv2.FONT_HERSHEY_COMPLEX, 1, 2)[0]
            
            if self.temp < 60:
                color = (0, 255, 0)
            elif 60 <= self.temp < 80:
                color = (0, 255, 255)
            else:
                color = (0, 0, 255)
            cv2.putText(frame, "{:.2f} C".format(self.temp), (1260 - textSize[0], 80), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
            height = 120 + textSize[1]
            
            # Jesuit "Watermark"
            textSize = cv2.getTextSize("Jesuit Robotics", cv2.FONT_HERSHEY_TRIPLEX, 1, 2)[0]
            cv2.putText(frame, "Jesuit Robotics", (1260 - textSize[0], 705), cv2.FONT_HERSHEY_TRIPLEX, 1, (48, 18, 196), 2, cv2.LINE_AA)
            
            # Depth Bar
            frame = self.depth_bar(frame)
            
            # Old code for electromagnets
            #for x in sorted(self.electromags.values()):
                #textSize = cv2.getTextSize("{}: {}".format(x[1], "On" if x[0] else "Off"), cv2.FONT_HERSHEY_COMPLEX, 1, 2)[0]
                #cv2.putText(frame, "{}: {}".format(x[1], "On" if x[0] else "Off"), (1260 - textSize[0], height), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                #height += textSize[1] + 20
            return frame

    def wait(self):
        """Waits for a camera IP to be put into verified"""
        rospy.loginfo('camera_viewer: waiting for cameras - None connected')
        while not self.verified:
            if rospy.is_shutdown():
                return
            rospy.logdebug('camera_viewer: still no cameras connected')
            time.sleep(1)

        self.num = 1
        rospy.loginfo("camera_viwer: loading capture from camera {}".format(self.num))
        if self.ip:
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))
        else:
            # lol something is really wrong
            rospy.logerr("camera_viwer: there is no camera at spot 1 after waiting.")

    def change_camera_callback(self, camera_num):
        """ROSPY subscriber to change cameras"""
        if not len(self.verified.keys()):
          rospy.logerr('camera_viewer: passed a value when none are initialized')
        elif self.num != camera_num.data:
            if self.ip:
                self.change = True
                self.num = camera_num.data
                rospy.loginfo("camera_viewer: changing to camera {}".format(self.num))

    def change_depth_callback(self, depth):
        """ROSPY subscriber to change depth"""
        self.depth = depth.data

    def change_temp_callback(self, temp):
        """ROSPY subscriber to change temp"""
        self.temp = temp.data

    def change_gpio_callback(self, data):
        """ROSPY subscriber to change gpio info"""
        if self.gpio_service is None:
            try:
                self.gpio_service = rospy.ServiceProxy('gpio_status_server', gpio_status)
            except rospy.ServiceException as e:
                rospy.logerr("camera_viewer: GPIO status server not initialized. Will retry.")
        if data.data in self.electromags:
            self.electromags[data.data][0] = self.gpio_service(data.data).status

    def find_cameras(self):
        """Creates a web server on port 12345 and waits until it gets pinged"""
        app = flask.Flask(__name__)

        @app.route('/', methods=["POST", "GET"])
        def page():
            rospy.logdebug('camera_viewer: ping from {}'.format(flask.request.remote_addr))
            if flask.request.remote_addr not in self.verified.values() and len(flask.request.form.keys()) > 0:
                if flask.request.form.keys()[0] in self.config.keys():
                    self.verified[self.config[flask.request.form.keys()[0]]] = flask.request.remote_addr
                else:
                    self.verified[self.give_num(flask.request.remote_addr)] = flask.request.remote_addr
                rospy.loginfo('camera_viewer: cameras currently connected: {}'.format(self.verified))
            return ""

        rospy.loginfo('camera_viewer: camera web server online')
        app.run(host='0.0.0.0', port=12345)

    def give_num(self, ip):
        """Gives the lowest available number to the ip"""
        if ip in self.config:
            return self.config[ip]
        else:
            try:
                available = [num for num in range(1, 8) if num not in self.verified and num not in self.config.values()][0]
            except IndexError:
                rospy.logerr('camera_viewer: camera detected, but there are no available numbers')
            return available

    def cleanup(self):
        """Closes the camera thread and attempts to cleanup the program"""
        flask.request.environ.get('werkzeug.server.shutdown')()
        self.camera_thread.terminate()
        self.camera_thread.join()


def main():
    rospy.init_node('pilot_page')
    switcher = CameraSwitcher()
    switcher.wait()

    cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
    fullscreen = False
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
