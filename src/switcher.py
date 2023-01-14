#!/usr/bin/env python3
# Switcher.py
# By Andrew Grindstaff '21
# Maintained by Alex Bertran '24 (06/27/22 - Present)
# ROS Package to switch between cameras streaming to port 5000

import cv2
import numpy as np
import time
import threading
import flask
import rospy
from std_msgs.msg import UInt8, Float32, Int32
from copilot_interface.msg import controlData
from sensor_msgs.msg import Image

class CameraSwitcher:
    #"""The class for handling all the camera logic. Switches and reads the
    #camera, adding an overlay to it.
    #"""
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

        # Old functionality where we had a config file. Not necessary, Cut when possible.
        self.config = {}

        # Create Subscribers
        self.camera_sub = rospy.Subscriber('/control', controlData, self.change_camera_callback)
        self.depth_sub = rospy.Subscriber('rov/depth_sensor', Float32, self.change_depth_callback)
        
        self.depth = 0

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

    # Code for displaying most recent frame + overlay
    def read(self):
        # """Reads a frame from the cv2 video capture and adds the overlay to it"""
        depthLevel = self.depth_calibration()
        if self.change:
            self.cap.release()
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))
            self.change = False

        ret, frame = self.cap.read()
        if frame is None:
            self.change = True
            return False
        if ret is None:
            rospy.logwarn('camera_viewer: ret is None, can\'t display new frame')
            return False
        else:
            cv2.putText(frame, str(self.num), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            if depthLevel < 0.5: # Displays 0 ft when surfaced rather than weird number
                depthLevel = 0
            textSize = cv2.getTextSize("{:.2f} ft".format(-abs(depthLevel)), cv2.FONT_HERSHEY_COMPLEX, 1, 2)[0]
            cv2.putText(frame, "{:.2f} ft".format(-abs(depthLevel)), (1260 - textSize[0], 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            #textSize = cv2.getTextSize("{:.2f} C".format(self.temp), cv2.FONT_HERSHEY_COMPLEX, 1, 2)[0]            
            # Depth Bar
            frame = self.depth_bar(frame)

            return frame

    # Delay until camera IP is added to verified
    def wait(self):
        #"""Waits for a camera IP to be put into verified"""
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
            rospy.logerr("camera_viwer: there is no camera at spot 1 after waiting.")

    # Changes cameras
    def change_camera_callback(self, control_data):
        # """ROSPY subscriber to change cameras"""
        if self.num != control_data.camera:
            if self.ip:
                self.change = True
                self.num = control_data.camera
                rospy.loginfo("camera_viewer: changing to camera {}".format(self.num))

    def change_depth_callback(self, depth):
        # """ROSPY subscriber to change depth"""
        self.depth = depth.data


    def find_cameras(self):
        # """Creates a web server on port 12345 and waits until it gets pinged"""
        app = flask.Flask(__name__)
        @app.route('/', methods=["POST", "GET"])
        def page():
            rospy.loginfo("find_cameras running")
            if flask.request.remote_addr not in self.verified.values() and len(flask.request.form) > 0:
                rospy.loginfo(list(flask.request.form.keys())[0]) #()[0] in self.config.keys())
                if list(flask.request.form.keys())[0] in self.config.keys():
                    self.verified[self.config[list(flask.request.form.keys())[0]]] = flask.request.remote_addr
                else:
                    self.verified[self.give_num(flask.request.remote_addr)] = flask.request.remote_addr
                rospy.loginfo('camera_viewer: cameras currently connected: {}'.format(self.verified))
            return ""

        rospy.loginfo('camera_viewer: camera web server online')
        app.run(host='0.0.0.0', port=12345)

    def give_num(self, ip):
        # """Gives the lowest available number to the ip"""
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

    # Renders the window
def main():
    rospy.init_node('camera_feed')
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
