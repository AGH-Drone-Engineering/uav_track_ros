#!/usr/bin/env python

import os
import shutil
import cv2
from time import time as get_time

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from uav_track.uav_track import UavTrack


BBOX_SIZE = 320, 320


class DetectorNode:
    def __init__(self):
        rospy.init_node('detector')

        cap_device = rospy.get_param('~cap', 0)
        self.out_dir = rospy.get_param('~out', None)

        if self.out_dir is None:
            raise ValueError('Output directory is not specified')
        
        self.out_dir = os.path.abspath(self.out_dir)

        self.position_sub = rospy.Subscriber('position', NavSatFix, self.position_callback)
        self.hdg_sub = rospy.Subscriber('hdg', Float64, self.hdg_callback)

        self.detection_pub = rospy.Publisher('detection', NavSatFix, queue_size=16)

        self.last_position = None
        self.last_hdg = None

        shutil.rmtree(self.out_dir, ignore_errors=True)
        os.makedirs(os.path.join(self.out_dir, 'raw'))
        os.makedirs(os.path.join(self.out_dir, 'tracker'))
        os.makedirs(os.path.join(self.out_dir, 'detection'))

        self.uav_track = UavTrack()
        self.cap = cv2.VideoCapture(cap_device)

        if not self.cap.isOpened():
            raise Exception('Could not open video device', cap_device)

    def position_callback(self, msg: NavSatFix):
        self.last_position = msg

    def hdg_callback(self, msg: Float64):
        self.last_hdg = msg

    def report_detection(self, frame, i, track):
        x, y = track
    
        x0 = max(0, x - BBOX_SIZE[0] // 2)
        y0 = max(0, y - BBOX_SIZE[1] // 2)
        x1 = min(frame.shape[1], x + BBOX_SIZE[0] // 2)
        y1 = min(frame.shape[0], y + BBOX_SIZE[1] // 2)

        cropped = frame[y0:y1, x0:x1]

        path = os.path.join(self.out_dir, 'detection', f'{i}.png')
        cv2.imwrite(path, cropped)

        if self.last_position is None or self.last_hdg is None:
            return

        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = path
        # TODO offset with heading
        msg.latitude = self.last_position.latitude
        msg.longitude = self.last_position.longitude
        msg.altitude = self.last_position.altitude

        self.detection_pub.publish(msg)

    def run(self):
        last_time = get_time()
        i = 0

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr('Failed to capture frame')
                continue

            now = get_time()
            self.uav_track.feed(now - last_time, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            last_time = now

            frame_tracker = frame.copy()
            for x, y, w, h in self.uav_track.detections:
                cv2.rectangle(frame_tracker, (x, y), (x + w, y + h), (0, 255, 0), 2)

            for x, y in self.uav_track.tracks:
                cv2.circle(frame_tracker, (x, y), 2, (0, 255, 0), 2)

            for x, y in self.uav_track.tracks:
                self.report_detection(frame_tracker, i, (x, y))

            if self.out_dir is not None:
                cv2.imwrite(os.path.join(self.out_dir, 'raw', f'{i}.png'), frame)
                cv2.imwrite(os.path.join(self.out_dir, 'tracker', f'{i}.png'), frame_tracker)

            i += 1


if __name__ == '__main__':
    node = DetectorNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
