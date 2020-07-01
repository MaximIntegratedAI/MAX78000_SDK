#!/usr/bin/env python3
###################################################################################################
#
# Copyright (C) 2018-2020 Maxim Integrated Products, Inc. All Rights Reserved.
#
# Maxim Integrated Products, Inc. Default Copyright Notice:
# https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
#
###################################################################################################
"""Includes camera class to capture image
"""
import cv2


class Camera:
    """Captures image from selected camera and returns image inside drawed rectangle"""
    def __init__(self, cam_num=0, frame_size=(240, 320)):
        self.cap = cv2.VideoCapture(cam_num)
        self.cam_num = cam_num

        width = int(self.cap.get(3))  # int
        height = int(self.cap.get(4))  # int
        print(f'Webcam image size: ({width}, {height})')  # webcam_size

        frame_width = frame_size[0]
        frame_height = frame_size[1]

        self.thickness = 3
        self.start_point = ((width - frame_width) // 2, (height - frame_height) // 2)
        self.end_point = (self.start_point[0] + frame_width, self.start_point[1] + frame_height)
        self.color = (255, 0, 0)

    def get_frame(self):
        """Returns the next captured image"""
        rval, frame = self.cap.read()
        if rval:
            frame = cv2.rectangle(frame, tuple(q-self.thickness for q in self.start_point),
                                  tuple(q+self.thickness for q in self.end_point),
                                  self.color, self.thickness)
            frame = cv2.flip(frame, 1)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        return rval, frame

    def close_camera(self):
        """Releases camera"""
        self.cap.release()

    def __str__(self):
        return f'OpenCV Camera {self.cam_num}'
