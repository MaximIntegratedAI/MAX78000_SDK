#!/usr/bin/env python3
###################################################################################################
#
# Copyright (C) 2018-2020 Maxim Integrated Products, Inc. All Rights Reserved.
#
# Maxim Integrated Products, Inc. Default Copyright Notice:
# https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
#
###################################################################################################
"""Thread to capture image from camera continiously
"""
import time

from PyQt5.QtCore import (QThread, pyqtSignal, Qt)
from PyQt5.QtGui import QImage

from image_utils import cvt_img_to_qimage


class Thread(QThread):
    """
    Thread to capture image from camera
    """
    change_pixmap = pyqtSignal(QImage)

    def __init__(self, parent=None, camera=None, frame_rate=25):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        self.emit_period = 1.0 / frame_rate

    def run(self):
        """Runs camera capture"""
        prev = time.time()
        while True:
            now = time.time()
            rval, frame = self.camera.get_frame()
            if rval:
                convert_qt_format = cvt_img_to_qimage(frame)
                qt_img = convert_qt_format.scaled(640, 480, Qt.KeepAspectRatio)
                if (now - prev) >= self.emit_period:
                    self.change_pixmap.emit(qt_img)
                    prev = now
