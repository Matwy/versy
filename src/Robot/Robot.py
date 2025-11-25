from .picamera_stream import PiCameraStream
import time

class Robot():
    def __init__(self):
        self.cam_stream = PiCameraStream()
        self.cam_stream.start()
        time.sleep(1)
            
    def get_frame(self):
        return self.cam_stream.read()