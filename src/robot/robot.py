import time
from .motors import Motors

# Try to import PiCamera, fallback to cv2 if not available
try:
    from .picamera_stream import PiCameraStream
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False
    print("PiCamera2 non disponibile, uso cv2.VideoCapture")


class Robot:
    def __init__(self, camera_index: int = 0):
        if PICAMERA_AVAILABLE:
            self.cam_stream = PiCameraStream()
        else:
            from .cv2_stream import CV2CameraStream
            self.cam_stream = CV2CameraStream(camera_index)

        self.cam_stream.start()
        self.motorsObj = Motors()
        time.sleep(1)

    def get_frame(self):
        return self.cam_stream.read()

    def stop(self):
        self.cam_stream.stop()
    
    def motors(self, power1, power2, power3):
        self.motorsObj.set_speeds(power1, power2, power3)