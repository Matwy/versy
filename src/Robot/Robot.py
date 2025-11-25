import time

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
        time.sleep(1)

    def get_frame(self):
        return self.cam_stream.read()

    def stop(self):
        self.cam_stream.stop()