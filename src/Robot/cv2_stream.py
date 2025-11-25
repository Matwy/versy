import cv2


class CV2CameraStream:
    """Fallback camera stream using cv2.VideoCapture for PC"""

    # Stesse impostazioni di PiCameraStream
    DEFAULT_RESOLUTION = (320*2, 240*2)

    def __init__(self, camera_index: int = 0, resolution: tuple = DEFAULT_RESOLUTION):
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

        if not self.cap.isOpened():
            raise RuntimeError(f"Impossibile aprire la camera {camera_index}")

    def start(self):
        return self

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        # Converti BGR (default cv2) in RGB per uniformità con PiCamera (RGB888)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def stop(self):
        self.cap.release()
