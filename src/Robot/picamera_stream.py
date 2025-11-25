from picamera2 import Picamera2
from threading import Thread
import cv2
import numpy as np
from typing import Tuple, Optional

class PiCameraStream:
    """
    Camera stream in another thread
    """

    def __init__(
        self,
        resolution: Tuple[int, int] = (640, 480),
        brightness: Optional[float] = None,
        hflip: bool = False,
        vflip: bool = True,
    ):
        self._stopped = False
        self._frame = None
        self._thread: Optional[Thread] = None

        self.cam = Picamera2()

        print(self.cam.sensor_modes)

        full_fov_mode = None
        for mode in self.cam.sensor_modes:
            # Controlla se il crop_limits inizia da (0,0) - indica full FOV
            if mode['crop_limits'][0] == 0 and mode['crop_limits'][1] == 0:
                full_fov_mode = mode
                break

        # Configurazione video: stream "main" in BGR (perfetto per OpenCV)
        # buffer_count=6 è quello tipico per use-case video.
        config = self.cam.create_video_configuration(
            main={"size": resolution, "format": "RGB888"}, 
            sensor={
                'output_size': full_fov_mode['size'],  # Forza il sensor mode con full FOV
                'bit_depth': full_fov_mode['bit_depth']
            },
        )

        # Flip opzionale (equivalente di rotation/flip delle vecchie API)
        if hflip or vflip:
            from libcamera import Transform
            config["transform"] = Transform(hflip=int(hflip), vflip=int(vflip))

        self.cam.configure(config)

        # Imposta luminosità se fornita (ATTENZIONE: non è una scala 0-100)
        if brightness is not None:
            self.cam.set_controls({"Brightness": float(brightness)})

        self.cam.start()

    def start(self):
        self._thread = Thread(target=self._update, daemon=True)
        self._thread.start()
        return self

    def _update(self):
        while not self._stopped:
            frame = self.cam.capture_array()
            self._frame = frame

        self.cam.stop()

    def read(self):
        return self._frame

    def stop(self):
        self._stopped = True
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
