import cv2
import cv2.aruco as aruco
import numpy as np
import math

def rotation_vector_to_euler_angles(rvec):
    R, _ = cv2.Rodrigues(rvec)

    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2]) 
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    roll  = ((np.degrees(x)) % 360 - 180)
    pitch = ((np.degrees(y) + 180) % 360 - 180)
    yaw   = ((np.degrees(z) + 180) % 360 - 180)

    return roll, pitch, yaw


class ArucoDetector:
    def __init__(self, calibration_path="config/camera_calibration.npz", marker_size=0.077):
        # Dizionario ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Carico calibrazione camera
        data = np.load(calibration_path)
        self.camera_matrix = data["camera_matrix"]
        self.dist_coeffs = data["dist_coeffs"]

        self.marker_size = marker_size

    def detect(self, frame, show=True):
        """
        Ritorna una lista di dizionari:
        [
          {
            "id": 23,
            "rvec": [...],
            "tvec": [...],
            "distance": float,
            "roll": float,
            "pitch": float,
            "yaw": float
          },
          ...
        ]
        """

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gaus = cv2.GaussianBlur(gray, (3,3), 0)
        cv2.imshow("gr", gaus)
        corners, ids, _ = self.detector.detectMarkers(gaus)

        results = []

        if ids is not None:
            out = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            aruco.drawDetectedMarkers(frame, corners, ids)
            # Stima pose
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                distance = np.linalg.norm(tvec)
                roll, pitch, yaw = rotation_vector_to_euler_angles(rvec)

                # Salvo tutto
                results.append({
                    "id": int(marker_id[0]),
                    "rvec": rvec,
                    "tvec": tvec,
                    "distance": float(distance),
                    "roll": float(roll),
                    "pitch": float(pitch),
                    "yaw": float(yaw)
                })

                if show:
                    # Disegno marker

                    # Disegno assi
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                      rvec, tvec, self.marker_size * 0.5)

                    # Testi
                    cv2.putText(frame,
                                f"ID:{marker_id[0]} Dist:{distance*100:.1f}cm",
                                tuple(corners[i][0][0].astype(int)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                    cv2.putText(frame,
                                f"R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f}",
                                tuple(corners[i][0][3].astype(int) + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        if show:
            cv2.imshow("Aruco Detection", frame)

        return results
