from Robot.Robot import Robot
from vision.aruco import ArucoDetector
import cv2

robot = Robot()
detector = ArucoDetector()

count = 0

try:
    while True:
        frame = robot.get_frame()
        if frame is None:
            continue

        results = detector.detect(frame, show=True)

        if results:
            marker = results[0]
            print("Posizione XYZ:", marker["tvec"])
            print("Distanza:", marker["distance"])

        cv2.imshow("frame", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
finally:
    robot.stop()
    cv2.destroyAllWindows()