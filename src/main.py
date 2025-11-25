from Robot.Robot import Robot
from vision.codereader import ArucoDetector
import cv2

robot = Robot()
detector = ArucoDetector()

count = 0

while True:
    frame = robot.get_frame()
    results = detector.detect(frame, show=True)

    if results:
        marker = results[0]
        print("Posizione XYZ:", marker["tvec"])
        print("Distanza:", marker["distance"])

    cv2.imshow("frame", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("p"):
        count += 1
        cv2.imwrite((str(count)), frame)

    if key == ord("q"):
        cv2.destroyAllWindows()
        break