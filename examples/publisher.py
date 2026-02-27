import cv2
import numpy as np
from zros import zNode, zCvBridge


class CameraPublisher(zNode):
    def __init__(self):
        super().__init__("camera_publisher")
        self.pub_img = self.create_publisher("camera/image_raw")
        self.create_subscriber("camera/image_gray", self.gray_callback)
        self.cam = cv2.VideoCapture(0)
        self.bridge = zCvBridge()

        self.create_timer(1 / 60, self.timer_callback)

    def gray_callback(self, payload):
        msg = payload.get("msg", "")
        image = payload.get("image", None)

        print(msg)

        if image is None:
            print("No image")
            return

        gray_frame = self.bridge.zimgmsg_to_cv2(image)
        cv2.imshow("Gray image", gray_frame)
        cv2.waitKey(1)

    def timer_callback(self):
        ret, frame = self.cam.read()
        if not ret:
            return

        cv2.imshow("Original image", frame)
        cv2.waitKey(1)

        img_msg = self.bridge.cv2_to_zimgmsg(frame)
        payload = {
            "msg": "Hello from publisher",
            "image": img_msg,
        }
        self.pub_img.publish(payload)


if __name__ == "__main__":
    node = CameraPublisher()
    node.spin()
