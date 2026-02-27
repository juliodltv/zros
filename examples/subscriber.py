import cv2
import numpy as np
from zros import zNode, zCvBridge


class ImageSubscriber(zNode):
    def __init__(self):
        super().__init__("image_processor")
        self.create_subscriber("camera/image_raw", self.image_callback)
        self.pub_gray = self.create_publisher("camera/image_gray")
        self.bridge = zCvBridge()

    def image_callback(self, payload):
        msg = payload.get("msg", "")
        image = payload.get("image", None)

        print(msg)

        if image is None:
            print("No image")
            return

        frame = self.bridge.zimgmsg_to_cv2(image)
        cv2.imshow("image from the publisher", frame)
        cv2.waitKey(1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        img_msg = self.bridge.cv2_to_zimgmsg(gray)
        result_payload = {
            "msg": "Hello from subscriber",
            "image": img_msg,
        }
        self.pub_gray.publish(result_payload)


if __name__ == "__main__":
    node = ImageSubscriber()
    node.spin()
