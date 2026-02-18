# Usage Guide

ZROS mimics the ROS 2 workflow but uses ZeroMQ for communication.

### Create a Publisher (publisher.py)

This example demonstrates how to capture images from a camera and publish them.

1.  **OpenCV (`cv2`)**: Used to capture the video feed from your camera (`cv2.VideoCapture(0)`).
2.  **`CvBridge`**: ZROS provides a helper class to convert OpenCV images/arrays into compressed bytes suitable for transmission.
3.  **Payload**: ZROS sends data as a Python dictionary. This allows you to send multiple fields (e.g., "image", "timestamp", "metadata") in a single message.

```python
from zros import Node, CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_pub")
        self.pub = self.create_publisher("video_topic")
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.create_timer(1/60, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Payload is a dictionary
            msg = {
                "image": self.bridge.cv2_to_msg(frame),
                "info": "My Camera Frame"
            }
            self.pub.publish(msg)

if __name__ == "__main__":
    CameraPublisher().spin()
```

### Create a Subscriber (subscriber.py)
```python
from zros import Node, CvBridge
import cv2

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__("video_sub")
        self.bridge = CvBridge()
        self.create_subscriber("video_topic", self.callback)

    def callback(self, payload):
        img = self.bridge.msg_to_cv2(payload["image"])
        cv2.imshow("Video", img)
        cv2.waitKey(1)

if __name__ == "__main__":
    VideoSubscriber().spin()
```

### Running Examples
You can find another examples in the `examples/` directory.

```bash
# Terminal 1
uv run zroscore

# Terminal 2
uv run publisher.py

# Terminal 3
uv run subscriber.py
```