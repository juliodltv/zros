# ZROS: ZeroMQ ROS-like Framework

[![PyPI version](https://badge.fury.io/py/zros.svg)](https://badge.fury.io/py/zros)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python versions](https://img.shields.io/pypi/pyversions/zros.svg)](https://pypi.org/project/zros)

<div align="center">
  <img
    src="assets/logo.png"
    width="600"
    alt="zros_logo"
  />
</div>

**ZROS** is a fast, lightweight [ROS](https://www.ros.org/)-like library designed to bring the ease of ROS-2 to Python projects that require minimal overhead and high performance, using [ZeroMQ](https://zeromq.org/) for fast, asynchronous communication.

It provides a simple, pure Python alternative for robotic applications, computer vision pipelines, and distributed systems where a full ROS installation might be overkill.

## Key Features

-   **Fast & Lightweight:** Built on top of **ZeroMQ**, ensuring low-latency communication between nodes.
-   **ROS-like API:** Uses familiar concepts like `Node`, `Publisher`, `Subscriber`, `Timer`, and `spin()` making it easy for ROS-2 developers to adapt.
-   **No Complex Build System:** Pure Python. No `catkin_make`, no `colcon build`, no `source setup.bash`. Just run your Python scripts.
-   **Computer Vision Ready:** Includes a built-in `CvBridge` for seamless OpenCV image transport.

## Installation

### From PyPI (Recommended)
```bash
uv pip install zros
```

### From Source
```bash
git clone https://github.com/juliodltv/zros.git
cd zros
uv sync
```

## Quick Start

### Create a Publisher
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
            self.pub.publish({"image": self.bridge.cv2_to_msg(frame)})

if __name__ == "__main__":
    CameraPublisher().spin()
```

### Create a Subscriber
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

## Running Examples
You can find full examples in the `examples/` directory.

```bash
# Terminal 1
uv run zroscore

# Terminal 2
uv run publisher

# Terminal 3
uv run subscriber
```

## Documentation
For detailed usage instructions, examples, and API documentation, please refer to the [ZROS Documentation](https://juliodltv.github.io/zros/).

## Citation

```bibtex
@software{zros2026,
  author = {Julio De La Torre-Vanegas},
  title = {ZROS: A fast, lightweight ZeroMQ ROS-like library},
  year = {2026},
  url = {https://github.com/juliodltv/zros}
}
```