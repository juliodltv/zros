# ZROS: A fast, lightweight ROS-like library

[![PyPI version](https://badge.fury.io/py/zros.svg)](https://badge.fury.io/py/zros)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Documentation](https://img.shields.io/badge/docs-MkDocs-blue.svg)](https://juliodltv.github.io/zros/)


<div align="center">
  <img
    src="https://github.com/user-attachments/assets/a9d1d0d0-23df-4f1c-9462-415914c68fa9"
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
-   **Computer Vision Ready:** Includes a built-in `zCvBridge` for seamless OpenCV image transport, and `zCompressedCVBridge` which can optimize bandwidth usage up to 30x during transmission.
-   **Bridge Legacy and Modern Environments:** Build hybrid architectures spanning Python 3.8 and Python 3.12+ gracefully over `localhost`.

## Why ZROS? Bridging Legacy and Modern Robotics

A major hurdle in modern robotics research is the software gap between established frameworks and cutting-edge AI.

Projects using robust legacy systems like **ROS1 Noetic** are strictly tied to Python 3.8 and specific OS/hardware driver stacks. However, integrating state-of-the-art AI tools like **SAM3** (Segment Anything) requires modern Python 3.10+, updated PyTorch versions, and fresh dependencies.

**Traditional workarounds fall short:**

* Installing new ML libraries into a legacy `catkin_ws` almost always breaks the system with dependency hell.
* Using **Docker** introduces friction: passing through physical GPUs/cameras is unreliable, networking ROS topics across container boundaries is tedious, and rebuilding containers ruins rapid prototyping.

**The ZROS + `uv` Solution:**
Run your hardware and control loops in their native ROS1 environment. Run your modern AI models in completely isolated, modern Python virtual environments managed by `uv`. **ZROS connects them instantly**. 

**Example Usage:**
```bash
sjmp@pc:~$ rostopic pub -1 /detect_object/prompt std_msgs/String "data: 'hand'"
```
SAM3 Object Detection in ROS Noetic:
![SAM3 Object Detection]("https://github.com/user-attachments/assets/bba33a8b-52e0-4bc9-a14b-d5c61150acba")

*Read the [full guide on Bridging Systems](https://juliodltv.github.io/zros/bridge_case/) in the documentation.*

## Installation

### From PyPI (Recommended)
```bash
uv venv
# For basic ZROS (no computer vision features):
uv pip install zros

# To include OpenCV and Numpy for image transport:
uv pip install zros[cv2]
```

### From Source
```bash
git clone https://github.com/juliodltv/zros.git
cd zros
uv sync # Add --extra cv2 for image transport support
```

## Quick Start

### Create a Publisher (publisher.py)

```python
from zros import zNode, zCompressedCVBridge
import cv2

class CameraPublisher(zNode):
    def __init__(self):
        super().__init__("camera_pub")
        self.pub = self.create_publisher("video_topic")
        self.bridge = zCompressedCVBridge()
        self.cap = cv2.VideoCapture(0)
        self.create_timer(1/60, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # msg is a dictionary
            msg = {
                "zimgmsg": self.bridge.cv2_to_zimgmsg(frame),
                "info": "My Camera Frame"
            }
            self.pub.publish(msg)

if __name__ == "__main__":
    CameraPublisher().spin()
```

### Create a Subscriber (subscriber.py)
```python
from zros import zNode, zCompressedCVBridge
import cv2

class VideoSubscriber(zNode):
    def __init__(self):
        super().__init__("video_sub")
        self.bridge = zCompressedCVBridge()
        self.create_subscriber("video_topic", self.callback)

    def callback(self, msg):
        img = self.bridge.zimgmsg_to_cv2(msg["zimgmsg"])
        # info = msg["info"]
        # print(info)
        cv2.imshow("Video", img)
        cv2.waitKey(1)

if __name__ == "__main__":
    VideoSubscriber().spin()
```

## Running Examples
You can find another examples in the `examples/` directory.

```bash
# Terminal 1
uv run zroscore

# Terminal 2
uv run publisher.py

# Terminal 3
uv run subscriber.py
```

## Documentation
For detailed usage instructions, please refer to the [ZROS Documentation](https://juliodltv.github.io/zros/).

## Citation

```bibtex
@software{zros2026,
  author = {Julio De La Torre-Vanegas},
  title = {ZROS: A fast, lightweight ZeroMQ ROS-like library},
  year = {2026},
  url = {https://github.com/juliodltv/zros}
}
```
