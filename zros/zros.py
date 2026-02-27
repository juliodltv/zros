import pickle
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import zmq

try:
    import numpy as np
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class _CvBridgeBase:
    def __init__(self) -> None:
        if not HAS_CV2:
            raise ImportError(
                "Computer Vision features require 'numpy' and 'opencv-python'. "
                "Please install them manually or use: uv pip install zros[cv2]"
            )


class zCompressedCvBridge(_CvBridgeBase):
    def cv2_to_zimgmsg(self, cv_image: "np.ndarray") -> bytes:
        """Converts an OpenCV image to JPEG-compressed bytes."""
        _, buffer = cv2.imencode('.jpg', cv_image)
        return buffer.tobytes()

    def zimgmsg_to_cv2(self, image_bytes: bytes) -> "Optional[np.ndarray]":
        """Converts JPEG-compressed bytes back to an OpenCV image."""
        try:
            image_array = np.frombuffer(image_bytes, dtype=np.uint8)
            return cv2.imdecode(image_array, cv2.IMREAD_UNCHANGED)
        except Exception as e:
            print(f"zCompressedCvBridge Error: {e}")
            return None


class zCvBridge(_CvBridgeBase):
    def cv2_to_zimgmsg(self, cv_image: "np.ndarray") -> Optional[Dict[str, Any]]:
        """Converts an OpenCV image to a raw bytes dictionary message."""
        if cv_image is None:
            return None
        return {
            "bytes": cv_image.tobytes(),
            "shape": cv_image.shape,
            "dtype": str(cv_image.dtype),
        }

    def zimgmsg_to_cv2(self, msg: Optional[Dict[str, Any]]) -> "Optional[np.ndarray]":
        """Converts a raw bytes dictionary message back to an OpenCV image."""
        if msg is None or "bytes" not in msg:
            return None
        try:
            return np.frombuffer(msg["bytes"], dtype=msg["dtype"]).reshape(msg["shape"])
        except Exception as e:
            print(f"zCvBridge Error: {e}")
            return None


class Publisher:
    def __init__(self, socket: zmq.Socket, topic: str) -> None:
        self.socket = socket
        self.topic = topic

    def publish(self, payload: Any) -> None:
        """Publishes an arbitrary Python object to this topic."""
        self.socket.send_multipart([
            self.topic.encode('utf-8'),
            pickle.dumps(payload),
        ])


class Timer:
    def __init__(self, period: float, callback: Callable[[], None]) -> None:
        self.period = period
        self.callback = callback
        self.last_call = time.time()

    def check(self) -> None:
        now = time.time()
        if now - self.last_call >= self.period:
            self.callback()
            self.last_call = now


class zNode:
    def __init__(
        self,
        name: str,
        ip: str = "127.0.0.1",
        port_pub: int = 5556,
        port_sub: int = 5555,
    ) -> None:
        self.name = name
        self.context = zmq.Context()

        # Socket for Publishing (connects to zroscore input)
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.connect(f"tcp://{ip}:{port_pub}")

        # Socket for Subscribing (connects to zroscore output)
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{ip}:{port_sub}")

        self.poller = zmq.Poller()
        self.poller.register(self.sub_socket, zmq.POLLIN)

        self.callbacks: Dict[str, Callable[[Any], None]] = {}
        self.timers: List[Timer] = []
        self.publishers: List[Publisher] = []
        self._queue_sizes: Dict[str, int] = {}  # topic → max queue size (0 = unlimited)
        self.running = True

        self._graph_pub = self.create_publisher("_zros/graph")
        self.create_timer(1.0, self._broadcast_graph_info)

    def _broadcast_graph_info(self) -> None:
        pubs = [p.topic for p in self.publishers if not p.topic.startswith("_zros")]
        subs = [t for t in self._queue_sizes if not t.startswith("_zros")]
        self._graph_pub.publish({
            "name": self.name,
            "publishers": pubs,
            "subscribers": subs,
        })

    def create_publisher(self, topic: str) -> Publisher:
        pub = Publisher(self.pub_socket, topic)
        self.publishers.append(pub)
        return pub

    def create_subscriber(
        self,
        topic: str,
        callback: Optional[Callable[[Any], None]] = None,
        queue_size: int = 1,
    ) -> None:
        """
        Args:
            topic: Topic name to subscribe to.
            callback: Function called when a message is received. Signature: callback(payload).
            queue_size: Maximum number of messages to buffer per topic.
                        - 1 (default): keep only the latest — good for high-frequency sensor data.
                        - N > 1: keep the last N messages in order.
                        - 0: unlimited — process every message — use for commands or events.
        """
        self.sub_socket.setsockopt(zmq.SUBSCRIBE, topic.encode('utf-8'))
        if callback:
            self.callbacks[topic] = callback
        self._queue_sizes[topic] = queue_size

    def create_timer(self, period: float, callback: Callable[[], None]) -> Timer:
        """
        Args:
            period: Time in seconds between callbacks.
            callback: Function to call.
        """
        timer = Timer(period, callback)
        self.timers.append(timer)
        return timer

    def spin(self) -> None:
        """Blocks and processes messages and timers until the node is stopped."""
        print(f"[{self.name}] Spinning...")
        try:
            while self.running:
                for timer in self.timers:
                    timer.check()
                # 1 ms poll timeout keeps timers responsive without busy-waiting
                self.spin_once(timeout=1)
        except KeyboardInterrupt:
            print(f"[{self.name}] Stopping...")
        finally:
            self.destroy_node()

    def spin_once(self, timeout: int = 0) -> Optional[Tuple[str, Any]]:
        """
        Checks for incoming messages once.

        Drains the full socket queue, applies per-topic queue_size limits,
        then fires callbacks.

        Args:
            timeout: Poller timeout in milliseconds (0 = non-blocking).

        Returns:
            (topic, payload) of the last processed message, or None.
        """
        socks = dict(self.poller.poll(timeout))
        if self.sub_socket not in socks:
            return None

        collected: Dict[str, List[Any]] = {}
        while True:
            try:
                topic_bytes, pickled_payload = self.sub_socket.recv_multipart(flags=zmq.NOBLOCK)
                topic = topic_bytes.decode('utf-8')
                payload = pickle.loads(pickled_payload)
                collected.setdefault(topic, []).append(payload)
            except zmq.Again:
                break
            except Exception as e:
                print(f"Error decoding message: {e}")
                break

        last = None
        for topic, payloads in collected.items():
            queue_size = self._queue_sizes.get(topic, 1)
            if queue_size == 0:
                trimmed = payloads                    # unlimited: process all
            elif queue_size == 1:
                trimmed = [payloads[-1]]              # keep only latest
            else:
                trimmed = payloads[-queue_size:]      # keep last N
            for payload in trimmed:
                if topic in self.callbacks:
                    self.callbacks[topic](payload)
                last = (topic, payload)

        return last

    def destroy_node(self) -> None:
        self.running = False
        self.pub_socket.close()
        self.sub_socket.close()
        self.context.term()
