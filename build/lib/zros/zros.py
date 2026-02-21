import zmq
import pickle
import time
import numpy as np
import cv2

class zCompressedCVBridge:
    def cv2_to_zimgmsg(self, cv_image):
        """
        Converts a OpenCV image to compressed bytes.
        Returns: bytes
        """
        _, buffer = cv2.imencode('.jpg', cv_image)
        return buffer.tobytes()

    def zimgmsg_to_cv2(self, image_bytes):
        """
        Converts compressed bytes back to an OpenCV image.
        """
        try:
            image_array = np.frombuffer(image_bytes, dtype=np.uint8)
            return cv2.imdecode(image_array, cv2.IMREAD_UNCHANGED)
        except Exception as e:
            print(f"zCompressedCVBridge Error: {e}")
            return None

class zCvBridge:
    def cv2_to_zimgmsg(self, cv_image):
        """
        Converts a OpenCV image to a raw bytes dictionary message.
        """
        if cv_image is None:
            return None
        return {
            "bytes": cv_image.tobytes(),
            "shape": cv_image.shape,
            "dtype": str(cv_image.dtype)
        }

    def zimgmsg_to_cv2(self, msg):
        """
        Converts a raw bytes dictionary message back to an OpenCV image.
        """
        if msg is None or "bytes" not in msg:
            return None
        try:
            return np.frombuffer(msg["bytes"], dtype=msg["dtype"]).reshape(msg["shape"])
        except Exception as e:
            print(f"CvBridge Error: {e}")
            return None

class Publisher:
    def __init__(self, socket, topic):
        self.socket = socket
        self.topic = topic
        
    def publish(self, payload):
        """
        Publishes an arbitrary python object (payload) as a dictionary.
        """
        self.socket.send_multipart([
            self.topic.encode('utf-8'),
            pickle.dumps(payload)
        ])

class Timer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.last_call = time.time()

    def check(self):
        now = time.time()
        if now - self.last_call >= self.period:
            self.callback()
            self.last_call = now

class zNode:
    def __init__(self, name, ip="127.0.0.1", port_pub=5556, port_sub=5555):
        self.name = name
        self.context = zmq.Context()
        
        # Socket for Publishing (Connects to zroscore input)
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.connect(f"tcp://{ip}:{port_pub}")
        
        # Socket for Subscribing (Connects to zroscore output)
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{ip}:{port_sub}")
        
        self.poller = zmq.Poller()
        self.poller.register(self.sub_socket, zmq.POLLIN)
        
        self.callbacks = {}
        self.timers = []
        self.publishers = []
        self.running = True
        
        # Broadcast graph info every 1s
        self.create_timer(1.0, self._broadcast_graph_info)
        self._graph_pub = Publisher(self.pub_socket, "_zros/graph")

    def create_publisher(self, topic):
        pub = Publisher(self.pub_socket, topic)
        self.publishers.append(pub)
        return pub

    def create_subscriber(self, topic, callback=None):
        """
        Args:
            topic: Topic name to subscribe to.
            callback: Function to call when a message is received (optional).
                      Signature: callback(payload)
        """
        self.sub_socket.setsockopt(zmq.SUBSCRIBE, topic.encode('utf-8'))
        
        if callback:
            self.callbacks[topic] = callback
            
        try:
            self.sub_socket.setsockopt(zmq.CONFLATE, 1)
        except zmq.ZMQError:
            pass

    def create_timer(self, period, callback):
        """
        Args:
            period: Time in seconds between callbacks.
            callback: Function to call.
        """
        timer = Timer(period, callback)
        self.timers.append(timer)
        return timer

    def _broadcast_graph_info(self):
        """
        Broadcasts the node's graph information (name, pubs, subs).
        """
        graph_info = {
            "name": self.name,
            "publishers": [p.topic for p in self.publishers],
            "subscribers": list(self.callbacks.keys())
        }
        # We use a special internal publisher for this to avoid infinite recursion
        # if we were tracking *all* publishers including this one.
        # But here _graph_pub is arguably a publisher too. 
        # For simplicity, we don't add _graph_pub to self.publishers list.
        self._graph_pub.publish(graph_info)
        
    def spin(self):
        """
        Blocks and processes messages and timers forever until node is stopped.
        """
        print(f"[{self.name}] Spinning...")
        try:
            while self.running:
                # Check timers
                for timer in self.timers:
                    timer.check()
                
                # Check for messages (non-blocking to allow timers to run)
                self.spin_once(timeout=1) 
                
                # Small sleep to prevent 100% CPU usage if no messages
                # But we have spin_once with timeout=1ms so that acts as sleep
                
        except KeyboardInterrupt:
            print(f"[{self.name}] Stopping...")
        finally:
            self.destroy_node()

    def spin_once(self, timeout=0):
        """
        Checks for incoming messages.
        If a callback is registered for the topic, it executes it.
        Returns: (topic, payload) or None
        """
        socks = dict(self.poller.poll(timeout))
        if self.sub_socket in socks:
            try:
                # Receive [Topic, PickledPayload]
                topic_bytes, pickled_payload = self.sub_socket.recv_multipart(flags=zmq.NOBLOCK)
                topic = topic_bytes.decode('utf-8')
                payload = pickle.loads(pickled_payload)
                
                # Execute callback if exists
                if topic in self.callbacks:
                    self.callbacks[topic](payload)
                    
                return topic, payload
            except zmq.Again:
                return None
            except Exception as e:
                print(f"Error decoding message: {e}")
                return None
        return None
        
    def destroy_node(self):
        self.pub_socket.close()
        self.sub_socket.close()
        self.context.term()

