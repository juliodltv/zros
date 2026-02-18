import argparse
import time
import ast
from zros import Node

class ZrosPub(Node):
    def __init__(self, topic, data, rate):
        super().__init__(f"zros_pub_{int(time.time())}")
        self.pub = self.create_publisher(topic)
        self.data = data
        self.create_timer(1.0 / rate, self.timer_callback)
        print(f"Publishing to {topic} at {rate} Hz: {data}")

    def timer_callback(self):
        self.pub.publish(self.data)

def main():
    parser = argparse.ArgumentParser(description="Publish a dictionary payload to a ZROS topic.")
    parser.add_argument("topic", help="Topic name")
    parser.add_argument("data", help="Dictionary payload (e.g. \"{'key': 'value'}\")")
    parser.add_argument("--rate", type=float, default=1.0, help="Publish rate in Hz")

    args = parser.parse_args()
    
    try:
        payload = ast.literal_eval(args.data)
        if not isinstance(payload, dict):
             print(f"Error: Payload must be a dictionary, got {type(payload)}")
             return
    except (ValueError, SyntaxError) as e:
        print(f"Error parsing dictionary: {e}")
        return

    node = ZrosPub(args.topic, payload, args.rate)
    try:
        node.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
