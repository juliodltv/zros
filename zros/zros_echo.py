import argparse
import time
from zros import Node

class ZrosEcho(Node):
    def __init__(self, topic):
        super().__init__(f"zros_echo_{int(time.time())}")
        self.create_subscriber(topic, self.callback)
        print(f"Subscribed to {topic}")

    def callback(self, payload):
        print(payload)

def main():
    parser = argparse.ArgumentParser(description="Print messages from a ZROS topic.")
    parser.add_argument("topic", help="Topic name")
    
    args = parser.parse_args()
    
    node = ZrosEcho(args.topic)
    try:
        node.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
