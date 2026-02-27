import argparse
import ast
import time
from zros import zNode


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

    node = zNode(f"zros_pub_{int(time.time())}")
    pub = node.create_publisher(args.topic)
    node.create_timer(1.0 / args.rate, lambda: pub.publish(payload))
    print(f"Publishing to {args.topic} at {args.rate} Hz: {payload}")
    try:
        node.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
