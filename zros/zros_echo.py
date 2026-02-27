import argparse
import time
from zros import zNode


def main():
    parser = argparse.ArgumentParser(description="Print messages from a ZROS topic.")
    parser.add_argument("topic", help="Topic name")
    args = parser.parse_args()

    node = zNode(f"zros_echo_{int(time.time())}")
    node.create_subscriber(args.topic, print)
    print(f"Subscribed to {args.topic}")
    try:
        node.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
