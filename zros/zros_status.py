import argparse
import time
from zros import zNode


def main():
    argparse.ArgumentParser(description="Live status of ZROS nodes and topics.").parse_args()

    nodes: dict = {}
    node = zNode(f"zros_status_{int(time.time())}")

    def graph_callback(payload):
        name = payload.get("name")
        if name and name != node.name:
            nodes[name] = {
                "publishers": payload.get("publishers", []),
                "subscribers": payload.get("subscribers", []),
            }

    def print_status_and_exit():
        if not nodes:
            print("No active nodes found. (Node graph discovery requires _zros/graph broadcasting)")
        else:
            for name, data in sorted(nodes.items()):
                print(f"- {name}")

                pubs = [p for p in data["publishers"] if not p.startswith("_zros")]
                if pubs:
                    print("    Publishers:")
                    for p in sorted(pubs):
                        print(f"      - {p}")

                subs = [s for s in data["subscribers"] if not s.startswith("_zros")]
                if subs:
                    print("    Subscribers:")
                    for s in sorted(subs):
                        print(f"      - {s}")

        node.running = False

    node.create_subscriber("_zros/graph", graph_callback, queue_size=0)
    # Wait 1.1s to allow nodes to broadcast their info (every 1s)
    node.create_timer(1.1, print_status_and_exit)
    try:
        node.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
