import argparse
import time
from zros import zNode

class ZrosStatus(zNode):
    def __init__(self):
        super().__init__(f"zros_status_{int(time.time())}")
        self.create_subscriber("_zros/graph", self.graph_callback, queue_size=0)
        self.nodes = {}

    def graph_callback(self, payload):
        name = payload.get("name")
        if name and name != self.name:
            self.nodes[name] = {
                "publishers": payload.get("publishers", []),
                "subscribers": payload.get("subscribers", [])
            }

    def print_status_and_exit(self):
        if not self.nodes:
            print("No active nodes found. (Node graph discovery requires _zros/graph broadcasting)")
        else:
            for name, data in sorted(self.nodes.items()):
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
        
        self.running = False

def main():
    parser = argparse.ArgumentParser(description="Live status of ZROS nodes and topics.")
    parser.parse_args()
    
    node = ZrosStatus()
    # Wait 1.1s to allow nodes to broadcast their info (every 1s)
    node.create_timer(1.1, node.print_status_and_exit)
    
    try:
        node.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
