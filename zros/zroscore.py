import sys
import errno
import zmq

def main():
    context = zmq.Context()
    frontend = context.socket(zmq.XPUB)
    backend = context.socket(zmq.XSUB)

    # Try to bind before starting the proxy.
    try:
        frontend.bind("tcp://*:5555")
        backend.bind("tcp://*:5556")
    except zmq.error.ZMQError as e:
        frontend.close()
        backend.close()
        context.term()
        if e.errno == errno.EADDRINUSE:
            print("zroscore is already running. Using existing instance.")
            sys.exit(0)
        print(f"zroscore failed to bind: {e}")
        sys.exit(1)

    print("zroscore started...")
    print("Subscribers connect to port 5555")
    print("Publishers connect to port 5556")

    try:
        zmq.proxy(frontend, backend)
    except KeyboardInterrupt:
        print("Stopping zroscore...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        frontend.close()
        backend.close()
        context.term()

if __name__ == "__main__":
    main()
