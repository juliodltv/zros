import zmq

def main():
    context = zmq.Context()

    # Socket facing clients (Subscribers)
    frontend = context.socket(zmq.XPUB)
    frontend.bind("tcp://*:5555")  # Subscribers connect here (standard port)

    # Socket facing services (Publishers)
    backend = context.socket(zmq.XSUB)
    backend.bind("tcp://*:5556")   # Publishers connect here

    print("zroscore started...")
    print("Subscribers connect to port 5555")
    print("Publishers connect to port 5556")

    try:
        # Start the proxy
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
