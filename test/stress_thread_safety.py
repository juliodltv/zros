"""
Stress test: thread safety of pub_socket.

This reproduces the EXACT race condition observed in production:
- Thread A (zros spin): fires _zros/graph timer every 1s → send_multipart on pub_socket
- Thread B (external callback): fires rapidly → send_multipart on the SAME pub_socket

If both threads call send_multipart simultaneously, pyzmq may release the GIL
between frames, interleaving them into a 3-frame message on the receiver:

  Thread A sends: [b"_zros/graph",    graph_payload]
  Thread B sends: [b"my_topic",       data_payload ]

  Interleaved result at receiver: [b"_zros/graph", b"my_topic", graph_payload]
  → "too many values to unpack (expected 2)"

This is the scenario in detect_object.py:
  - ZROS spin thread fires _broadcast_graph_info() → pub_socket.send_multipart
  - ROS prompt_callback thread fires z_pub.publish() → SAME pub_socket.send_multipart

Run:
    uv run zroscore &
    uv run python test/stress_thread_safety.py
"""

import sys, time, threading
from zros import zNode

TOPIC    = "stress/thread_safety"
DURATION = 30

lock         = threading.Lock()
received     = 0
bad_frames   = 0  # captured from [zros] Unexpected frame count messages

original_print = print

def patched_print(*args, **kwargs):
    text = " ".join(str(a) for a in args)
    if "Unexpected frame count" in text:
        global bad_frames
        bad_frames += 1
    original_print(*args, **kwargs)

import builtins
builtins.print = patched_print


def external_thread_fn(node: zNode, pub):
    """
    Simulates a ROS callback thread (like prompt_callback) calling
    pub_socket.send_multipart from OUTSIDE the zros spin thread.
    Fires at ~200 Hz to maximize collision probability with the 1Hz graph timer.
    """
    deadline = time.time() + DURATION
    sent = 0
    while time.time() < deadline:
        pub.publish({"seq": sent, "data": b"x" * 1024})
        sent += 1
        time.sleep(0.005)  # 200 Hz
    original_print(f"[external thread] sent {sent} msgs")


def subscriber_fn():
    global received

    def cb(msg):
        global received
        with lock:
            received += 1

    node = zNode("stress_sub_ts")
    node.create_subscriber(TOPIC, cb, queue_size=0)
    deadline = time.time() + DURATION + 2
    while time.time() < deadline:
        node.spin_once(timeout=5)
    node.destroy_node()


if __name__ == "__main__":
    original_print(f"Running thread-safety stress test for {DURATION}s")
    original_print("Simulating concurrent pub_socket access from 2 threads\n")

    # Publisher node — its spin() will fire _zros/graph every 1s (Thread A)
    pub_node = zNode("stress_pub_ts")
    pub = pub_node.create_publisher(TOPIC)

    # Thread A: zros spin (fires _graph_pub every 1s on pub_socket)
    spin_thread = threading.Thread(target=pub_node.spin, daemon=True)
    spin_thread.start()

    # Subscriber
    sub_thread = threading.Thread(target=subscriber_fn, daemon=True)
    sub_thread.start()

    time.sleep(0.3)

    # Thread B: external thread using SAME pub_socket (like ROS prompt_callback)
    ext_thread = threading.Thread(target=external_thread_fn, args=(pub_node, pub))
    ext_thread.start()
    ext_thread.join()

    time.sleep(2)
    pub_node.running = False
    spin_thread.join(timeout=3)
    sub_thread.join(timeout=3)

    original_print(f"\n{'='*50}")
    original_print(f"SUMMARY")
    original_print(f"{'='*50}")
    original_print(f"Messages received:   {received}")
    original_print(f"Bad frames detected: {bad_frames}")
    if bad_frames > 0:
        verdict = "FAIL — frame interleaving detected (fix not applied)"
    else:
        verdict = "PASS — no frame interleaving"
    original_print(f"Verdict: {verdict}")
