# ZROS — TODO

Items under active development go to GitHub Issues. This file tracks planned but not yet started work.

---

## Serialization

### `zros[msgpack]` — msgpack as optional serialization backend

Replace `pickle` with [msgpack](https://msgpack.org/) as an opt-in serialization format.

**Motivation:**
- `pickle` is Python-only — blocks interoperability with C++, Julia, JavaScript, etc.
- msgpack is faster and produces smaller payloads
- Cross-language support enables ZROS to act as a true middleware layer

**Proposed API:**
```python
node = zNode("my_node", serializer="msgpack")   # opt-in
node = zNode("my_node")                         # default: pickle (backwards compat)
```

**Notes:**
- Requires `pip install msgpack` or `zros[msgpack]`
- numpy arrays need special handling (`msgpack-numpy` or manual encoding)
- All nodes on the same network must use the same serializer

---

## Compatibility

### `zros[python2]` — bridge for Python 2 scripts

Allow legacy Python 2 processes to publish/subscribe to ZROS topics.

**Motivation:**
- Some robotics labs still run Python 2.7 code (older ROS1 stacks, legacy hardware drivers)
- ZROS should be able to bridge those scripts into a modern Python 3 environment

**Proposed approach:**
- A standalone Python 2 compatible script (`zros_bridge_py2.py`) that uses `zmq` (available for py2) and a simple JSON-based protocol (no pickle)
- Requires `zros[msgpack]` or a JSON serialization mode to be implemented first (pickle is not cross-version safe between py2/py3)
- The bridge script would be a thin shim — no class hierarchy, just raw ZMQ pub/sub with JSON encoding

**Notes:**
- Python 2 ZMQ bindings (`pyzmq`) still work on 2.7
- JSON is the safest common serializer between py2 and py3
- This depends on implementing the serialization backend first

---

## Network visualization

### Move `zros_graph.py` to main package

`future/zros_graph.py` is feature-complete. Integrate it as `zros_graph` CLI tool.

- Requires `networkx` and `matplotlib` as optional dependencies (`zros[graph]`)
- Register as entry point: `zros_graph = "zros.zros_graph:main"`
