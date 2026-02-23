# ZROS Troubleshooting (ZROScore)

When you try to run `uv run zroscore` and you get an error like this:

```text
zmq.error.ZMQError: Address already in use (addr='tcp://*:5555')
```

It means that ports `5555` or `5556` (which ZROS uses by default for subscribers and publishers) are occupied by a previous ZROS or Python process that wasn't closed correctly.

## Commands to free the ports

You can use any of the following methods from the terminal to kill the process that is using the ports.

### Method 1: Using `fuser` (Recommended and fastest)

This command will immediately find and kill the process using TCP ports 5555 or 5556.

```bash
fuser -k 5555/tcp
fuser -k 5556/tcp
```
*(If it says permission denied, add `sudo` at the beginning: `sudo fuser -k 5555/tcp`)*

### Method 2: Closing all ZROSCore and Python processes

If the error is caused specifically because the ZROS broker is still running in the background:

```bash
pkill -f zroscore
```

If your `pegasus_bridge.py` or `pegasus_viva.py` scripts got hung (zombies), you can force close all python processes (this will close all active simulations):

```bash
pkill -f python
```

### Method 3: Using `lsof` to find the PID and then `kill`

If you want to see exactly what is occupying the port before closing it:

1. Find the process ID (PID):
```bash
lsof -i :5555
```
*(Look for the number in the `PID` column)*

2. Kill the process using its PID (replacing `<PID>` with the number):
```bash
kill -9 <PID>
```
