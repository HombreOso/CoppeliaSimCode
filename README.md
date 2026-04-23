# CoppeliaSim Python: Two Ways to Run Code

This folder contains examples for working with CoppeliaSim from Python.

There are two different ways to run Python with CoppeliaSim:

1. **External Python client**
2. **Python script inside CoppeliaSim**

They look similar, but they run in different environments and are not interchangeable.

## 1. External Python Client

An external Python client runs in your normal Python interpreter from:

- a terminal
- VS Code / Cursor
- another IDE

This Python process connects to CoppeliaSim through the Remote API.

Typical pattern:

```python
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require('sim')
```

### When to use this

Use an external Python client when:

- you want to use normal Python packages such as `numpy`, `scipy`, or `matplotlib`
- you want to debug in your IDE
- you want to run scripts directly with `python your_script.py`
- you want heavier math or optimization outside the simulator

### How to run it

1. Open CoppeliaSim.
2. Make sure the **ZMQ Remote API** add-on is available/running.
3. Open the scene you want to control.
4. In a terminal in this folder, run:

```bash
python warm_start_Newton_Raphson.py
```

You can also run the same file from Cursor, VS Code, or another IDE using your normal Python interpreter.

### Important notes

- CoppeliaSim must usually already be open.
- The script uses your system Python environment.
- Required Python packages must be installed in that environment.
- These scripts are usually plain top-to-bottom Python programs.

## 2. Python Script Inside CoppeliaSim

A Python script inside CoppeliaSim runs inside the simulator itself.

Typical pattern:

```python
sim = require('sim')
simIK = require('simIK')

def sysCall_init():
    pass
```

### When to use this

Use an in-simulator Python script when:

- the logic should live inside the scene
- you want the script to run with the simulation lifecycle
- you want to use `sysCall_init`, `sysCall_actuation`, and `sysCall_cleanup`

### How to run it

1. Open CoppeliaSim.
2. Add or open a script attached to the scene or an object.
3. Set the script language to **Python**.
4. Paste in the script contents.
5. Start the simulation.

For callback-based scripts, code in `sysCall_init()` runs when the simulation starts.

### Important notes

- This is **not** the same as running `python file.py`.
- The available Python packages may be more limited than your normal Python environment.
- You should usually use CoppeliaSim callbacks such as:
  - `sysCall_init()`
  - `sysCall_actuation()`
  - `sysCall_sensing()`
  - `sysCall_cleanup()`

## Why Copy-Paste Often Fails

A script written for one mode often fails in the other mode.

For example, this is an **external** script:

```python
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require('sim')
simIK = client.require('simIK')
```

If you paste that directly into a CoppeliaSim script and press Run, it may fail because:

- `RemoteAPIClient` is for an external connection
- the script is missing `sysCall_*` callbacks
- the script may rely on packages not available in CoppeliaSim's embedded Python
- the script may be pasted into a Lua script instead of a Python script

## Files in This Folder

- `warm_start_Newton_Raphson.py`  
  External Python client. Run it from a terminal or IDE.

- `warm_start_Newton_Raphson_coppelia.py`  
  CoppeliaSim-side Python version intended to be pasted into a Python script in CoppeliaSim.

## Quick Rule of Thumb

- If the script starts with `RemoteAPIClient`, run it from a terminal or IDE.
- If the script starts with `require('sim')`, it is meant for CoppeliaSim's internal scripting environment.
