# robotlib — MATLAB Compiled Python Package

This package provides Python access to MATLAB functions via MATLAB Compiler SDK.

---

## Contents

This folder contains the compiled Python package:

```
robotlib/
├── __init__.py
├── robotlib.ctf
├── setup.py
```

---

## 1. Prerequisites for Deployment

### MATLAB Runtime

Ensure that **MATLAB Runtime R2024b** is installed.  
You can either:

- From MATLAB prompt:
  ```matlab
  >> mcrinstaller
  ```

- Or download it manually (Windows version) from:

  https://www.mathworks.com/products/compiler/mcr/index.html

> You **must** have Administrator privileges to install MATLAB Runtime.

---

### Python Version

Make sure you're using **Python 3.9**, **3.10**, **3.11**, or **3.12** (Windows only).  
Use the same architecture (64-bit) as the MATLAB Runtime.

---

## 2. Installing the `robotlib` Package

### A. Open Terminal (or Anaconda Prompt), then:

1. Navigate to the folder containing `setup.py` and the `robotlib` folder.

   ```bash
   cd path/to/for_testing  # or for_redistribution_files_only
   ```

2. Install the package using `pip`:

   - Standard install:
     ```bash
     python -m pip install .
     ```

   - Install to user directory (if lacking admin rights):
     ```bash
     python -m pip install --user .
     ```

   - Install to a custom location:
     ```bash
     python -m pip install --prefix="your/install/path" .
     ```

   Refer to: https://docs.python.org/3/installing/index.html for details.

---

## 3. Using the `robotlib` Package

### Step-by-step example:

```python
import robotlib
import numpy as np


rob = robotlib.initialize()
print(dir(rob))  # 列出所有可用函數
robot_path = r'path\ur10e_HandGuide.urdf'  # Note: The URDF file should not include visualization elements.
robot = rob.import_robot(robot_path)
print(robot)

tform1 = np.array(rob.forward_kinematic(robot, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), "tool0"))  # 檢查正向運動學
print("工具末端位姿 (tform):", tform1)
rob.terminate()

```

> ℹUse `dir(matlab_instance)` to inspect available functions inside the package.

---

## 🛠 Troubleshooting

- If you get an `AttributeError`, double-check that:
  - Your MATLAB function was added to the compilation project
  - The correct function name is being used
- If MATLAB Runtime is not installed or not compatible, your script will raise runtime errors.
- You can check required toolboxes via `requiredMCRProducts.txt`.

---

## Resources

- MATLAB Runtime: https://www.mathworks.com/products/compiler/matlab-runtime.html  
- Python Install Options: https://docs.python.org/3/installing/index.html  
- MATLAB Compiler SDK Docs: [Distribute Applications](https://www.mathworks.com/help/compiler_sdk/)
