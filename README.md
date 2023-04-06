# **TAICHI (Tracking Algorithm for Imitation of Complex Human Inputs)**
<p align="center">
  <img src="./Images/Imagen1.png" height=200 />
</p>

This project presents the TAICHI algorithm. This algorithm is focused on the tracking of people to collect data for the Imitation Learning technique. TAICHI is composed of a detection of the significant points of the human arm and its extraction and mapping to the robot, a Gaussian filtering process to smooth the movements and filter the sensor noise and an optimization algorithm that seeks to obtain through the inverse kinematics of the model the configuration that is closest to the human one and that does not generate collisions with its environment or with itself.

# Installation
To be used on your device, follow the installation steps below.

**Requierements:**
- Python 3.10.0 or higher

> **Note**: The Adam Simulator works on Linux, Windows and Mac.

## Install miniconda (highly-recommended)
It is highly recommended to install all the dependencies on a new virtual environment. For more information check the conda documentation for [installation](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) and [environment management](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html). For creating the environment use the following commands on the terminal.

```bash
conda create -n taichi python==3.10.9
conda activate taichi
```

## Install from source
Firstly, clone the repository in your system.
```bash
git clone (link)
```

Then, enter the directory and install the required dependencies
```bash
cd taichi
pip install -r requirements.txt
```

# Usage
To use the TAICHI algorithm directly on the model of the robot and robotic arm presented in the paper, it is necessary to have both [MATLAB (R2022a)](https://es.mathworks.com/products/new_products/release2022a.html) and the [MujoCo](https://github.com/openai/mujoco-py) simulator installed. The simulator used in this work has been developed by our laboratory and all the information can be found in [ADAM Simulator](https://github.com/vistormu/adam_simulator). The algorithm has been tested and uses the specific packages for the [RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) camera. Feel free to use our algorithm to applied it in other models and robotic arms.

The code is divided in two files:

`MatlabCode` stores all the files to apply the algorithm directly in Matlab.

`PythonCode` stores all the files to apply the algorithm directly in Python.

### **MatlabCode**





# Citation
If you use this code, please quote our work :blush:

In construction :construction_worker: