# SUMO-V2I/V2X

A gym environment to simulate Vehicle-to-Vehicle and Vehicle-to-Infrastructure interactions.

# Installation

We recommend using a python virtual environment. Follow the below steps to get the virtual environment.

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-py310_23.5.2-0-Linux-x86_64.sh
bash Miniconda3-py310_23.5.2-0-Linux-x86_64.sh
conda env create -f environment.yml
conda activate sumo
```

Install the package by following the below steps
```
git clone 
cd sumo/
python setup.py install
pip install -e
```

Check your installation

```python
import sumo
import gymnasium as gym

viewSize = 20
env = gym.make('sumo/v2i-v0', view_size=viewSize)
```