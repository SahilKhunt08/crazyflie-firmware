# Setup Instructions

The official instructions for downloading the dependencies are [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#flashing).
The official instructions for making the firmware and uploading it to the Crazyflie are [here](https://www.bitcraze.io/documentation/tutorials/getting-started-with-development/). 
Flashing requires the Crazyflie Client to be set up in Python. We provide .yaml files to set up a Python Conda
environment below, but the official instructions are given [here](https://github.com/bitcraze/crazyflie-clients-python/blob/master/docs/installation/install.md). 

## Installing Git LFS
Instructions on how to set up Git LFS are given [here](https://docs.github.com/en/repositories/working-with-files/managing-large-files/installing-git-large-file-storage).

## Cloning the Repo

The Crazyflie firmware repo has git submodule dependencies meaning that it requires other public github repos to make 
the firmware work. To get these dependencies, use the following command:

```shell
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
```
If you have already cloned the repo without the `--recursive` command, you will need to run the two following commands:
```shell
git submodule init
git submodule update
```

## Setting up Python
We provide a file to set up a conda environment with the dependencies required for flashing the Crazyflie drone. 
This file is in the root directory and can be loaded using the following command:
```shell
conda env create -f crazyflie_<os>_py311.yml
```
Where `os` can be replaced with [linux, mac]. Afterward, you may activate the environment using:
```shell
conda activate crazyflie
```

## Controller

And explanation of the controllers that are available are given [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/). 
Instructions on how to integrate a controller into the Crazyflie code is given by this [article](https://www.bitcraze.io/2023/02/adding-an-estimator-or-controller/).
