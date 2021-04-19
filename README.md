# Drivers for my 2 alvium cameras

## Remarks
1. Instead a typicl `config.yaml` file, you set up your config directly in the `scripts/*.py` files

## How to run
###just 1 camera
To run **just 1 camera** simply run in terminal (or in debug PyCharm) `roslaunch pp_alvium_driver rgb_only.launch` or `roslaunch pp_alvium_driver nir_only.launch`

###2 cameras simultaneously
To run **2 cameras simultaneously**:
    1. run `roslaunch pp_alvium_driver rgb_only.launch` in a terminal.
    2. Then open another terminal, and run `conda activate 4ros1`
    3. with the activated conda's `4ros1` run `roslaunch pp_alvium_driver nir_only.launch`

The whole point is that I could not manage to use alvium's `multithreading.py` to get the desired frequencies of both cameras. Things start breaking up when the nir camera achieves fps > 20 Hz