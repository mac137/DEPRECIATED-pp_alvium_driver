# Drivers for my Alvium USB cameras

## How to run
### 1 camera
To run **just 1 camera** simply run in a terminal `roslaunch pp_alvium_driver rgb_only.launch` or `roslaunch pp_alvium_driver nir_only.launch`. The launch files should also work in your IDEs (tested on PyCharm Pro).

### 2 cameras simultaneously
To run **2 cameras simultaneously**:
1. run `roslaunch pp_alvium_driver rgb_only.launch` in a terminal
2. then open another terminal, and run `conda activate <<virtual env>>`
3. in the activated conda's `<virtual env>>` run `roslaunch pp_alvium_driver nir_only.launch`

The whole point for the virtual environemnt is that I could not manage to use alvium's `VimbaPython/Examples/multithreading_opencv.py` to get the desired frequencies for both cameras. Things start breaking up when my second camera wants to achieve fps > 20 Hz


## Remarks
1. Instead of a typical `config.yaml` file, you set up your config directly in the `scripts/*.py` files. Enter your camera ID as `cam_id`, desired frequency `frequency` and the camera calibration file `yaml_fname` 
2. Install `Vimba_4_2` to a desired python as instructed by `VimbaPython/Install.sh`
3. Make sure that `GENICAM_GENTL64_PATH` points to `path/2/vimba/Vimba_4_2/VimbaUSBTL/CTI/x86_64bit`