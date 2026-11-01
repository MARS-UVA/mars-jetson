Training setup steps:

1. `cd src/obstacle_detection/model_training`
2. `./clone_synmars.sh`
3. `cd` into train-SynMars
4. `python3 -m pip install -r requirements.txt`
5. `python3 prepare_synmars.py`

Training:

1. Change and run `./test_train.sh` for modifiable weights
2. Run `./real_train.sh --device auto` for training
3. (optional) run `python3 evaluate_unet.py` to see image generated masks

Simulation:

1. Run `./view_obstacle_overlay.py` to see 1-1 obstacle overlay after running Gazebo.