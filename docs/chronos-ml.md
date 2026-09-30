# Chronos autopilot on Glitch MK2

Behavioral cloning from [Deep-Learning-Chronos](https://github.com/efieldsUCA/Deep-Learning-Chronos):
drive laps with the Xbox controller while the robot records camera frames and your
commands, train `ChronosResNetLite` on a computer with a GPU, then let the model drive.

The model outputs `(steering, throttle)` in `[-1, 1]`. On Glitch these map to
`angular.z = steering * max_angular_vel` and `linear.x = throttle * max_linear_vel`
(`config/chronos.yaml`). Keep those values and the image size the same between
recording and driving.

## 1. Setup (Pi)

```console
sudo apt install ros-jazzy-camera-ros ros-jazzy-cv-bridge
pip install --break-system-packages torch  # CPU wheel, only needed for pilot mode
```

## 2. Record

```console
ros2 launch glitchmk2_bringup glitchmk2_launch.py      # terminal 1: base, joy, teleop
ros2 launch glitchmk2_bringup chronos_launch.py mode:=record   # terminal 2
```

Press **A** to start and stop a session, and drive with LB held as usual. Frames are
only saved while the robot is commanded to move. Each session lands in
`~/glitchmk2_data/<YYYY-MM-DD-HH-MM-SS>/` as `images/*.jpg` plus `labels.csv`.

Tips: drive smoothly, record several laps in both directions, and include recoveries
(drift toward an edge, then correct) so the model learns to come back.

## 3. Train (server/laptop)

Copy the session folder into `Deep-Learning-Chronos/data/`, then in
`autopilot_architectures/chronoslearn.py` switch the model to the one the robot runs:

```python
from autopilot_architectures.Chronos_Res_Net_Lite import ChronosResNetLite
...
model = ChronosResNetLite().to(DEVICE)
```

```console
python autopilot_architectures/chronoslearn.py <YYYY-MM-DD-HH-MM-SS>
```

Copy the resulting `best_model.pth` to the Pi, e.g. `~/glitchmk2_models/pilot.pth`.

## 4. Drive

```console
ros2 launch glitchmk2_bringup chronos_launch.py mode:=pilot model_path:=~/glitchmk2_models/pilot.pth
```

The pilot starts **paused**. **Start/Menu** toggles it, **B** pauses it, and holding
**LB** hands control back to teleop while held. It also pauses if camera frames stop.
`throttle_limit` caps forward speed; start low.

Button numbers assume the Xbox mapping in `gamepad.yaml` (LB = 4). Check yours with
`ros2 topic echo /joy` and edit `config/chronos.yaml` if they differ.
