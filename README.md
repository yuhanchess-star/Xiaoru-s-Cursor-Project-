# Dual-arm contact-rich insertion (Embodied Future)

Modular Cartesian control stack for a **cooperative peg-in-hole / mating** task. Both arms run an admittance loop on wrist F/T, stay coupled through a relative-pose constraint, and fall back to a spiral + dither search when insertion stalls. A latching safety monitor bounds force, torque, and search radius.

The default backend is a **numpy mock plant** (no ROS, no Bullet) so the skill can be unit-tested offline. Optional adapters exist for PyBullet primitives and MoveIt.

## Layout

```
dual_arm_insertion/
  dual_arm/
    config.py          # geometry, gains, search, safety limits
    state_machine.py   # Approach → Contact → Alignment → Insertion → Secured
    controller.py      # CartesianAdmittanceController + DualArmInsertionController
    bimanual.py        # relative-pose split + complementary-force bias
    search.py          # Archimedean spiral + orthogonal dither
    safety.py          # wrench / dt / search-radius envelope
    simulation.py      # MockDualArmWorld, PyBulletPlant, MoveItPlant stub
    run.py             # closed-loop test harness
  tests/test_insertion.py
```

## Control law

Each arm integrates a decoupled 6-DOF admittance (socket frame: `[x, y, z, roll, pitch, yaw]`):

```
M v̇ + D v + K (x − x_ref) = F_meas − F_des
```

- **Arm A (socket / fixture)** — high `K`, holds the hole with light shock compliance.
- **Arm B (peg)** — soft in the search plane `(x, y, rx, ry)`, firmer along insertion `z`.
- **Bimanual split** — a relative pose `x_B − x_A` is tracked with weight `α ≈ 0.85` on the peg so the fixture barely moves. Complementary (internal) force `½(F_A + F_B)` is regulated toward zero so the arms do not wrestle through the workpiece.
- **Search** — on `ALIGNMENT`, an Archimedean spiral plus a small Lissajous dither is added to the XY reference. Search is *overlayed* on the force loop; it does not replace it.
- **Safety** — any sample with `|F|`, `|T|`, `F_xy`, `|Fz|`, or search radius above the configured bound latches `FAULT` and freezes the admittance integrators.

## Run

```bash
cd dual_arm_insertion
pip install -r requirements.txt
python -m dual_arm
python -m dual_arm --plot
python -m dual_arm --backend pybullet --gui   # needs: pip install pybullet
pytest -q
```

Expected mock demo: the peg starts several millimetres off-axis (outside the chamfer capture region), contacts the plate, spirals until the mouth is found, then inserts to the target depth and reports `SECURED`.

## Hooking up a real dual-arm cell

Subclass `DualArmPlant` (see `MoveItPlant` in `simulation.py`):

1. Servo both end-effectors to the 6-vector pose commands at `cfg.dt` (200 Hz Cartesian servo or MoveIt Servo).
2. Return measured EE poses and wrist F/T, **expressed in the socket frame**.
3. Keep the same `DualArmInsertionController` loop:

```python
from dual_arm import DualArmInsertionController

ctrl = DualArmInsertionController()
pose_a, pose_b = plant.reset()
ctrl.reset(pose_a, pose_b)
ctrl.start()
while not ctrl.done:
    cmd = ctrl.step(pose_a, pose_b, wrench_a, wrench_b)
    pose_a, pose_b, wrench_a, wrench_b = plant.step(cmd.pose_a, cmd.pose_b)
```

Tune `ControlConfig` stiffness / `desired_insert_force` / `SearchConfig.spiral_pitch` to the hardware F/T range before raising `SafetyLimits`.
