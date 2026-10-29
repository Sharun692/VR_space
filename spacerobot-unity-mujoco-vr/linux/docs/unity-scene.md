# The Unity scene

## Opening the project

Unity 6000.6.6f1. Extract `packages/spacerobot-unity-mujoco-project.zip` and open the `SpaceRobotMj` folder.

MuJoCo does not need to be installed. `Assets/Plugins/x86_64` holds `libmujoco.so` for Linux and `mujoco.dll` for Windows, each enabled for the editor and for player builds on its own platform.

Open `Assets/SpaceRobot/Scenes/SpaceRobot_Service_Mj.unity` and press Play. The robot walks the rail and opens the service cap. The mission lasts 150 seconds.

The first Play after opening the project is slow while Unity compiles its shaders.

## What is in the scene

**SpaceRobot** — the robot and the station. Each body, joint, geometry and actuator from the MuJoCo model is a Unity object, with its MuJoCo settings in the inspector.

**Mission** — the scripts that run the simulation:

| Script | Role |
| --- | --- |
| `SpaceRobotMission` | Steps the physics, publishes force readings |
| `SpaceRobotHaptics` | XR controller vibration |
| `SpaceRobotHandControl` | Drives a limb from a tracked hand |
| `SpaceRobotCapture` | Writes a screenshot, for checking a build without a desktop |

Untick `AutonomousMission` on `SpaceRobotMission` to hold the robot on the rail and skip the cap sequence. That is the mode for hand control.

Also in the scene: **Observer camera**, **Sun light**, and **Global Settings**, which carries `MjGlobalSettings`. `MjScene` is created on entering Play mode and is not saved.

## Rebuilding the scene

The scene is generated from the model file. After editing `tools/spacerobot_high_fidelity.xml`, regenerate it with **SpaceRobot → Build in-Unity MuJoCo scene**, which reimports the model, applies the settings below and saves the scene.

**SpaceRobot → Build in-Unity MuJoCo Linux player** and **SpaceRobot → Build in-Unity MuJoCo Windows player** build players. Both runtimes are in `Assets/Plugins/x86_64` and are bundled into the build automatically.

## Checks in the editor menu

| Command | What it does |
| --- | --- |
| **Run in-Unity mission validation** | Runs the 150 s mission headless and logs the final cap angle, stage, latch and contact counts. Reproduces the figures in `validation/in-unity-run-metrics.json` |
| **Test external limb control** | Hands a limb to external control, drives it 25 cm and reports how close it got |
| **Import MJCF and benchmark** | Imports the model and times `mj_step` on its own |

Each writes its results to the Unity log and exits, so they also run from the command line:

```
Unity -batchmode -nographics -quit -projectPath <project> \
      -executeMethod MjMissionValidation.Run -logFile -
```

## Required settings

The scene builder applies both. They matter when assembling a scene without it.

**Timestep.** The plug-in overwrites the model's timestep with `Time.fixedDeltaTime`. `SpaceRobotMission` sets it to keep MuJoCo at 1 ms; Unity's default of 20 ms is too coarse for contact.

**`UseRawGameObjectNames`**, on the **Global Settings** object, must be ticked, or every MuJoCo element is renamed — `robot` becomes `robot_0` — and lookups by name fail.

## Stepping

The plug-in updates all 180 Unity objects after every MuJoCo step, which is too costly at 1 kHz.

`SpaceRobotMission` therefore runs eleven 1 ms steps per frame and lets Unity update once. `PhysicsSubsteps` sets the count; eleven is about one 90 Hz frame.

## Reading forces

`SpaceRobotMission.Contacts`, refreshed every physics step:

| Member | Meaning |
| --- | --- |
| `CapContactForce` | Total force on the service cap, newtons |
| `GripForces[leg]` | Force on a limb's fingers |
| `CapFingerCount(leg)`, `RailFingerCount(leg)` | Fingers in contact |

Limbs are `Rig.FL` 0, `Rig.FR` 1, `Rig.RL` 2, `Rig.RR` 3.

For a continuous feed use the event instead — see [custom-controls.md](custom-controls.md).

## The MuJoCo plug-in

The bundled `org.mujoco` carries five changes for this model, so use the copy in the project rather than installing the package separately.

1. Body rotations given as `euler`, `axisangle`, `xyaxes` or `zaxis` were ignored, so two legs of this model imported unrotated. All four forms are now read.
2. Constraints using `site1`/`site2` were unsupported, so the import failed. The rail latches and the cap grasp use that form, and it is now supported.
3. A call to `Object.GetInstanceID()`, which Unity 6 rejects, was replaced.
4. Scene objects were gathered in an undefined order, which made results differ between runs. They are now sorted, and runs repeat exactly.
5. On first open the plug-in tried to copy MuJoCo from a system install, and failed with an error where none was present. It is removed, since the project carries its own copy.
