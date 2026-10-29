# Space service robot in Unity with MuJoCo: Windows

The robot walks along the truss rail of a space station and services a cap. The physics is computed by MuJoCo 3.11 running inside Unity, at a 1 ms time step. Unity's own physics engine is not used, and every contact force comes from MuJoCo.

MuJoCo is included in the package. Nothing needs to be installed apart from Unity.

The package was tested on Windows 11 with no MuJoCo or Python installed. The 150 s mission completes without errors both in the program and in Unity's Play mode, the built-in checks give the results listed in section 4, and the program builds from the editor.

## 1. Requirements

- Windows 10 or 11, 64-bit
- Unity **6000.6.6f1** with Windows Build Support (Mono) (`unityhub://6000.6.6f1/d20c9ac777da`)

## 2. Download

The Windows folder alone is downloaded with:

```powershell
git clone --filter=blob:none --sparse https://github.com/Sharun692/VR_space.git
cd VR_space
git sparse-checkout set spacerobot-unity-mujoco-vr/windows
cd spacerobot-unity-mujoco-vr\windows
```

The folder holds two folders.

| Folder | Contents |
| --- | --- |
| `SpaceRobot_Player` | Ready-to-run Windows program |
| `SpaceRobotMj` | Unity project |

## 3. Running the program

`SpaceRobot_Player\SpaceRobot_Service.exe` runs the mission. The robot walks along the rail and then services the cap. The first launch takes a few minutes while Unity prepares its shaders.

| Control | Action |
| --- | --- |
| Left drag | Orbit the camera |
| Right or middle drag | Pan |
| Mouse wheel | Zoom |
| F | Recentre on the robot |

## 4. The project

The Unity project is `SpaceRobotMj`. The service scene `Assets/SpaceRobot/Scenes/SpaceRobot_Service_Mj.unity` opens automatically. In Play mode the robot walks along the rail, with no errors in the Console.

### The scene

| Object | Contents |
| --- | --- |
| `SpaceRobot` | The robot and the station. Each body, joint, geometry and actuator of the MuJoCo model is a Unity object, with its MuJoCo settings in the Inspector. |
| `Mission` | The scripts that run the simulation, listed below |
| `Observer camera` | Desktop camera with the mouse controls of section 3 |
| `Sun light` | Scene lighting |
| `Global Settings` | `MjGlobalSettings`, with `UseRawGameObjectNames` ticked |

`MjScene`, which holds the compiled MuJoCo model, is created on entering Play mode and is not saved with the scene. The scene contains no rigidbodies and no colliders.

| Script on `Mission` | Role |
| --- | --- |
| `SpaceRobotMission` | Steps the physics and publishes force readings |
| `SpaceRobotHandControl` | Drives a limb from a tracked hand |
| `SpaceRobotHaptics` | Controller vibration from a measured force |
| `SpaceRobotCapture` | Writes a screenshot at a given simulated time |

The supporting code is in `Assets/SpaceRobot/Mj`. `MissionController` holds the gait, the cap sequence and the rail latches. `Stabilization` covers station keeping, attitude hold and limb Cartesian control. `ContactMeasurements` measures contacts each step, and `MjModelAccess` gives named access to the MuJoCo model and data.

Unticking **Autonomous Mission** on `SpaceRobotMission` keeps the robot holding the rail and skips the cap sequence, while station keeping, contact measurement and hand control continue. This is the mode for hand control.

`UseRawGameObjectNames` has to stay ticked. Without it every MuJoCo element is renamed, `robot` becomes `robot_0`, and lookups by name fail.

### Rebuilding the scene

The scene is generated from the model file `tools/spacerobot_high_fidelity.xml`. After the model is edited, **SpaceRobot → Build in-Unity MuJoCo scene** reimports it, applies the settings above, attaches the mission components and saves the scene. Changes made by hand to the scene are lost when it is rebuilt.

### Verifying the simulation

Two checks are available from the **SpaceRobot** menu. Each runs outside Play mode and writes its results to the Console. The mission check takes about 40 s and the limb test about 10 s.

**SpaceRobot → Run in-Unity mission validation** runs the full 150 s mission. Expected lines:

```
MISSION final_cap_angle_rad=0.9459...
MISSION final_stage=cap-task-complete
MISSION regrasp_failures=0
```

**SpaceRobot → Test external limb control** drives two limbs toward targets 25 cm away. Expected lines:

```
HAND limb=FR distance_before=0.2500 m  distance_after=0.0016 m
HAND limb=FL distance_before=0.2500 m  distance_after=0.0016 m
HAND verdict=ALL FOLLOW TARGET
```

## 5. Building the program

**SpaceRobot → Build in-Unity MuJoCo Windows player** builds the program in under a minute and replaces the contents of `SpaceRobot_Player`, whose folder then opens in Explorer.

## 6. VR interfaces

No XR packages or rig are included. The project provides limb control from a tracked hand and force feedback, both on the `Mission` object. `SpaceRobotHandControl` drives one limb from a transform, and a second component drives a second limb. `SpaceRobotHaptics` sends vibration to a controller. With **Autonomous Mission** ticked, the mission also commands the limbs.

**Run In Background** is enabled in the player settings, so the program keeps running when its window loses focus.

### Hand control

A driven limb follows its target position. MuJoCo resolves any contact on the way, and no Unity colliders are involved.

| Field | Meaning |
| --- | --- |
| `Hand` | Transform the limb follows |
| `Limb` | 0 FL, 1 FR, 2 RL, 3 RR |
| `Grip` | Finger closure, 0 open to 1 closed |
| `ReleaseRailLatch` | Releases the limb from the rail when control starts |
| `Stiffness` | Strength of the pull toward the hand (default 420) |
| `Damping` | Damping on that pull |
| `ForceLimit` | Maximum force the limb applies, in newtons |

At the default settings a limb 25 cm from its target reaches 1.6 mm from it within four seconds. A higher `Stiffness` tracks more closely, and `Damping` is raised with it to avoid overshoot.

### Driving a limb from a script

Targets are given in the station's frame, since the station drifts in orbit. Unity is Y up and MuJoCo is Z up.

```csharp
var mission = FindAnyObjectByType<SpaceRobotMission>();
var controller = mission.Controller;      // null until the first physics step
int limb = Rig.FR;

Vector3 p = handTransform.position;
V3 target = mission.Model.WorldToStationLocal(new V3(p.x, p.z, p.y));

controller.SetRailLatch(limb, false);           // release the rail
controller.SetExternalLimb(limb, target, grip); // grip 0 open to 1 closed

controller.ClearExternalLimb(limb);             // return the limb to the mission
controller.ClearAllExternalLimbs();
```

`SetExternalLimb` is called every frame for each driven limb, and `controller.IsExternal(limb)` reports whether a limb is driven. Limbs not driven keep their grip on the rail.

### Grasping

```csharp
int fingersOnCap = mission.Contacts.CapFingerCount(Rig.FR);
double capForce = mission.Contacts.CapContactForce;
double gripForce = mission.Contacts.GripForces[Rig.FR];
```

The mission treats a grasp as secure once two or more fingers touch the cap and the hand is within 20 mm of it. `controller.SetCapLatch(true)` then attaches the cap to the hand.

### Force feedback

`SpaceRobotHaptics` converts a measured force into controller vibration. **From** selects the force. `LimbGrip` is everything the limb's fingers touch, including the rail, and `CapContact` is the cap alone. `FullScaleForce` is the force giving full amplitude and `Threshold` the force below which nothing is sent.

Other devices can read the forces directly. `SpaceRobotMission.ForceSampleAvailable` is raised after every 1 ms physics step:

```csharp
var mission = FindAnyObjectByType<SpaceRobotMission>();
mission.ForceSampleAvailable += s => {
    // s.Time               simulated time, s
    // s.CapContactForce    force on the cap, N
    // s.GripForceFL, s.GripForceFR, s.GripForceRL, s.GripForceRR
    //                      force on each limb's fingers, N
    // s.CapFingerContacts  number of fingers touching the cap
};
```

The handler runs on Unity's main thread, so slow work in it slows the simulation.

## 7. Time stepping

The MuJoCo plug-in compiles the model with Unity's `Time.fixedDeltaTime` as its time step, which is 20 ms by default. `SpaceRobotMission` sets 1 ms before the model is compiled and enforces it on the compiled model as well.

The simulation advances once per rendered frame, by as many 1 ms steps as the frame lasted. At 90 Hz that is eleven steps, taking about 3 ms of the 11.1 ms frame on a current desktop processor. Each frame therefore shows the state at its own time.

## 8. Changes to the MuJoCo plug-in

The project includes a modified copy of `org.mujoco` 3.11.0 in `Packages/org.mujoco`. The model does not import or run correctly with the unmodified package, so the included copy is used rather than a separately installed one.

1. Body rotations given as `euler`, `axisangle`, `xyaxes` or `zaxis` were ignored, so two limbs imported unrotated. All four forms are now read.
2. Constraints using `site1` and `site2` were unsupported, and the import failed. The rail latches and the cap grasp use that form. It is now supported, and constraints can be engaged and released during simulation.
3. A call to `Object.GetInstanceID()`, which Unity 6 rejects, was replaced.
4. Scene objects were gathered in an undefined order, so results differed between runs. They are now sorted, and runs repeat exactly.
5. On first opening, the plug-in copied MuJoCo from a system installation and failed where none was present. This step is removed, since the project carries its own copy in `Assets/Plugins/x86_64`.
6. `MjScene.ManualStepping` lets a script step the physics once per rendered frame instead of from `FixedUpdate`.
