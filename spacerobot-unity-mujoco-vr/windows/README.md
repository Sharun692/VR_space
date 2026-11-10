# Space Service Robot: MuJoCo Physics in Unity (Windows)

This package contains a simulation of a four-limbed service robot moving along the truss rail of a space station and servicing a cap. The physics is computed by MuJoCo 3.11 running inside Unity. Unity's own physics engine is not used: every contact force comes from MuJoCo, at a 1 ms time step.

MuJoCo is included in the package. Nothing needs to be installed apart from Unity.

## Contents

| Folder | Description |
| --- | --- |
| `SpaceRobot_Player` | Ready-to-run Windows program |
| `SpaceRobotMj` | Unity project |

## Requirements

- Windows 10 or 11, 64-bit
- Unity Hub, signed in with a Unity account and a Personal licence
- Unity Editor **6000.6.6f1**, with **Windows Build Support (Mono)**. To install this exact version, open the following link after installing Unity Hub: `unityhub://6000.6.6f1/d20c9ac777da`

## 1. Running the program

Run `SpaceRobot_Player\SpaceRobot_Service.exe`.

The first launch takes a few minutes while Unity prepares its shaders. Later launches start within seconds. The robot walks along the rail, then services the cap.

| Control | Action |
| --- | --- |
| Left drag | Orbit the camera |
| Right or middle drag | Pan |
| Mouse wheel | Zoom |
| F | Recentre on the robot |

## 2. Opening the project

1. In Unity Hub, select **Add → Add project from disk** and choose the `SpaceRobotMj` folder.
2. Open the project with version 6000.6.6f1. The first opening takes several minutes.
3. The scene `Assets/SpaceRobot/Scenes/SpaceRobot_Service_Mj.unity` opens automatically. If it does not, open it from the Project window.
4. Press **Play**. The robot walks along the rail.

The Console (**Window → General → Console**) should contain no errors.

## 3. Verifying the simulation

Four checks are available from the **SpaceRobot** menu, each writing its results to the Console. The first three run without Play mode; the editor does not respond while they run, about 40 s for the mission and 10–30 s for the others, and then returns to the service scene.

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

**SpaceRobot → Test body control** moves the body beyond the legs' reach and back, then moves and turns it while the two front limbs are driven and the rear limbs hold the rail, and finally pulls it out of reach and back with the hands held still. Expected last line:

```
BODY verdict=BODY FOLLOWS TARGET
```

**SpaceRobot → Test body and hands in Play mode** checks the components as an XR rig uses them. It starts Play mode, assigns stand-in objects as the headset and the two controllers, moves and turns the head, and stops Play mode after about 15 s. Use it to confirm the robot follows assigned transforms before connecting a headset. Expected last line:

```
PLAYTEST verdict=BODY AND HANDS FOLLOW
```

## 4. Building the program

Select **SpaceRobot → Build in-Unity MuJoCo Windows player**. The build takes under a minute and replaces the program in `SpaceRobot_Player`, whose folder then opens in Explorer. Close the program first if it is running.

## 5. Connecting a VR headset

The project provides limb and body control from tracked transforms and force feedback, but no headset support is configured. To add it:

1. In **Window → Package Manager**, install **XR Plugin Management** and **OpenXR**, and, if a ready-made rig is wanted, the **XR Interaction Toolkit**.
2. In **Edit → Project Settings → XR Plug-in Management**, enable **OpenXR** and add the interaction profile of the headset in use.
3. Add an XR Origin rig to the scene. The existing `Observer camera` can then be disabled.
4. On the `Mission` object:
   - In `SpaceRobotHandControl`, set **Hand** to a controller transform of the rig and **Limb** to the limb it should drive (0 FL, 1 FR, 2 RL, 3 RR). Add a second component for the other hand.
   - In `SpaceRobotBodyControl`, set **Body** to the transform that should pose the robot's body, such as the headset.
   - In `SpaceRobotHaptics`, set **Hand** to the same controller. Vibration then follows the contact force measured on that limb.

   The autonomous mission switches itself off as soon as a Hand or Body transform is in use, so that it does not also command the limbs and body.

### Hand control

A driven limb follows its target position, with the hand held palm-down. MuJoCo resolves any contact on the way; no Unity colliders are involved.

| Field | Meaning |
| --- | --- |
| `Hand` | Transform the limb follows |
| `Limb` | 0 FL, 1 FR, 2 RL, 3 RR |
| `Grip` | Finger closure, 0 open to 1 closed |
| `ReleaseRailLatch` | Releases the limb from the rail when control starts |
| `Stiffness` | Strength of the pull toward the hand (default 420) |
| `Damping` | Damping on that pull |
| `ForceLimit` | Maximum force the limb applies, in newtons |

At the default settings a limb 25 cm from its target reaches 1.6 mm from it within four seconds. Higher `Stiffness` tracks more closely; `Damping` should be raised with it to avoid overshoot.

### Body control

The robot's body takes the world position and rotation of the **Body** transform: the body origin moves to the transform's position, and the body's axes align with the transform's axes. The limbs still on the rail bend to carry the body there, and MuJoCo resolves all contact. Body and hand control can be combined, for example the headset on the body and the two controllers on the front limbs, with the rear limbs holding the rail.

| Field | Meaning |
| --- | --- |
| `Body` | Transform the body follows, in position and rotation |
| `OutOfReach` | True when the body has stopped short of the target because the legs on the rail cannot reach further (read only) |
| `PositionError` | Distance from the body to the target, in metres (read only) |
| `AngleError` | Angle between the body and the target, in degrees (read only) |

Moved 10 cm forward, 5 cm sideways and 8 cm up and turned 10°, the body settles within 6 mm and 0.3° of the target. A target beyond the legs' reach leaves the body at rest at the limit, with the rail grip held, and `OutOfReach` is set after about 1.5 s without progress. When the target comes back within reach, the body and any driven hands return to their targets.

From a script, `mission.Controller.SetExternalBody(position, rotation)` sets the target relative to the station, `ClearExternalBody()` releases it, and `SpaceRobotBodyControl.ToStationFrame` converts a Unity transform into that form.

### Force feedback

`SpaceRobotHaptics` converts a measured force into controller vibration. **From** selects the force: `LimbGrip`, everything the limb's fingers touch, including the rail, or `CapContact`, the cap alone. `FullScaleForce` is the force giving full amplitude and `Threshold` the force below which nothing is sent.

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

### Timing

The simulation advances once per rendered frame, by as many 1 ms steps as the frame lasted: eleven steps at 90 Hz, taking about 3 ms of the 11.1 ms frame on a current desktop processor.
