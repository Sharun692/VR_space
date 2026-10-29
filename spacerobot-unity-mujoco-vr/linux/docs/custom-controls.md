# Adding custom controls

## How a limb is driven

A limb is driven by a **target position**. `MissionController` computes the joint torques that move the hand toward it, and MuJoCo resolves any contact on the way. The autonomous mission moves the arms the same way. Hand control supplies the target instead.

Contact is never resolved in Unity, so no colliders are involved.

Targets are in the **station's frame**, since the station drifts in orbit. `MjModelAccess.WorldToStationLocal` converts from world coordinates.

## From a tracked hand

`SpaceRobotHandControl`, on the **Mission** object, drives a limb from a transform. Each component drives the single limb it names; add a second component for a second limb, one per hand.

| Field | Meaning |
| --- | --- |
| `Hand` | Transform to follow, typically the XR controller |
| `Limb` | 0 FL, 1 FR, 2 RL, 3 RR |
| `Grip` | Finger closure, 0 open to 1 closed |
| `ReleaseRailLatch` | Unlatches the limb from the rail when control starts |
| `ForceLimit` | Ceiling on the force the limb can push with, newtons |
| `Stiffness` | How hard the limb is pulled toward the hand |
| `Damping` | Damping on that pull |

With the autonomous mission off, a limb 25 cm from its target closed to 1.6 mm in four seconds, and two limbs driven at once reached 1.6 mm each. With the mission left running the same limb reached only 4 cm, because the mission commands it too.

`Stiffness` sets how closely the limb tracks: 100 settled 6.9 mm out, the default 420 gave 1.6 mm, and 1200 gave 0.5 mm. Raise `Damping` alongside it to stop overshoot. `ForceLimit` bounds the force against contact and makes little difference in free space, where 20 N and 300 N both tracked to within 1.8 mm.

A limb under external control is driven by this Cartesian term alone. Its joint actuators are held at their measured angles so they produce no force of their own, leaving one control path to tune.

## From a custom script

```csharp
var mission = FindAnyObjectByType<SpaceRobotMission>();
var controller = mission.Controller;      // null until the first physics step
var model = mission.Model;

int limb = Rig.FR;

// Unity is Y up, MuJoCo is Z up.
Vector3 p = handTransform.position;
V3 target = model.WorldToStationLocal(new V3(p.x, p.z, p.y));

controller.SetRailLatch(limb, false);           // release the rail
controller.SetExternalLimb(limb, target, grip); // grip is 0 to 1

controller.ClearExternalLimb(limb);             // return the limb to the mission
controller.ClearAllExternalLimbs();             // or return all of them
```

`SetExternalLimb` is called each `FixedUpdate`, once per limb being driven. Any number of limbs can be taken over at once, and `controller.IsExternal(limb)` reports whether a given limb is. The rest stay under the mission and keep their grip on the rail.

## Grasping

`Grip` closes the fingers, and the resulting contact can be read back:

```csharp
int fingersOnCap = mission.Contacts.CapFingerCount(Rig.FR);
double force = mission.Contacts.CapContactForce;
```

The mission counts a grasp as secure once two or more fingers touch the cap and the hand is within 20 mm of it. `controller.SetCapLatch(true)` then attaches the cap to the hand.

## Force feedback

`ForceSampleAvailable` fires once per physics substep, giving a device samples at 1 kHz rather than frame rate:

```csharp
void OnEnable()  => mission.ForceSampleAvailable += OnForce;
void OnDisable() => mission.ForceSampleAvailable -= OnForce;

void OnForce(SpaceRobotMission.ForceSample s) {
    // s.Time              simulated seconds
    // s.CapContactForce   newtons on the cap
    // s.GripForceFR       newtons on that limb's fingers
    // s.CapFingerContacts fingers in contact
}
```

The handler runs on Unity's main thread. Slow work there stalls the simulation and the frame rate with it.

`SpaceRobotHaptics` maps a measured force to controller vibration. `From` chooses which: `LimbGrip` is everything that limb's fingers are touching, rail and cap alike, and `CapContact` is the service cap on its own. `Limb` picks the limb for `LimbGrip`. `FullScaleForce` is full amplitude, `Threshold` the force below which nothing is sent, `PulseInterval` the gap between pulses.

`LimbGrip` is the default because it covers holding the rail. Over the first 40 s of the mission, while the robot is walking, grip force on the front-right limb peaked at 322 N and averaged 172 N, and sat above the 2 N threshold 78% of the time. Cap contact force over the same period was zero, so `CapContact` gives no feedback until the robot reaches the cap.

A resistive device uses the same event.

## Switching the mission off

Untick `AutonomousMission` on the Mission object. The robot holds its grip on the rail and the cap sequence never starts, while station keeping, contact measurement and external limb control continue.
