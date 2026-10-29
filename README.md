# VR_space

A four-limbed service robot walks along the truss rail of a space station and opens a service cap. The physics is computed by MuJoCo 3.11 running inside Unity through the `org.mujoco` plug-in. Unity's own physics engine is not used, and every contact force comes from MuJoCo at a 1 ms time step. The autonomous mission was first written in Python against the same model and has been rewritten in C# to run in the same process.

The work is packaged separately for each operating system.

| Folder | Contents |
| --- | --- |
| [`spacerobot-unity-mujoco-vr/windows`](spacerobot-unity-mujoco-vr/windows) | Windows program, Unity project, hand control and force feedback. Tested on Windows 11 without MuJoCo or Python installed. |
| [`spacerobot-unity-mujoco-vr/linux`](spacerobot-unity-mujoco-vr/linux) | Linux program, Unity project, documentation of the scene and controls, and validation against the Python reference. |

Each folder has its own README with the steps to run the program, open the project and check the results.

## Requirements

Unity **6000.6.6f1** (`unityhub://6000.6.6f1/d20c9ac777da`).

MuJoCo is included in both packages and does not need to be installed.
