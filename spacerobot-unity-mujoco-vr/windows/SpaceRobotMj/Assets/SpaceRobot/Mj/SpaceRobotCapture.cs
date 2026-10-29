using System;
using UnityEngine;

namespace SpaceRobot {

// Optional diagnostic: -capture <path> [-captureAt <seconds>] writes one frame
// and exits. Used for verifying a build without a visible desktop session.
public class SpaceRobotCapture : MonoBehaviour {
  private string _path;
  private double _at = 20.0;
  private SpaceRobotMission _mission;
  private bool _done;

  private void Start() {
    var args = Environment.GetCommandLineArgs();
    for (int i = 0; i < args.Length - 1; i++) {
      if (args[i] == "-capture") _path = args[i + 1];
      if (args[i] == "-captureAt") double.TryParse(args[i + 1], out _at);
    }
    _mission = FindAnyObjectByType<SpaceRobotMission>();
    if (_path == null) enabled = false;
  }

  private void LateUpdate() {
    if (_done || _mission == null || _mission.SimulationTime < _at) return;
    _done = true;
    ScreenCapture.CaptureScreenshot(_path);
    Debug.Log($"CAPTURE written {_path} at sim {_mission.SimulationTime:F2} s, stage {_mission.Stage}");
    Invoke(nameof(Finish), 1.0f);
  }

  private void Finish() => Application.Quit();
}
}
