using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using System.Xml;
using Mujoco;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Debug = UnityEngine.Debug;

public static class MjImportBenchmark {

  [MenuItem("SpaceRobot/Import MJCF and benchmark")]
  public static unsafe void Run() {
    if (!BuildSpaceRobotMjScene.ConfirmSceneReplace()) return;
    var mjcfPath = Path.GetFullPath(Path.Combine(Application.dataPath, "../tools/spacerobot_high_fidelity.xml"));
    if (!File.Exists(mjcfPath)) throw new FileNotFoundException(mjcfPath);

    // MuJoCo's own timestep must survive the import; the plug-in otherwise
    // substitutes Unity's fixedDeltaTime.
    Time.fixedDeltaTime = 0.001f;

    EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

    var xml = new XmlDocument();
    xml.Load(mjcfPath);
    var importer = new MjcfImporter();
    var root = importer.ImportXml(xml, "SpaceRobot");

    foreach (var nm in new[] { "RL_yaw_body", "FL_yaw_body" }) {
      var t = GameObject.Find(nm);
      if (t != null) Debug.Log($"BENCH afterimport {nm} localEuler={t.transform.localEulerAngles} localRot={t.transform.localRotation}");
      else Debug.Log($"BENCH afterimport {nm} NOT FOUND");
    }

    var settings = UnityEngine.Object.FindFirstObjectByType<MjGlobalSettings>();
    if (settings == null) {
      settings = new GameObject("MjGlobalSettings").AddComponent<MjGlobalSettings>();
    }
    settings.UseRawGameObjectNames = true;

    var scene = new GameObject("MjScene").AddComponent<MjScene>();
    var regenerated = scene.CreateScene();
    regenerated.Save(Path.Combine(Application.dataPath, "../unity_regenerated.xml"));

    int ngeom = (int)scene.Model->ngeom;
    int nq = (int)scene.Model->nq;
    int nv = (int)scene.Model->nv;
    int nu = (int)scene.Model->nu;
    int neq = (int)scene.Model->neq;
    double timestep = scene.Model->opt.timestep;
    Debug.Log($"BENCH model ngeom={ngeom} nq={nq} nv={nv} nu={nu} neq={neq} timestep={timestep:R}");
    Debug.Log($"BENCH solver={scene.Model->opt.solver} iterations={scene.Model->opt.iterations} integrator={scene.Model->opt.integrator}");

    // ---- numerical equivalence: identical reset state, zero control ----
    MujocoLib.mj_resetData(scene.Model, scene.Data);
    for (int i = 0; i < nu; i++) scene.Data->ctrl[i] = 0.0;
    const int cmpSteps = 5000;
    for (int i = 0; i < cmpSteps; i++) MujocoLib.mj_step(scene.Model, scene.Data);

    double qsum = 0, qabs = 0;
    for (int i = 0; i < nq; i++) { qsum += scene.Data->qpos[i]; qabs += Math.Abs(scene.Data->qpos[i]); }
    Debug.Log($"BENCH equiv steps={cmpSteps} time={scene.Data->time:R} qpos_sum={qsum:R} qpos_abssum={qabs:R}");

    var sb = new StringBuilder();
    for (int i = 0; i < nq; i++) {
      sb.Append(scene.Data->qpos[i].ToString("R", CultureInfo.InvariantCulture));
      if (i < nq - 1) sb.Append(',');
    }
    File.WriteAllText(Path.Combine(Application.dataPath, "../unity_qpos.csv"), sb.ToString());

    // ---- step-cost benchmark at the real timestep ----
    MujocoLib.mj_resetData(scene.Model, scene.Data);
    for (int i = 0; i < 500; i++) MujocoLib.mj_step(scene.Model, scene.Data);

    const int steps = 20000;
    var sw = Stopwatch.StartNew();
    for (int i = 0; i < steps; i++) MujocoLib.mj_step(scene.Model, scene.Data);
    sw.Stop();

    double usPerStep = sw.Elapsed.TotalMilliseconds * 1000.0 / steps;
    Debug.Log($"BENCH perf us_per_step={usPerStep:F2} max_rate_hz={1e6 / usPerStep:F0} realtime_at_1ms={1000.0 / (1e6 / usPerStep) * 1000:F1}x");
    Debug.Log($"BENCH budget_at_1kHz_pct={usPerStep / 1000.0 * 100:F1} budget_per_90Hz_frame_pct={usPerStep * 11.1 / 11111.0 * 100:F1}");
    BuildSpaceRobotMjScene.Finish();
  }
}
