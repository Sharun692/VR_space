using UnityEditor;
using UnityEditor.SceneManagement;

// A freshly opened project starts on an empty untitled scene. Open the service
// scene instead, once per editor session, unless a saved scene is already open.
[InitializeOnLoad]
public static class OpenServiceSceneOnLoad {
  private const string ScenePath = "Assets/SpaceRobot/Scenes/SpaceRobot_Service_Mj.unity";
  private const string SessionKey = "SpaceRobot.OpenServiceSceneOnLoad";

  static OpenServiceSceneOnLoad() {
    if (UnityEngine.Application.isBatchMode || SessionState.GetBool(SessionKey, false)) return;
    EditorApplication.delayCall += () => {
      SessionState.SetBool(SessionKey, true);
      if (EditorApplication.isPlayingOrWillChangePlaymode) return;
      if (!string.IsNullOrEmpty(EditorSceneManager.GetActiveScene().path)) return;
      if (AssetDatabase.LoadAssetAtPath<SceneAsset>(ScenePath) == null) return;
      EditorSceneManager.OpenScene(ScenePath);
    };
  }
}
