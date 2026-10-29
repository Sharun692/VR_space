using System;
using System.Linq;
using System.Xml;
using UnityEngine;

namespace Mujoco {

  public class MjConnect : MjBaseConstraint {
    public MjBaseBody Body1;
    public MjBaseBody Body2;
    public MjSite Site1;
    public MjSite Site2;
    public Transform Anchor;
    public bool Active = true;

    protected override string _constraintName => "connect";

    private bool UsesSites => Site1 != null && Site2 != null;

    protected override void FromMjcf(XmlElement mjcf) {
      Active = mjcf.GetBoolAttribute("active", defaultValue: true);

      if (mjcf.HasAttribute("site1") || mjcf.HasAttribute("site2")) {
        Site1 = mjcf.GetObjectReferenceAttribute<MjSite>("site1");
        Site2 = mjcf.GetObjectReferenceAttribute<MjSite>("site2");
        return;
      }

      Body1 = mjcf.GetObjectReferenceAttribute<MjBaseBody>("body1");
      Body2 = mjcf.GetObjectReferenceAttribute<MjBaseBody>("body2");
      var anchorPos = MjEngineTool.UnityVector3(
          mjcf.GetVector3Attribute("anchor", defaultValue: Vector3.zero));
      Anchor = new GameObject("connect_anchor").transform;
      Anchor.parent = Body1.transform;
      Anchor.localPosition = anchorPos;
    }

    protected override void ToMjcf(XmlElement mjcf) {
      if (!Active) mjcf.SetAttribute("active", "false");

      if (UsesSites) {
        mjcf.SetAttribute("site1", Site1.MujocoName);
        mjcf.SetAttribute("site2", Site2.MujocoName);
        return;
      }

      if (Body1 == null || Body2 == null) {
        throw new NullReferenceException($"Both bodies in connect {name} are required.");
      }
      if (Anchor == null) {
        throw new NullReferenceException($"Anchor in connect {name} is required.");
      }
      mjcf.SetAttribute("body1", Body1.MujocoName);
      mjcf.SetAttribute("body2", Body2.MujocoName);
      mjcf.SetAttribute("anchor",
                        MjEngineTool.Vector3ToMjcf(MjEngineTool.MjVector3(Anchor.localPosition)));
    }

    // Engage or release the constraint while the simulation is running.
    public unsafe void SetActive(bool active) {
      Active = active;
      if (!MjScene.InstanceExists || MujocoId < 0) return;
      MjScene.Instance.Data->eq_active[MujocoId] = (byte)(active ? 1 : 0);
    }

    public unsafe bool IsActive() {
      if (!MjScene.InstanceExists || MujocoId < 0) return Active;
      return MjScene.Instance.Data->eq_active[MujocoId] != 0;
    }

    public void OnValidate() {
      if (Body1 != null && Body1 == Body2) {
        Debug.LogError("Body1 and Body2 can't be the same - resetting Body2.", this);
        Body2 = null;
      }
      if (Site1 != null && Site1 == Site2) {
        Debug.LogError("Site1 and Site2 can't be the same - resetting Site2.", this);
        Site2 = null;
      }
    }
  }
}
