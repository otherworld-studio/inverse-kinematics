using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;


public class Inverse_Kinematics : MonoBehaviour
{
    [SerializeField]
    private List<Transform> joints;

    private Vector3 target;

    [SerializeField]
    private GameObject targetObj;//TODO: remove eventually

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        //TODO: update target
        target = targetObj.transform.position;

        List<Vector3> jointPos = new List<Vector3>();
        foreach (Transform t in joints)
        {
            jointPos.Add(t.position);
        }

        fabrik_solve(target, ref jointPos);

        for (int i = 0; i < joints.Count; ++i)
        {
            joints[i].position = jointPos[i];
        }

        /*
        List<Quaternion> segmentRot = new List<Quaternion>();
        foreach (Transform t in ???)
        {
            segmentRot.Add(t.rotation);
        }

        align_segments(ref segmentRot, jointPos);
        */
    }

    private void fabrik_solve(Vector3 target, ref List<Vector3> jointPos) {

        List<float> lengths = new List<float>();
        for (int i = 0; i < jointPos.Count - 1; ++i)
        {
            lengths.Add(Vector3.Distance(jointPos[i], jointPos[i + 1]));
        }

        float tolerance = 0.1f;
        if (Math.Abs(Vector3.Distance(jointPos[0], target)) >= lengths.Sum())
        {
            for (int i = 0; i < lengths.Count; ++i)
            {
                float lambda = lengths[i] / Vector3.Distance(jointPos[i], target);
                jointPos[i + 1] = (1.0f - lambda) * jointPos[i] + lambda * target;
            }
        } else
        {
            float dif = float.PositiveInfinity;
            while (dif > tolerance)
            {
                jointPos[lengths.Count] = target;

                for (int i = lengths.Count - 1; i >= 0; --i) {
                    float lambda = lengths[i] / Vector3.Distance(jointPos[i], jointPos[i + 1]);
                    jointPos[i] = (1.0f - lambda) * jointPos[i + 1] + lambda * jointPos[i];
                }
                
                for (int i = 0; i < lengths.Count; ++i)
                {
                    float lambda = lengths[i] / Vector3.Distance(jointPos[i], jointPos[i + 1]);
                    jointPos[i + 1] = (1.0f - lambda) * jointPos[i] + lambda * jointPos[i + 1];
                }

                dif = Math.Abs(Vector3.Distance(jointPos[lengths.Count], target));
            }
        }
    }

    private void align_segments(ref List<Quaternion> segmentRot, List<Vector3> jointPos)
    {
        for (int i = 0; i < segmentRot.Count; ++i)
        {
            segmentRot[i] = Quaternion.FromToRotation(Vector3.up, jointPos[i] - jointPos[i + 1]);
        }
    }
}
