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
        target = targetObj.transform.position;

        List<Vector3> jointPos = new List<Vector3>();
        foreach (Transform t in joints)
        {
            jointPos.Add(t.position);
        }

        //Use inverse kinematics to find the new joint positions
        fabrik_solve(target, ref jointPos);

        //Now we have to rotate the joints to match these positions

        List<Quaternion> segmentRot = align_segments(jointPos);

        for (int i = 0; i < segmentRot.Count; ++i)
        {
            joints[i].rotation = segmentRot[i];
        }
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

    private List<Quaternion> align_segments(List<Vector3> jointPos)
    {
        List<Quaternion> segmentRot = new List<Quaternion>();
        for (int i = 0; i < jointPos.Count - 1; ++i)
        {
            //Vector3.left works for the right arm, but not sure about the other limbs
            //We might have to generalize this later by aligning based on the previous joint positions
            segmentRot.Add(Quaternion.FromToRotation(Vector3.left, jointPos[i] - jointPos[i + 1]));
        }

        return segmentRot;
    }
}
