using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[Serializable]
public class Joint
{
    public Transform transform;

    //Constraints for orientation in degrees
    public float oMin, oMax;

    //Constraints for rotation in degrees, between 0 and 360
    public float thetaMin, thetaMax;
    public float phiMin, phiMax;
}

public class InverseKinematics : MonoBehaviour
{
    [SerializeField]
    private List<Joint> joints;

    [SerializeField]
    private Transform origin, target;
    
    void Update()
    {
        fabrik_solve(target, in joints);
    }

    private void fabrik_solve(Transform target, in List<Joint> joints) {
        List<Vector3> joint_positions = new List<Vector3>();
        List<Quaternion> joint_rotations = new List<Quaternion>();
        foreach (Joint j in joints)
        {
            joint_positions.Add(j.transform.position);
            joint_rotations.Add(j.transform.rotation);
        }

        List<float> lengths = new List<float>();
        for (int i = 0; i < joint_positions.Count - 1; ++i)
        {
            lengths.Add(Vector3.Distance(joint_positions[i], joint_positions[i + 1]));
        }

        float tolerance = 0.1f * lengths.Sum();
        if (Math.Abs(Vector3.Distance(joint_positions[0], target.position)) >= lengths.Sum())
        {
            {
                Vector3 dir = target.position - joint_positions[0];
                joint_positions[1] = joint_positions[0] + (lengths[0] / dir.magnitude) * dir;
                Joint j = joints[0];
                joint_rotations[0] = orientation(dir, origin.rotation, j.oMin, j.oMax);
            }
            for (int i = 1; i < lengths.Count; ++i)
            {
                Vector3 dir = target.position - joint_positions[i];
                joint_positions[i + 1] = joint_positions[i] + (lengths[i] / dir.magnitude) * dir;
                Joint j = joints[i];
                joint_rotations[i] = orientation(dir, joint_rotations[i - 1], j.oMin, j.oMax);
            }
        } else
        {
            int num_loops = 0;
            float dif = float.PositiveInfinity;
            while (dif > tolerance)
            {
                //First pass: end to base
                joint_positions[lengths.Count] = target.position;
                joint_rotations[lengths.Count] = target.rotation;
                for (int i = lengths.Count - 1; i > 0; --i)
                {
                    Vector3 dir = joint_positions[i + 1] - joint_positions[i];
                    joint_positions[i] = joint_positions[i + 1] - (lengths[i] / dir.magnitude) * dir;
                    Joint j = joints[i + 1];
                    joint_rotations[i] = orientation(dir, joint_rotations[i + 1], 360f - j.oMax, 360f - j.oMin);
                }

                //Second pass: base to end
                {
                    Vector3 dir = joint_positions[1] - joint_positions[0];
                    joint_positions[1] = joint_positions[0] + (lengths[0] / dir.magnitude) * dir;
                    Joint j = joints[0];
                    joint_rotations[0] = orientation(dir, origin.rotation, j.oMin, j.oMax);
                }
                for (int i = 1; i < lengths.Count; ++i)
                {
                    Vector3 dir = joint_positions[i + 1] - joint_positions[i];
                    joint_positions[i + 1] = joint_positions[i] + (lengths[i] / dir.magnitude) * dir;
                    Joint j = joints[i];
                    joint_rotations[i] = orientation(dir, joint_rotations[i - 1], j.oMin, j.oMax);
                }

                dif = Math.Abs(Vector3.Distance(joint_positions[lengths.Count], target.position));
                ++num_loops;
                if (num_loops > 10)
                {
                    Debug.Log("IK took too long to converge!");
                    break;
                }
            }
        }

        for (int i = 0; i < joints.Count; ++i)
        {
            joints[i].transform.rotation = joint_rotations[i];
        }
    }

    private Quaternion orientation(Vector3 child_direction, Quaternion parent_rotation, float oMin, float oMax)
    {
        Quaternion q = Quaternion.FromToRotation(parent_rotation * Vector3.right, child_direction) * parent_rotation;
        float clamped = (oMin < 360f - oMax) ? oMin : oMax;
        return Quaternion.AngleAxis(clamped, child_direction) * q;
    }
}