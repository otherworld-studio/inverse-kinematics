using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[Serializable]
public class Joint
{
    public Transform transform;

    //Constraints for rotation about each local axis in degrees
    public float xMin, xMax;
    public float yMin, yMax;
    public float zMin, zMax;
}

public class InverseKinematics : MonoBehaviour
{
    [SerializeField]
    private List<Joint> joints;

    private Transform target;

    [SerializeField]
    private GameObject targetObj;//TODO: eventually replace this with the ground/whatever object we want the robot to be touching
    
    void Update()
    {
        target = targetObj.transform;

        //Use inverse kinematics to find the new joint positions
        fabrik_solve(target, ref joints);
    }

    private void fabrik_solve(Transform target, ref List<Joint> joints) {
        List<Vector3> jointPos = new List<Vector3>();
        List<Quaternion> jointRot = new List<Quaternion>();
        foreach (Joint j in joints)
        {
            jointPos.Add(j.transform.position);
            jointRot.Add(j.transform.rotation);
        }

        List<float> lengths = new List<float>();
        for (int i = 0; i < jointPos.Count - 1; ++i)
        {
            lengths.Add(Vector3.Distance(jointPos[i], jointPos[i + 1]));
        }

        float tolerance = 0.1f * lengths.Sum();
        if (Math.Abs(Vector3.Distance(jointPos[0], target.position)) >= lengths.Sum())
        {
            float lambda = lengths[0] / Vector3.Distance(jointPos[0], target.position);
            jointPos[1] = (1.0f - lambda) * jointPos[0] + lambda * target.position;
            jointRot[0] = align_segment(jointRot[0], jointPos[1] - jointPos[0]);

            //For now at least, the first joint is constrained relative to the ORIGIN
            jointRot[0] = constrain_rotation(jointRot[0], joints[0]);
            jointPos[1] = jointPos[0] + lengths[0] * (jointRot[0] * Vector3.right);

            for (int i = 1; i < lengths.Count; ++i)
            {
                lambda = lengths[i] / Vector3.Distance(jointPos[i], target.position);
                jointPos[i + 1] = (1.0f - lambda) * jointPos[i] + lambda * target.position;
                jointRot[i] = align_segment(jointRot[i], jointPos[i + 1] - jointPos[i]);

                //Apply constraints relative to parent's orientation
                Quaternion localRotation = Quaternion.Inverse(jointRot[i - 1]) * jointRot[i];
                jointRot[i] = jointRot[i - 1] * constrain_rotation(localRotation, joints[i]);
                jointPos[i + 1] = jointPos[i] + lengths[i] * (jointRot[i] * Vector3.right);
            }
        } else
        {
            int num_loops = 0;
            float dif = float.PositiveInfinity;
            while (dif > tolerance)
            {
                //First pass: end to base
                jointPos[lengths.Count] = target.position;

                float lambda;
                for (int i = lengths.Count - 1; i >= 0; --i) {
                    lambda = lengths[i] / Vector3.Distance(jointPos[i], jointPos[i + 1]);
                    jointPos[i] = (1.0f - lambda) * jointPos[i + 1] + lambda * jointPos[i];

                    //TODO: implement constraints during the first pass
                }

                //Second pass: base to end
                lambda = lengths[0] / Vector3.Distance(jointPos[0], jointPos[1]);
                jointPos[1] = (1.0f - lambda) * jointPos[0] + lambda * jointPos[1];
                jointRot[0] = align_segment(jointRot[0], jointPos[1] - jointPos[0]);

                //For now at least, the first joint is constrained relative to the ORIGIN
                jointRot[0] = constrain_rotation(jointRot[0], joints[0]);
                jointPos[1] = jointPos[0] + lengths[0] * (jointRot[0] * Vector3.right);

                for (int i = 1; i < lengths.Count; ++i)
                {
                    lambda = lengths[i] / Vector3.Distance(jointPos[i], jointPos[i + 1]);
                    jointPos[i + 1] = (1.0f - lambda) * jointPos[i] + lambda * jointPos[i + 1];
                    jointRot[i] = align_segment(jointRot[i], jointPos[i + 1] - jointPos[i]);

                    //Apply constraints relative to parent's orientation
                    Quaternion localRotation = Quaternion.Inverse(jointRot[i - 1]) * jointRot[i];
                    jointRot[i] = jointRot[i - 1] * constrain_rotation(localRotation, joints[i]);
                    jointPos[i + 1] = jointPos[i] + lengths[i] * (jointRot[i] * Vector3.right);
                }

                dif = Math.Abs(Vector3.Distance(jointPos[lengths.Count], target.position));
                ++num_loops;
                if (num_loops > 10)
                {
                    Debug.Log("IK took too long to converge!");
                    break;
                }
            }
        }

        //Lazily match end effector's orientation with target. We will work on constraining this later (maybe)
        jointRot[lengths.Count] = target.rotation;

        for (int i = 0; i < joints.Count; ++i)
        {
            joints[i].transform.rotation = jointRot[i];
        }
    }

    //Aligns "from" to point in direction "to". Use this over FromToRotation to avoid random flipping
    private Quaternion align_segment(Quaternion from, Vector3 to)
    {
        Vector3 forward = Vector3.Cross(to, from * Vector3.up);
        Vector3 up = Vector3.Cross(forward, to);
        return Quaternion.LookRotation(forward, up);
    }

    //Constrains the local orientation of a joint
    private Quaternion constrain_rotation(Quaternion rotation, Joint joint)
    {
        //TODO: this doesn't really work yet, possibly because rotations aren't necessary applied in the order x-y-z or z-y-x
        //We have to figure this out if we want realistic poses
        Vector3 rotation_euler = rotation.eulerAngles;
        //rotation_euler.x = Mathf.Clamp(rotation_euler.x, joint.xMin, joint.xMax);
        //rotation_euler.y = Mathf.Clamp(rotation_euler.y, joint.yMin, joint.yMax);
        //rotation_euler.z = Mathf.Clamp(rotation_euler.z, joint.zMin, joint.zMax);
        return Quaternion.Euler(rotation_euler);
    }
}