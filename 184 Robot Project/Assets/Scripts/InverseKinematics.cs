using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public enum JointType
{
    free,
    hinge,
    end
}

public enum Axis
{
    x,
    y,
    z
}

[Serializable]
public struct Joint
{
    public Joint(Joint other, bool reverse)
    {
        type = other.type;
        transform = other.transform;
        axis = other.axis;
        if (reverse)
        {
            phiMin = -other.phiMax;
            phiMax = -other.phiMin;
            thetaMin = -other.thetaMax;
            thetaMax = -other.thetaMin;
        } else
        {
            phiMin = other.phiMin;
            phiMax = other.phiMax;
            thetaMin = other.thetaMin;
            thetaMax = other.thetaMax;
        }
    }

    public JointType type;

    public Transform transform;

    //Constraints for orientation in degrees, from -180 to 180
    public float phiMin, phiMax;

    //Hinge joints only
    public Axis axis;
    public float thetaMin, thetaMax;//-180 to 180
}

public class InverseKinematics : MonoBehaviour
{
    [SerializeField]
    private Transform origin;//Only for the purpose of constraining the rotation of the first joint
    
    [SerializeField]
    private List<Joint> joints;

    //[SerializeField]
    //private Transform pointer;

    [SerializeField]
    private Transform target;

    private List<float> lengths;

    private List<Vector3> local_alignments;

    private Vector3 origin_alignment;

    void Awake()
    {
        lengths = new List<float>();
        local_alignments = new List<Vector3>();

        origin_alignment = (joints[0].transform.position - origin.position).normalized;
        Vector3 dir;
        for (int i = 0; i < joints.Count - 1; ++i)
        {
            dir = joints[i + 1].transform.position - joints[i].transform.position;
            float norm = dir.magnitude;
            lengths.Add(norm);
            local_alignments.Add(joints[i].transform.InverseTransformDirection(dir / norm));
        }
        //dir = pointer.position - joints[joints.Count - 1].transform.position;
        //local_alignments.Add(joints[joints.Count - 1].transform.InverseTransformDirection(dir.normalized));
        local_alignments.Add(Vector3.left);
    }

    void Update()
    {
        fabrik_solve();
    }

    private void fabrik_solve() {
        List<Vector3> joint_positions = new List<Vector3>();
        List<Quaternion> joint_rotations = new List<Quaternion>();
        foreach (Joint j in joints)
        {
            joint_positions.Add(j.transform.position);
            joint_rotations.Add(j.transform.rotation);
        }

        float tolerance = 0.1f * lengths.Sum();
        int num_loops = 0;
        float dif = float.PositiveInfinity;
        while (dif > tolerance)
        {
            //First pass: end to base
            joint_positions[lengths.Count] = target.position;
            joint_rotations[lengths.Count] = target.rotation;
            Vector3 dir;
            Joint j;
            for (int i = lengths.Count - 1; i > 0; --i)
            {
                j = new Joint(joints[i + 1], true);
                dir = get_direction(joint_positions[i], joint_positions[i + 1], joint_rotations[i + 1], j, local_alignments[i]);
                joint_positions[i] = joint_positions[i + 1] - (lengths[i] / dir.magnitude) * dir;
                Vector3 align0 = local_alignments[i];
                Vector3 align1 = local_alignments[i + 1];
                joint_rotations[i] = constrain_spin(reorient(joint_rotations[i], dir, align0), reorient(joint_rotations[i + 1], dir, align1), dir, j);
            }

            //Second pass: base to end
            
            //Buggy code from before:
            /*
            dir = origin.rotation * Vector3.right;
            j = joints[0];
            Quaternion q = constrain_spin(reorient(joint_rotations[0], dir), origin.rotation, dir, j);
            dir = get_direction(joint_positions[0], joint_positions[1], q, j);
            joint_rotations[0] = reorient(q, dir);
            */
            
            dir = joint_positions[1] - joint_positions[0];
            joint_rotations[0] = constrain_spin(joint_rotations[0], reorient(origin.rotation, dir, origin_alignment), dir, joints[0]);
            Quaternion q;
            for (int i = 1; i < lengths.Count; ++i)
            {
                joint_positions[i] = joint_positions[i - 1] + (lengths[i - 1] / dir.magnitude) * dir;
                j = joints[i];
                q = constrain_spin(reorient(joint_rotations[i], dir, local_alignments[i]), joint_rotations[i - 1], dir, j);
                dir = get_direction(joint_positions[i], joint_positions[i + 1], q, j, local_alignments[i]);
                joint_rotations[i] = reorient(q, dir, local_alignments[i]);
            }

            //End effector
            int n = lengths.Count;
            joint_positions[n] = joint_positions[n - 1] + (lengths[n - 1] / dir.magnitude) * dir;
            j = joints[n];
            q = constrain_spin(reorient(joint_rotations[n], dir, local_alignments[n]), joint_rotations[n - 1], dir, j);
            dir = get_direction(joint_positions[n], target.position, q, j, local_alignments[n]);
            joint_rotations[n] = reorient(q, dir, local_alignments[n]);

            //TODO: quick fix to make hand rotate with target
            //joint_rotations[n] = target.rotation;

            dif = Math.Abs(Vector3.Distance(joint_positions[n], target.position));
            ++num_loops;
            if (num_loops > 100)
            {
                Debug.Log("IK took too long to converge!");
                break;
            }
        }

        for (int i = 0; i < joints.Count; ++i)
        {
            joints[i].transform.rotation = joint_rotations[i];
        }
    }

    //Reorients an existing global orientation to point in a specific direction
    private Quaternion reorient(Quaternion rot, Vector3 dir, Vector3 align)
    {
        return Quaternion.FromToRotation(rot * align, dir) * rot;
    }

    private Quaternion constrain_spin(Quaternion child, Quaternion parent, Vector3 axis, Joint j)
    {
        float angle;
        Vector3 angle_axis;
        (child * Quaternion.Inverse(parent)).ToAngleAxis(out angle, out angle_axis);
        if (Vector3.Dot(axis, angle_axis) < 0f) angle = -angle;
        return Quaternion.AngleAxis(Mathf.Clamp(angle, j.phiMin, j.phiMax), axis) * parent;
    }

    //Constrains the direction vector between from and to, according to the angle constraints of the joint at from
    private Vector3 get_direction(Vector3 from, Vector3 to, Quaternion rot, Joint j, Vector3 align)
    {
        Vector3 dir = to - from;
        if (j.type == JointType.hinge)
        {
            Vector3 n = rot * ((j.axis == Axis.x) ? Vector3.right :
                               (j.axis == Axis.y) ? Vector3.up :
                                                    Vector3.forward);
            //Planar projection
            dir -= Vector3.Dot(dir, n) * n;
            Vector3 straight = rot * align;
            float angle = Mathf.Clamp(Vector3.SignedAngle(straight, dir, n), j.thetaMin, j.thetaMax);
            return Quaternion.AngleAxis(angle, n) * straight;
        } else if (j.type == JointType.end)
        {
            return target.rotation * align;
        }
        return dir;
    }
}