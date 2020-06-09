using System;
using System.Collections.Generic;
using UnityEngine;

//How to use, when the joints are misaligned:
//1. For each joint, create an IK transform at its exact position
//2. Rotate this IK transform so that its axis corresponding to alignment_axis points toward the next joint (additionally one can use the autoalign toggle to get this as accurate as possible)
//3. Assign the IKJoint script to this rotated object
//4. Take both the actual joint, and its children (which must include the NEXT joint for this to work properly), and make these direct children of the IKJoint object

public enum Axis
{
    x,
    y,
    z
}

public class InverseKinematics : MonoBehaviour
{
    public Axis alignment_axis;//TODO: make this, tangent and local_frame per-joint?

    [SerializeField]
    private bool autoalign;//Automatically align joint transforms

    [SerializeField]
    private Transform origin;//For the purpose of constraining the base joint (an unconstrained base twists gradually). Should be a parent of joints[0].

    [SerializeField]
    private List<IKJoint> joints;

    [SerializeField]
    private Transform target;

    [SerializeField]
    private float fractional_tolerance = 0.01f;
    [SerializeField]
    private int max_loops = 100;

    [SerializeField]
    private string debug_string;

    [NonSerialized]
    public Vector3 tangent;
    [NonSerialized]
    public Quaternion local_frame;

    private float tolerance;

    void Awake()
    {
        switch(alignment_axis)
        {
            case Axis.x:
                tangent = Vector3.right;
                local_frame = Quaternion.LookRotation(Vector3.right, Vector3.forward);
                break;
            case Axis.y:
                tangent = Vector3.up;
                local_frame = Quaternion.LookRotation(Vector3.up, Vector3.right);
                break;
            default:
                tangent = Vector3.forward;
                break;
        }

        float total_length = 0f;
        for (int i = 0; i < joints.Count - 1; ++i)
        {
            Vector3 dir = joints[i + 1].transform.position - joints[i].transform.position;
            if (autoalign)
            {
                float angle;
                Vector3 axis;
                Quaternion.FromToRotation(dir, joints[i].transform.rotation * tangent).ToAngleAxis(out angle, out axis);
                Debug.Assert(!float.IsInfinity(axis.x));
                foreach (Transform child in joints[i].transform)
                {
                    child.RotateAround(joints[i].transform.position, axis, angle);
                }
                dir = joints[i + 1].transform.position - joints[i].transform.position;
            }

            float debug = Vector3.Dot(joints[i].transform.rotation * tangent, dir.normalized);
            Debug.Assert(debug > 0.99f, debug_string + " " + i + ": " + debug);

            float length = Vector3.Distance(joints[i].transform.position, joints[i + 1].transform.position);
            joints[i].length = length;
            joints[i].IK = this;
            total_length += length;
        }
        joints[joints.Count - 1].length = 0f;
        joints[joints.Count - 1].IK = this;

        tolerance = fractional_tolerance * total_length;
    }

    void Update()
    {
        fabrik_solve();
    }

    private void fabrik_solve() {
        //Initialize variables
        Quaternion origin_rotation = origin.rotation;
        foreach (IKJoint j in joints)
        {
            j.position = j.transform.position;
            j.rotation = j.transform.rotation;
        }

        int num_loops = 0;
        float dif = float.PositiveInfinity;
        while (dif > tolerance)
        {
            //First pass: end to base
            joints[joints.Count - 1].position = target.position;
            joints[joints.Count - 1].rotation = target.rotation;

            IKJoint j, j_prev;
            Vector3 dir;
            for (int i = joints.Count - 2; i > 0; --i)
            {
                j = joints[i];
                j_prev = joints[i + 1];
                dir = j_prev.get_direction(j.position, true);
                j.position = j_prev.position + j.length * dir;
                j.constrain_spin_backward(j_prev, -dir);
            }

            //Second pass: base to end
            j = joints[0];
            j_prev = joints[1];
            j.constrain_spin_forward(origin_rotation);
            dir = j.constrain_direction((j_prev.position - j.position).normalized - j_prev.get_direction(j.position, true));//Constrain the vector bisector
            j.reorient(dir);
            for (int i = 1; i < joints.Count - 1; ++i)
            {
                j = joints[i];
                j_prev = joints[i - 1];
                j.position = j_prev.position + j_prev.length * dir;
                j.constrain_spin_forward(j_prev.rotation);
                dir = j.get_direction(joints[i + 1].position);
                j.reorient(dir);
            }

            //End effector
            int n = joints.Count - 1;
            j = joints[n];
            j_prev = joints[n - 1];
            j.position = j_prev.position + j_prev.length * dir;
            j.constrain_spin_forward(j_prev.rotation);
            dir = j.constrain_direction(target.rotation * tangent);
            j.reorient(dir);

            dif = Math.Abs(Vector3.Distance(j.position, target.position));
            if (++num_loops > max_loops)
            {
                Debug.Log("IK took too long to converge!");
                break;
            }
        }

        //Due to some sign-related error in the way Unity handles localRotations, we must handle the first step globally
        //Ideally, we would use:
        //joints[0].transform.localRotation = Quaternion.Inverse(origin_rotation) * joints[0].rotation;
        joints[0].transform.rotation = joints[0].rotation;
        for (int i = 1; i < joints.Count; ++i)
        {
            //Assumes each joint is a direct child of the previous joint
            joints[i].transform.localRotation = Quaternion.Inverse(joints[i - 1].rotation) * joints[i].rotation;
        }
    }
}