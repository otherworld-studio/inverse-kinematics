using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

//This system does NOT assume that all joint transforms are similarly aligned to the model (in other words, we don't assume that, for example, the local x-axis always points to the next joint).
//However, if this assumption can be made, then every joint's align vector is the same, and we can optimize.

//TODO: fix bug where an arm in which all joints are colinear tends to stay that way when it should be bending instead

public enum JointType
{
    free,
    hinge
}

public enum Axis
{
    x,
    y,
    z
}

[Serializable]
public class Joint
{
    public JointType type;

    public Transform transform;

    //Constraints for orientation in degrees, from -180 to 180
    public float phiMin, phiMax;

    //Hinge joints only
    public Axis axis;
    public float thetaMin, thetaMax;//-180 to 180

    [NonSerialized]
    public Vector3 position;
    [NonSerialized]
    public Quaternion rotation;
    [NonSerialized]
    public Vector3 align;
    [NonSerialized]
    public float length;
}

public class InverseKinematics : MonoBehaviour
{
    [SerializeField]
    private List<Joint> joints;

    [SerializeField]
    private Transform pointer;//Some point to tell us how the end effector is aligned

    [SerializeField]
    private Transform target;

    private float tolerance;

    void Awake()
    {
        Vector3 dir;
        float total_length = 0f;
        for (int i = 0; i < joints.Count - 1; ++i)
        {
            dir = joints[i + 1].transform.position - joints[i].transform.position;
            float length = dir.magnitude;
            joints[i].length = length;
            joints[i].align = joints[i].transform.InverseTransformDirection(dir / length);
            total_length += length;
        }
        dir = pointer.position - joints[joints.Count - 1].transform.position;
        joints[joints.Count - 1].align = joints[joints.Count - 1].transform.InverseTransformDirection(dir.normalized);

        tolerance = 0.1f * total_length;
    }

    void Update()
    {
        fabrik_solve();
    }

    private void fabrik_solve() {
        //Initialize variables
        foreach (Joint j in joints)
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

            Joint j, j_prev;
            Vector3 dir;
            for (int i = joints.Count - 2; i > 0; --i)
            {
                j = joints[i];
                j_prev = joints[i + 1];
                dir = get_direction(j_prev, j.position, true);
                j.position = j_prev.position + (j.length / dir.magnitude) * dir;
                joints[i].rotation = constrain_spin(j, j_prev, -dir);
            }

            //Second pass: base to end
            dir = joints[1].position - joints[0].position;
            joints[0].rotation = reorient(joints[0], dir);
            for (int i = 1; i < joints.Count - 1; ++i)
            {
                j = joints[i];
                j_prev = joints[i - 1];
                j.position = j_prev.position + (j_prev.length / dir.magnitude) * dir;
                j.rotation = constrain_spin(j, j_prev, dir);
                dir = get_direction(j, joints[i + 1].position);
                j.rotation = reorient(j, dir);
            }

            //End effector
            int n = joints.Count - 1;
            j = joints[n];
            j_prev = joints[n - 1];
            j.position = j_prev.position + (j_prev.length / dir.magnitude) * dir;
            j.rotation = constrain_spin(j, j_prev, dir);
            dir = get_direction(j, j.position + target.rotation * j.align);
            j.rotation = reorient(j, dir);

            dif = Math.Abs(Vector3.Distance(j.position, target.position));
            ++num_loops;
            if (num_loops > 100)
            {
                Debug.Log("IK took too long to converge!");
                break;
            }
        }

        //Update transforms
        for (int i = 0; i < joints.Count; ++i)
        {
            joints[i].transform.rotation = joints[i].rotation;
        }
    }

    //Reorients a joint to point in a specific direction
    private Quaternion reorient(Joint j, Vector3 dir)
    {
        return Quaternion.FromToRotation(j.rotation * j.align, dir) * j.rotation;
    }

    //TODO: for now, constrain_spin does all the thinking about different alignments
    //Hence this does NOT assume that child or parent is parallel to the given axis, even though they are sometimes
    //But eventually we might optimize...
    private Quaternion constrain_spin(Joint child, Joint parent, Vector3 axis)
    {
        Quaternion q = parent.rotation * Quaternion.FromToRotation(child.align, parent.align);
        q = Quaternion.FromToRotation(q * child.align, axis) * q;
        float angle;
        Vector3 angle_axis;
        (reorient(child, axis) * Quaternion.Inverse(q)).ToAngleAxis(out angle, out angle_axis);
        if (Vector3.Dot(axis, angle_axis) < 0f) angle = -angle;
        return Quaternion.AngleAxis(Mathf.Clamp(angle, child.phiMin, child.phiMax), axis) * q;
    }

    //Constrains the direction vector between from and to, according to from's constrants
    private Vector3 get_direction(Joint from, Vector3 to, bool reverse=false)
    {
        Vector3 dir = to - from.position;
        if (from.type == JointType.hinge)
        {
            Vector3 n = from.rotation * ((from.axis == Axis.x) ? Vector3.right :
                                         (from.axis == Axis.y) ? Vector3.up :
                                                                 Vector3.forward);
            dir -= Vector3.Dot(dir, n) * n;// Planar projection
            Vector3 straight = from.rotation * from.align;
            if (reverse) straight = -straight;
            float min = (reverse) ? -from.thetaMax : from.thetaMin;
            float max = (reverse) ? -from.thetaMin : from.thetaMax;
            float angle = Mathf.Clamp(Vector3.SignedAngle(straight, dir, n), min, max);
            return Quaternion.AngleAxis(angle, n) * straight;
        }
        return dir;
    }
}