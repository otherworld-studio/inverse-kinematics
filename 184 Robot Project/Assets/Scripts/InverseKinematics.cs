using System;
using System.Collections.Generic;
using UnityEngine;

//This system does NOT assume that all joint transforms are similarly aligned to the model (in other words, we don't assume that, for example, the local x-axis always points to the next joint).
//However, if this assumption can be made, then every joint's align vector is the same, and we can optimize.

//TODO - potential next steps:
//represent alignment as a rotation w.r.t. Vector3.forward
//optimize constrain_spin by moving reorient calls outside
//if the above step allows, move reorient into the end of get_direction/constrain_direction

public class InverseKinematics : MonoBehaviour
{
    [SerializeField]
    private Transform origin;//For the purpose of constraining the base joint
    //If we don't do this, the unconstrained base twists gradually

    [SerializeField]
    private List<Joint> joints;

    [SerializeField]
    private Transform pointer;//Some point to tell us how the end effector is aligned

    [SerializeField]
    private Transform target;

    private float tolerance;

    private Joint origin_joint;

    void Awake()
    {
        origin_joint = new Joint();
        origin_joint.transform = origin;
        Vector3 dir = joints[0].transform.position - origin_joint.transform.position;
        origin_joint.align = origin_joint.transform.InverseTransformDirection(dir.normalized);

        float total_length = 0f;
        for (int i = 0; i < joints.Count - 1; ++i)
        {
            Joint j = joints[i];
            dir = joints[i + 1].transform.position - j.transform.position;
            float length = dir.magnitude;
            j.length = length;
            j.align = j.transform.InverseTransformDirection(dir / length);
            total_length += length;
        }
        dir = pointer.position - joints[joints.Count - 1].transform.position;
        joints[joints.Count - 1].align = joints[joints.Count - 1].transform.InverseTransformDirection(dir.normalized);
        joints[joints.Count - 1].length = 0f;

        tolerance = 0.1f * total_length;
    }

    void Update()
    {
        fabrik_solve();
    }

    private void fabrik_solve() {
        //Initialize variables
        origin_joint.position = origin_joint.transform.position;
        origin_joint.rotation = origin_joint.transform.rotation;
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
                dir = j_prev.get_direction(j.position, true);
                j.position = j_prev.position + j.length * dir;
                j.constrain_spin(j_prev, -dir, true);
            }

            //Second pass: base to end
            j = joints[0];
            j.constrain_spin(origin_joint, j.position - origin_joint.position);
            dir = (j.get_direction(joints[1].position) - joints[1].get_direction(j.position, true)).normalized;//Vector bisector
            j.reorient(dir);
            for (int i = 1; i < joints.Count - 1; ++i)
            {
                j = joints[i];
                j_prev = joints[i - 1];
                j.position = j_prev.position + j_prev.length * dir;
                j.constrain_spin(j_prev, dir);
                dir = j.get_direction(joints[i + 1].position);
                j.reorient(dir);
            }

            //End effector
            int n = joints.Count - 1;
            j = joints[n];
            j_prev = joints[n - 1];
            j.position = j_prev.position + j_prev.length * dir;
            j.constrain_spin(j_prev, dir);
            dir = j.constrain_direction(target.rotation * j.align);
            j.reorient(dir);

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

    [Serializable]
    private class Joint
    {
        public Transform transform;

        [SerializeField]
        private JointType type;

        //Constraints for orientation in degrees, from -180 to 180
        [SerializeField]
        private float phiMin, phiMax;

        [SerializeField]
        private Axis axis;
        [SerializeField]
        private float thetaMin, thetaMax;//-180 to 180 if type == hinge; if type == ball_and_socket, min within (-90f, 0) and max within (0, 90f) (for now)
        [SerializeField]
        private float psiMin, psiMax;//(-90f, 0), (0, 90f), ball_and_sockets only

        [NonSerialized]
        public Vector3 position;
        [NonSerialized]
        public Quaternion rotation;
        [NonSerialized]
        public Vector3 align;
        [NonSerialized]
        public float length;

        //Reorients to a specific direction
        public void reorient(Vector3 dir)
        {
            rotation = Quaternion.FromToRotation(rotation * align, dir) * rotation;
        }

        //TODO: for now, constrain_spin does all the thinking about different alignments
        //Hence this does NOT assume that either joint is parallel to the given axis, even though they are sometimes
        //But eventually we might optimize...
        public void constrain_spin(Joint parent, Vector3 axis, bool reverse = false)
        {
            Quaternion q = parent.rotation * Quaternion.FromToRotation(align, parent.align);
            q = Quaternion.FromToRotation(q * align, axis) * q;
            reorient(axis);
            float angle;
            Vector3 angle_axis;
            (rotation * Quaternion.Inverse(q)).ToAngleAxis(out angle, out angle_axis);
            if (Vector3.Dot(axis, angle_axis) < 0f) angle = -angle;
            float min = (reverse) ? -parent.phiMax : phiMin;
            float max = (reverse) ? -parent.phiMin : phiMax;
            rotation = Quaternion.AngleAxis(Mathf.Clamp(angle, min, max), axis) * q;
        }

        //Constrains the direction vector between from and to, according to from's constrants. Returns a unit vector.
        public Vector3 get_direction(Vector3 to, bool reverse = false)
        {
            return constrain_direction(to - position, reverse);
        }

        //Constrains a direction vector according to j's constraints. Returns a unit vector. dir does not need to be normalized.
        public Vector3 constrain_direction(Vector3 dir, bool reverse = false)
        {
            if (type == JointType.hinge)
            {
                Vector3 n = rotation * ((axis == Axis.x) ? Vector3.right :
                                        (axis == Axis.y) ? Vector3.up :
                                                           Vector3.forward);
                dir -= Vector3.Dot(dir, n) * n;// Planar projection
                Vector3 straight = rotation * align;
                if (reverse) straight = -straight;
                float min = (reverse) ? -thetaMax : thetaMin;
                float max = (reverse) ? -thetaMin : thetaMax;
                float angle = Mathf.Clamp(Vector3.SignedAngle(straight, dir, n), min, max);
                return Quaternion.AngleAxis(angle, n) * straight;
            }
            /*
            if (type == JointType.ball_and_socket)
            {
                Debug.Assert(thetaMin > -90f && thetaMin < 0f && thetaMax > 0f && thetaMax < 90f && psiMin > -90f && psiMin < 0f && psiMax > 0f && psiMax < 90f);//DEBUG: only acute angles for now
                //Use a free joint if you want the allowed range of motion to be greater than 180 degrees
                Matrix3 o2w;
                get_coord_space(o2w);
                //TODO:psiMin, psiMax
            }
            */
            return dir.normalized;
        }

        /*
        public void get_coord_space(out Matrix3 o2w)
        {
            //TODO: need to represent alignment as a rotation!

            o2w[0] = x;
            o2w[1] = y;
            o2w[2] = z;
        }
        */

        private enum JointType
        {
            free,
            hinge,
            ball_and_socket
        }

        private enum Axis
        {
            x,
            y,
            z
        }
    }
}