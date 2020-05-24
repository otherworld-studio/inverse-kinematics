using System;
using System.Collections.Generic;
using UnityEngine;

//TODO: rewrite everything in terms of local transformations, for ultimate optimization...? (use performance testing)

public class InverseKinematics : MonoBehaviour
{
    [SerializeField]
    private Axis alignment_axis;

    [SerializeField]
    private bool autoalign;//Automatically align joint transforms
    //How to use:
    //1. For each joint, create a "target" transform at its exact position (this is what we do anyway, but now we don't have to worry about trying to point it at the next joint)
    //2. Make actual joint a child of the target

    [SerializeField]
    private Transform origin;//For the purpose of constraining the base joint
    //If we don't do this, the unconstrained base twists gradually

    [SerializeField]
    private List<Joint> joints;

    [SerializeField]
    private Transform target;

    private float tolerance;

    private Vector3 tangent, normal, binormal;

    [SerializeField]
    private string debug_string;

    void Awake()
    {
        switch(alignment_axis)
        {
            case Axis.x:
                tangent = Vector3.right;
                normal = Vector3.up;
                binormal = Vector3.forward;
                break;
            case Axis.y:
                tangent = Vector3.up;
                normal = Vector3.forward;
                binormal = Vector3.right;
                break;
            default:
                tangent = Vector3.forward;
                normal = Vector3.right;
                binormal = Vector3.up;
                break;
        }

        Vector3 dir = joints[0].transform.position - origin.position;
        if (autoalign)
        {
            Quaternion q = Quaternion.FromToRotation(dir, origin.rotation * tangent);
            foreach (Transform child in origin)
            {
                child.rotation = q * child.rotation;
            }
            dir = joints[0].transform.position - origin.position;
        }

        float debug = Vector3.Dot(origin.rotation * tangent, dir.normalized);
        Debug.Assert(debug > 0.99f, debug_string + " origin: " + debug);

        float total_length = 0f;
        for (int i = 0; i < joints.Count - 1; ++i)
        {
            dir = joints[i + 1].transform.position - joints[i].transform.position;
            if (autoalign)
            {
                Quaternion q = Quaternion.FromToRotation(dir, joints[i].transform.rotation * tangent);
                foreach (Transform child in joints[i].transform)
                {
                    child.rotation = q * child.rotation;
                }
                dir = joints[i + 1].transform.position - joints[i].transform.position;
            }

            debug = Vector3.Dot(joints[i].transform.rotation * tangent, dir.normalized);
            Debug.Assert(debug > 0.99f, debug_string + " " + i + ": " + debug);

            float length = Vector3.Distance(joints[i].transform.position, joints[i + 1].transform.position);
            joints[i].length = length;
            joints[i].IK = this;
            total_length += length;
        }
        joints[joints.Count - 1].length = 0f;
        joints[joints.Count - 1].IK = this;

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
                dir = j_prev.get_direction(j.position, true);
                j.position = j_prev.position + j.length * dir;
                j.constrain_spin_backward(j_prev, -dir);
            }

            //Second pass: base to end
            j = joints[0];
            j_prev = joints[1];
            j.constrain_spin_forward(origin.rotation);
            dir = (j.get_direction(j_prev.position) - j_prev.get_direction(j.position, true)).normalized;//Vector bisector
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
            ++num_loops;
            if (num_loops > 100)
            {
                Debug.Log("IK took too long to converge!");
                return;
            }
        }

        //Update transforms
        foreach (Joint j in joints)
        {
            j.transform.rotation = j.rotation;
        }
    }

    [Serializable]
    private class Joint
    {
        public Transform transform;

        [SerializeField]
        private JointType type;
        [SerializeField]
        private float phiMin, phiMax;//Constraints for orientation in degrees, from -180 to 180
        [SerializeField]
        private Axis axis;//Normal vector of hinge plane
        [SerializeField]
        private float thetaMin, thetaMax;//-180 to 180, hinges only
        [SerializeField]
        private float psiMax;//within [0, 90f), ball_and_sockets only

        [NonSerialized]
        public Vector3 position;
        [NonSerialized]
        public Quaternion rotation;
        [NonSerialized]
        public float length;
        [NonSerialized]
        public InverseKinematics IK;

        //Reorients to a specific direction
        public void reorient(Vector3 dir)
        {
            rotation = Quaternion.FromToRotation(rotation * IK.tangent, dir) * rotation;
        }

        public void constrain_spin_backward(Joint parent, Vector3 axis)
        {
            Quaternion q = Quaternion.FromToRotation(parent.rotation * IK.tangent, axis) * parent.rotation;
            constrain_spin(q, -parent.phiMax, -parent.phiMin);
        }

        public void constrain_spin_forward(Quaternion parent)
        {
            constrain_spin(parent, phiMin, phiMax);
        }

        private void constrain_spin(Quaternion parent, float min, float max)
        {
            Vector3 axis = parent * IK.tangent;
            reorient(axis);
            float angle;
            Vector3 angle_axis;
            (rotation * Quaternion.Inverse(parent)).ToAngleAxis(out angle, out angle_axis);//This returns an infinite axis if there is no net rotation. We also get angles greater than 180 degrees sometimes
            if (angle > 180f) angle -= 360f;
            if (angle < -180f) angle += 360f;
            float debug_thing = Vector3.Dot(axis, angle_axis.normalized);
            Debug.Assert((!float.IsInfinity(angle_axis.x) && (debug_thing > 0.99f || debug_thing < -0.99f)) || angle < 0.1f);
            if (Vector3.Dot(axis, angle_axis) < 0f) angle = -angle;
            rotation = Quaternion.AngleAxis(Mathf.Clamp(angle, min, max), axis) * parent;
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
                Vector3 straight = rotation * IK.tangent;
                if (reverse) straight = -straight;
                float min = (reverse) ? -thetaMax : thetaMin;
                float max = (reverse) ? -thetaMin : thetaMax;
                float angle = Mathf.Clamp(Vector3.SignedAngle(straight, dir, n), min, max);
                return Quaternion.AngleAxis(angle, n) * straight;
            }
            if (type == JointType.ball_and_socket)
            {
                Debug.Assert(psiMax >= 0f && psiMax < 90f);
                float r = Mathf.Tan(psiMax);//TODO: cache this on Awake?
                Matrix4x4 o2w = get_coord_space();
                Matrix4x4 w2o = o2w.transpose;
                Vector3 diro = w2o * dir;
                bool right_way = (reverse && diro.z < 0f || !reverse && diro.z > 0f);//"Is dir pointing in the same approximate direction as the joint? (is their dot product positive?)"
                float x, y;
                float m = diro.y / diro.x;
                if (float.IsInfinity(m) || float.IsNaN(m))
                {
                    if (right_way && Mathf.Abs(diro.y / diro.z) < r) return dir.normalized;

                    x = 0f;
                    y = Mathf.Sign(diro.y) * r;
                } else
                {
                    float max_abs_x = r / Mathf.Sqrt(1f + m * m);
                    if (right_way && Mathf.Abs(diro.x / diro.z) <= max_abs_x) return dir.normalized;

                    x = Mathf.Sign(diro.x) * max_abs_x;
                    y = m * x;
                }
                
                return (o2w * new Vector3(x, y, (reverse) ? -1f : 1f)).normalized;
            }
            return dir.normalized;
        }

        public Matrix4x4 get_coord_space()
        {
            return new Matrix4x4(rotation * IK.normal, rotation * IK.binormal, rotation * IK.tangent, new Vector4(0f, 0f, 0f, 1f));
        }

        private enum JointType
        {
            free,
            hinge,
            ball_and_socket
        }
    }

    private enum Axis
    {
        x,
        y,
        z
    }
}