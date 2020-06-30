using System;
using UnityEngine;

[Serializable]
public class IKJoint : MonoBehaviour
{
    [SerializeField]
    private JointType type;
    [SerializeField]
    private Axis align_axis;
    [SerializeField]
    private float phiMin = -180f, phiMax = 180f; // Constraints for orientation in degrees, from -180 to 180
    [SerializeField]
    private Axis hinge_axis; // Normal vector of hinge plane
    [SerializeField]
    private float thetaMin = -180f, thetaMax = 180f; // hinges only
    [SerializeField]
    private float psiMax = 180f; // ball_and_sockets only

    [NonSerialized]
    public Vector3 position;
    [NonSerialized]
    public Quaternion rotation;
    [NonSerialized]
    public float length;

    private Quaternion local_frame;
    private Quaternion local_space { get { return rotation * local_frame; } }

    public Vector3 tangent { get; private set; }

    void Awake()
    {
        switch (align_axis)
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
    }

    //Reorients to a specific direction
    public void reorient(Vector3 dir)
    {
        rotation = Quaternion.FromToRotation(rotation * tangent, dir) * rotation;
    }

    public void constrain_twist_backward(IKJoint other, Vector3 axis)
    {
        Quaternion q = Quaternion.FromToRotation(other.rotation * tangent, axis) * other.rotation;
        constrain_twist(q, -other.phiMax, -other.phiMin);
    }

    public void constrain_twist_forward(Quaternion other)
    {
        constrain_twist(other, phiMin, phiMax);
    }

    private void constrain_twist(Quaternion other, float min, float max)
    {
        Vector3 axis = other * tangent;
        reorient(axis);
        float angle;
        Vector3 angle_axis;
        (rotation * Quaternion.Inverse(other)).ToAngleAxis(out angle, out angle_axis); // This returns an infinite axis sometimes. We also get angles greater than 180 degrees sometimes
        if (float.IsInfinity(angle_axis.x)) return;

        Debug.Assert(angle >= 0f);
        if (angle > 180f) angle -= 360f;
        float dot = Vector3.Dot(axis, angle_axis);
        Debug.Assert(Mathf.Abs(dot) > 0.99f || Mathf.Abs(angle) < 1f); // TODO: why is this failing again?
        if (Mathf.Abs(dot) < 0.99f && Mathf.Abs(angle) > 1f)
        {
            Debug.Log(Mathf.Abs(dot) + " " + Mathf.Abs(angle)); // DEBUG
        }
        if (dot < 0f) angle = -angle;
        rotation = Quaternion.AngleAxis(Mathf.Clamp(angle, min, max), axis) * other;
    }

    //Constrains the direction vector between from and to, according to from's constrants. Returns a unit vector.
    public Vector3 get_direction(Vector3 to, bool reverse = false)
    {
        return constrain_direction(to - position, reverse);
    }

    //Constrains a direction vector according to angle constraints. Returns a unit vector. dir does not need to be normalized.
    public Vector3 constrain_direction(Vector3 dir, bool reverse = false)
    {
        if (type == JointType.hinge)
        {
            /*Alternative implementation which is somehow slower in Unity? TODO: needs to be tested again since replacing Matrix4x4 with Quaternion
            Matrix4x4 o2w = get_coord_space();
            Matrix4x4 w2o = o2w.transpose;
            Debug.Assert(o2w * w2o == Matrix4x4.identity);//Orthonormality check

            Vector3 diro = w2o * dir;
            ref float right = ref diro.x, up = ref diro.y, forward = ref diro.z;

            ref float a = ref diro.x, b = ref diro.y, c = ref diro.z;
            bool flip = reverse;
            Debug.Assert(axis != IK.alignment_axis);
            switch (IK.alignment_axis)
            {
                case Axis.x:
                    right = ref diro.z;
                    up = ref diro.x;
                    forward = ref diro.y;

                    a = ref right;
                    switch (axis)
                    {
                        case Axis.y:
                            b = ref forward;
                            c = ref up;
                            flip = !flip;
                            break;
                        default:
                            b = ref up;
                            c = ref forward;
                            break;
                    }
                    break;
                case Axis.y:
                    right = ref diro.y;
                    up = ref diro.z;
                    forward = ref diro.x;

                    a = ref up;
                    switch (axis)
                    {
                        case Axis.z:
                            b = ref right;
                            c = ref forward;
                            flip = !flip;
                            break;
                        default:
                            b = ref forward;
                            c = ref right;
                            break;
                    }
                    break;
                default:
                    a = ref forward;
                    switch (axis)
                    {
                        case Axis.x:
                            b = ref up;
                            c = ref right;
                            flip = !flip;
                            break;
                        default:
                            b = ref up;
                            c = ref forward;
                            break;
                    }
                    break;
            }

            float norm = Mathf.Sqrt(a * a + b * b);
            Debug.Assert(norm > 0.0001f);
            float cos_theta = ((reverse) ? -a : a) / norm;
            Debug.Assert(!float.IsInfinity(cos_theta) && !float.IsNaN(cos_theta));
            float angle = (flip ^ b < 0f) ? -Mathf.Acos(cos_theta) : Mathf.Acos(cos_theta);
            float min = Mathf.Deg2Rad * ((reverse) ? -thetaMax : thetaMin);
            float max = Mathf.Deg2Rad * ((reverse) ? -thetaMin : thetaMax);
            angle = Mathf.Clamp(angle, min, max);

            a = (reverse) ? -Mathf.Cos(angle) : Mathf.Cos(angle);
            b = (flip) ? -Mathf.Sin(angle) : Mathf.Sin(angle);
            c = 0f;

            return o2w * diro;
            */

            Debug.Assert(hinge_axis != align_axis);
            Vector3 n = rotation * ((hinge_axis == Axis.x) ? Vector3.right :
                                    (hinge_axis == Axis.y) ? Vector3.up :
                                                       Vector3.forward);
            dir -= Vector3.Dot(dir, n) * n;// Planar projection
            Vector3 straight = rotation * tangent;
            if (reverse) straight = -straight;
            float min = (reverse) ? -thetaMax : thetaMin;
            float max = (reverse) ? -thetaMin : thetaMax;
            float angle = Mathf.Clamp(Vector3.SignedAngle(straight, dir, n), min, max);

            return Quaternion.AngleAxis(angle, n) * straight;
        }
        if (type == JointType.ball_and_socket)
        {
            Quaternion o2w = local_space;
            Quaternion w2o = Quaternion.Inverse(o2w);

            float x, y, z;

            Debug.Assert(psiMax >= 0f && psiMax <= 180f);
            float r = Mathf.Abs(Mathf.Tan(psiMax * Mathf.Deg2Rad));//TODO: cache this on Awake?
            bool obtuse = psiMax > 90f;

            Vector3 diro = w2o * dir;
            bool right_way = diro.z > 0f ^ reverse;//"Is dir pointing in the same approximate direction as the joint? (is their dot product positive?)"
            if (float.IsInfinity(r))
            {
                if (right_way) return dir.normalized;

                x = diro.x;
                y = diro.y;
                z = 0f;
            }
            else
            {
                float m = diro.y / diro.x;
                if (float.IsInfinity(m) || float.IsNaN(m))
                {
                    float y_proj = Mathf.Abs(diro.y / diro.z);
                    Debug.Assert(!float.IsNaN(y_proj));
                    if (right_way && y_proj <= r || obtuse && y_proj >= r) return dir.normalized;

                    x = 0f;
                    y = Mathf.Sign(diro.y) * r;
                    z = (reverse ^ obtuse) ? -1f : 1f;
                }
                else
                {
                    float max_abs_x = r / Mathf.Sqrt(1f + m * m);
                    float x_proj = Mathf.Abs(diro.x / diro.z);
                    Debug.Assert(!float.IsNaN(x_proj));
                    if (right_way && x_proj <= max_abs_x || obtuse && x_proj >= max_abs_x) return dir.normalized;

                    x = Mathf.Sign(diro.x) * max_abs_x;
                    y = m * x;
                    z = (reverse ^ obtuse) ? -1f : 1f;
                }
            }

            Vector3 debug = o2w * new Vector3(x, y, z).normalized;
            Debug.Assert(!float.IsNaN(debug.x) && !float.IsInfinity(debug.x));
            return debug;
        }
        return dir.normalized;
    }

    

    private enum JointType
    {
        free,
        hinge,
        ball_and_socket
    }
}
