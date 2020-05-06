using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public struct KeyFrame
{
    public Transform transform;
    public float period;//Time taken to move from this keyframe to the next
}

public class IKAnimation : MonoBehaviour
{
    [SerializeField]
    private Transform target;//This is the target object that we move to make the skeleton do things

    [SerializeField]
    private List<KeyFrame> keyframes;//These are the states we will move the target object between, to make the skeleton do things

    private int cur;//This keeps track of the current keyframe
    private int next;//This keeps track of the NEXT keyframe
    private float timer;//Time since the last keyframe switch

    // Start is called before the first frame update
    void Start()
    {
        cur = 0;
        Debug.Assert(keyframes.Count > 0);
        next = 1;
        timer = 0f;
    }

    // Update is called once per frame
    void Update()
    {
        timer += Time.deltaTime;
        if (timer > keyframes[cur].period)
        {
            cur = next;
            next = (next + 1) % keyframes.Count;
            timer = 0f;
        }

        float t = timer / keyframes[cur].period;

        //Interpolate position and rotation
        Transform t0 = keyframes[cur].transform;
        Transform t1 = keyframes[next].transform;
        target.position = Vector3.Lerp(t0.position, t1.position, t);
        target.rotation = Quaternion.Slerp(t0.rotation, t1.rotation, t);
    }
}
