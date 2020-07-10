using UnityEngine;

public class LookAt : MonoBehaviour
{
    [SerializeField] private Transform thing;

    // Update is called once per frame
    void Update()
    {
        transform.LookAt(thing, Vector3.right);
    }
}
