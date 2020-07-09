using UnityEngine;

public class AnimationController : MonoBehaviour
{
    private Animator animator;

    // Start is called before the first frame update
    void Start()
    {
        animator = GetComponent<Animator>();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown("i"))
        {
            animator.Play("Idle");
        } else if (Input.GetKeyDown("w"))
        {
            animator.Play("Wave");
        } else if (Input.GetKeyDown("h"))
        {
            animator.Play("Handshake");
        }
    }
}
