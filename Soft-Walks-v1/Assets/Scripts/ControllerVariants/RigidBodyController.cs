using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

/// <summary>
/// RigidBody + Collider: Now, the character reacts to the Physics Systems from Unity. We need a FixedUpdate function for the physic updates, independent on the normal Update function and possibly FPS drops.
/// </summary>
public class RigidBodyController : MonoBehaviour
{
    #region Variables

    [Header("Basic Setup")]
    [Range(0, 10f)] public float speed = 0.5f;
    [Range(0, 200f)] public float rotationSpeed = 100.0f;
    Animator anim;
    private Rigidbody body; // For using Rigidbodies

    private float gravity;
    private float JumpHeight = 2f;

    // To check if it is grounded.
    [Header("External Method to check isGrounded")]
    [Range(0, 2f)] public float groundDistance = 0.2f;
    public LayerMask ground; // In Inspector, Unity capitalizes the label to "Ground".
    [SerializeField] private bool isGrounded;
    private Transform groundChecker;

    private Vector3 move = Vector3.zero; // This time we declared it because we need to use it in FixedUpdate too.

    #endregion

    #region Initialization

    // Start is called before the first frame update
    void Start()
    {
        groundChecker = this.transform.GetChild(transform.childCount - 1);

        gravity = Physics.gravity.y;

        anim = this.GetComponent<Animator>();
        body = this.GetComponent<Rigidbody>();

        if (anim == null)
            Debug.LogError("We require " + transform.name + " game object to have an Animator!");

        if (body == null)
            Debug.LogError("We require " + transform.name + " game object to have a Rigidbody!");
    }

    #endregion

    // Update is called once per frame
    void Update()
    {

        // isGrounded is true if the "imaginary" sphere on the empty gameObject hits some collider. If so, gravity stops increasing.
        // We do not need to stop gravity; it is done automatically. However we leave isGrounded bool for the jump.
        isGrounded = Physics.CheckSphere(groundChecker.position, groundDistance, ground, QueryTriggerInteraction.Ignore);
        Debug.Log("Is it grounded? " + isGrounded);

        // In this case, we use a Rigidbody and Capsule Collider to move our character.
        // This is special for Rigidbodies: We separate the physics update into the FixedUpdate.
        // This is done because in this method, the frame rates variation will not impact the simulation. 
        // We want physics to always happen smoothly.

        // Vector 3 to store control movement
        Vector3 move = new Vector3(Input.GetAxis("Horizontal"), 0, Input.GetAxis("Vertical"));
        Debug.Log(move);

        // If it is moving, we change the forward vector of the transform (z) to be the movement vector
        if (move != Vector3.zero)
        {
            transform.forward = move;
            animationForward(anim);
        }
        else
        {
            animationStop(anim);
        }

        if (Input.GetKeyDown(KeyCode.Space) && isGrounded)
            body.AddForce(Vector3.up * Mathf.Sqrt(JumpHeight * -2f * Physics.gravity.y), ForceMode.VelocityChange); // VelocityChange is mass independent.
    }

    /// <summary>
    ///  FixedUpdate is used to update the physics updates independent on the framerate.
    /// </summary>
    private void FixedUpdate()
    {
        body.MovePosition(body.position + move * speed * Time.fixedDeltaTime);
    }

    private void animationForward(Animator anim)
    {
        anim.SetBool("isWalking", true);
        anim.SetFloat("direction", 1);
    }

    private void animationStop(Animator anim)
    {
        anim.SetBool("isWalking", false);
    }
}
