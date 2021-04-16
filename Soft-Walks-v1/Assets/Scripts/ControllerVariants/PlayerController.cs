using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Character Controller: To move the player according to the environment (the colliders) - it does NOT respond nor uses physics in any way. It comes with a Capsule Collider.
/// It will move the player like with a Transform, but can't go through colliders. 
/// </summary>
public class PlayerController : MonoBehaviour
{
    #region Variables

    [Header("Basic Setup")]
    [Range(0, 10f)] public float speed = 0.5f;
    [Range(0, 200f)] public float rotationSpeed = 100.0f;
    Animator anim;
    private CharacterController controller; // For using Character Controller
    
    private float gravity;
    private float JumpHeight = 2f;
    private Vector3 characterVelocity;

    // To check if it is grounded.
    [Header("External Method to check isGrounded")]
    [Range(0, 2f)] public float groundDistance = 0.2f;
    public LayerMask ground; // In Inspector, Unity capitalizes the label to "Ground".
    [SerializeField] private bool isGrounded;
    private Transform groundChecker;

    #endregion

    #region Initialization

    // Start is called before the first frame update
    void Start()
    {
         // Sphere must be the last child of the parent.
        groundChecker = this.transform.GetChild(transform.childCount - 1);

        // Uses gravity from physics system.
        gravity = Physics.gravity.y;

        anim = this.GetComponent<Animator>();
        controller = this.GetComponent<CharacterController>();

        // Change skin width because for take it closer to ground
        controller.skinWidth = 0.01f;

        if (anim == null)
            Debug.LogError("We require " + transform.name + " game object to have an Animator!");

        if (controller == null)
            Debug.LogError("We require " + transform.name + " game object to have a Character Controller!");
    }

    #endregion

    // Update is called once per frame
    void Update()
    {
        // isGrounded is true if the "imaginary" sphere on the empty gameObject hits some collider. If so, gravity stops increasing.
        isGrounded = Physics.CheckSphere(groundChecker.position, groundDistance, ground, QueryTriggerInteraction.Ignore);
        if (isGrounded && characterVelocity.y < 0)
            characterVelocity.y = 0f;

        Debug.Log("Is it grounded? " + isGrounded);
        Debug.Log("Gravity: " + characterVelocity.y);

        // In this case, we use a CharacterController to move our character.
        // Two methods to move using a CharacterController:
        // - SimpleMove: Character responds to gravity (only physic you get with the controller). However, Y axis velocity is ignored in this method.
        // - Move: It takes in parameters the absolute movement. Therefore, it's framerate dependent and you have to implement gravity on your own.
        // This is special for Character Controllers: We need to program gravity and other forces.

        // Vector 3 to store control movement
        Vector3 move = new Vector3(Input.GetAxis("Horizontal"), 0, Input.GetAxis("Vertical"));
        Debug.Log(move);

        // characterVelocity keeps track of the velocity caused by other effects. We need to add gravity in y.
        characterVelocity.y += gravity * Time.deltaTime;
        
        // The character moves according to the user input and external forces such as gravity.
        controller.Move(move * Time.deltaTime * speed);
        controller.Move(characterVelocity * Time.deltaTime);

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
 
        // Jump action, modifying the velocity in y and then letting gravity to affect it.
        if (Input.GetKeyDown(KeyCode.Space) && isGrounded)
            characterVelocity.y += Mathf.Sqrt(JumpHeight * -2f * gravity);
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
