using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Simple Movement (transform.translate) + Walking/Turning animations. 
/// If no Rigid Body and Capsule Collider are added, the character "flies" around.
/// Otherwise, a Rigid Body and Capsule Collider will make the assets react to the Physics Engine and collide agains those objects with colliders as well.
/// </summary>
public class Drive : MonoBehaviour
{
    #region Variables

    [Header("Basic Setup")]
    [Range(0, 5f)]  public float speed = 0.5f;
    [Range(0, 200f)] public float rotationSpeed = 60.0f;
    Animator anim;

    #endregion

    #region Initialization

    void Start()
    {
        anim = this.GetComponent<Animator>();

        if (anim == null)
            Debug.LogError("We require " + transform.name + " game object to have an animator! This will allow for Foot IK to function");
    }

    #endregion

    // Update is called once per frame
    void Update()
    {
        // GetAxis gives between 0 and 1 - To make velocity/second and not velocity/frame, we multiply by deltaTime.
        float translation = Input.GetAxis("Vertical") * speed * Time.deltaTime;
        float rotation = Input.GetAxis("Horizontal") * rotationSpeed * Time.deltaTime;

        anim.SetFloat("inputZ", Input.GetAxis("Vertical"));
        anim.SetFloat("inputX", Input.GetAxis("Horizontal"));

        transform.Translate(0, 0, translation);
        transform.Rotate(0, rotation, 0);

        if (translation > 0)
        {
            anim.SetBool("isWalking", true);
            anim.SetFloat("direction", 1);
        }
        else if (translation < 0)
        {
            anim.SetBool("isWalking", true);
            anim.SetFloat("direction", -1);
        }
        else
        {
            anim.SetBool("isWalking", false);
        }

        if (rotation > 0)
        {
            anim.SetBool("isTurning", true);
        }
        else if (rotation < 0)
        {
            anim.SetBool("isTurning", true);
        }
        else
        {
            anim.SetBool("isTurning", false);
        }
    }
}
