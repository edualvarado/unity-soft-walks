using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class testChildCube : MonoBehaviour
{
    [Header("Global Transform")]
    public Vector3 transformPosition;

    [Header("Local Transform")]
    public Vector3 transformLocalPosition;

    [Header("Directions Transform")]
    public Vector3 transformPositionForward;
    public Vector3 transformPositionUp;
    public Vector3 transformPositionRight;

    [Header("Speed")]
    public float speed = 3.0f;

    //Transform transform;


    // Start is called before the first frame update
    void Start()
    {

        //transform = this.GetComponent<Transform>();

    }

    // Update is called once per frame
    void Update()
    {
        transformPosition = this.transform.position;

        transformLocalPosition = this.transform.localPosition;

        transformPositionForward = this.transform.forward;
        transformPositionUp = this.transform.up;
        transformPositionRight = this.transform.right;

        Debug.Log("Transform Parent: " + transform.parent.position);
        Debug.Log("Transform Child: " + transform.position);


        // If relativeTo is left out or set to Space.Self the movement is applied relative to the transform's local axes. 
        // (the x, y and z axes shown when selecting the object inside the Scene View.) 
        // If relativeTo is Space.World the movement is applied relative to the world coordinate system.
        if (Input.GetKey(KeyCode.Space))            
            transform.Translate(Vector3.forward * Time.deltaTime * speed);

        //Debug.Log("position in z: " + this.transform.position.z);
        //Debug.Log("local position in z: " + this.transform.localPosition.z);



    }
}
