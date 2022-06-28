using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class testCube : MonoBehaviour
{
    [Header("Global Transform")]
    public Vector3 transformPosition;

    [Header("Local Transform")]
    public Vector3 transformLocalPosition;

    [Header("Directions Transform")]
    public Vector3 transformPositionForward;
    public Vector3 transformPositionUp;
    public Vector3 transformPositionRight;

    [Header("Quaternion Slerp")]
    public Transform from;
    public Transform to;
    //private float timeCount = 0.0f;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transformPosition = this.transform.position;

        transformLocalPosition = this.transform.localPosition;

        transformPositionForward = this.transform.forward;
        transformPositionUp = this.transform.up;
        transformPositionRight = this.transform.right;

        //transform.rotation = Quaternion.Slerp(from.rotation, to.rotation, timeCount);
        //timeCount = timeCount + Time.deltaTime;

        //Debug.Log("position in x: " + this.transform.position.x);

    }
}
