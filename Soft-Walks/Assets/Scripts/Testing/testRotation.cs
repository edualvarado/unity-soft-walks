using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class testRotation : MonoBehaviour
{
    [Header("Global Transform")]
    public Vector3 transformPosition;

    [Header("Local Transform")]
    public Vector3 transformLocalPosition;

    [Header("Directions Transform")]
    public Vector3 transformPositionForward;
    public Vector3 transformPositionUp;
    public Vector3 transformPositionRight;

    public Transform target;
    //private float timeCount = 0.0f;

    public Vector3 relativePos;

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

        relativePos = target.position - transform.position;

        Quaternion rotation = Quaternion.LookRotation(relativePos, Vector3.up);
        transform.rotation = rotation;

        Debug.Log("Euler Angles: " + rotation.eulerAngles);
    }
}
