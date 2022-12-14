using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DisplayRigidBody : MonoBehaviour
{
    Rigidbody rb;

    public bool drawCOM;
    public Vector3 local_com;
    public Vector3 global_com;

    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        /*
        If you don't set the center of mass from a script it will be calculated automatically from all colliders attached to the rigidbody. 
        After a custom center of mass set, it will no longer be recomputed automatically on modifications such as adding or removing colliders, 
        translating them, scaling etc. To revert back to the automatically computed center of mass, use Rigidbody.ResetCenterOfMass.

        Setting the center of mass is often useful when simulating cars to make them more stable. A car with a lower center of mass is less likely to topple over.

        Note: centerOfMass is relative to the transform's position and rotation, but will not reflect the transform's scale!
        */

        local_com = rb.centerOfMass;
        global_com = rb.worldCenterOfMass;
    }

    void OnDrawGizmosSelected()
    {
        if (drawCOM)
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(global_com, 0.1f);
    }
}
