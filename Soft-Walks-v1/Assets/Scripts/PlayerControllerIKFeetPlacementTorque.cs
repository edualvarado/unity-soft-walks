using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEditor;
using UnityEngine;

[RequireComponent(typeof(CharacterController))]
public class PlayerControllerIKFeetPlacementTorque : MonoBehaviour
{
    #region Variables

    [Header("Movement Information")]
    public float InputX; // Range between -1 and 1 for horizontal axis (sides).
    public float InputZ; // Range between -1 and 1 for vertical axis (forward).
    public Vector3 desiredMoveDirection;

    [Header("Thresholds of movement")]
    public float allowPlayerRotation; // When we want the player to start rotating.
    public float Speed;

    [Header("Rotation Information")]
    public bool blockRotationPlayer; // For static animation forward.
    public float desiredRotationSpeed;
    
    public Animator anim;
    public CharacterController controller;
    public Camera cam;

    // Adding RigidBody.
    private Rigidbody rb;

    private float gravity;
    private Vector3 characterVelocity; // Measures the velocity of the character at each time.

    // To check if it is grounded.
    [Header("External Method to check isGrounded")]
    [Range(0, 2f)] public float groundDistance = 0.2f;
    public LayerMask ground; // In Inspector, Unity capitalizes the label to "Ground".
    [SerializeField] private bool isGrounded;
    private Transform groundChecker;

    [Header("Feet Positions")]
    private Vector3 rightFootPosition;
    private Vector3 leftFootPosition;
    private Vector3 rightFootIKPosition;
    private Vector3 leftFootIKPosition;

    private Quaternion leftFootIKRotation, rightFootIKRotation;
    private float lastPelvisPositionY, lastRightFootPositionY, lastLeftFootPositionY;

    [Header("Feet Grounder")]
    public bool enableFeetIK = true;
    [Range(0, 20f)] [SerializeField] private float heightFromGroundRaycast = 1.15f;
    [Range(0, 20f)] [SerializeField] private float raycastDownDistance = 1.5f;
    [SerializeField] private LayerMask environmentLayer;
    [SerializeField] private float pelvisOffset = 0f;
    [Range(0, 1f)] [SerializeField] private float pelvisUpAndDownSpeed = 0.3f;
    [Range(0, 1f)] [SerializeField] private float feetToIKPositionSpeed = 0.5f;

    public string leftFootAnimVariableName = "LeftFootCurve";
    public string rightFootAnimVariableName = "RightFootCurve";

    public bool useProIKFeature = false;
    public bool showSolverDebug = true;

    // New - Necessary for PD Controller

    [Header("Terrain Information")]
    private Terrain terrain;
    private TerrainData terrain_data;
    private int heightmap_width;
    private int heightmap_height;
    private float[,] heightmap_data;
    private float[,] heightmap_buffer;
    private float[,] heightmap_filtered;
    private int[,] contourmap_data;
    private int[,] contourmap_buffer;
    public bool showTerrainDebug = true;
    private float slopeAngle = 0.0f;
    private float slopeAngleChloe = 0.0f;

    [Header("PD Controller")]
    [SerializeField] private float alpha = 30.0f;
    [SerializeField] private float beta = 6.0f;
    private float targetRbPosition;
    private float targetRbPosition2;
    private float tau;
    public bool showTargetDebug = true;
    public bool addTorque = true;

    #endregion

    // Start is called before the first frame update.
    void Start()
    {
        // System Info
        System.Threading.Thread.CurrentThread.CurrentCulture = new System.Globalization.CultureInfo("en-US");

        // Takes terrain instance and save terrain information.
        GetTerrain();

        // For groundCheck.
        groundChecker = this.transform.GetChild(transform.childCount - 1); // Sphere must be the last child of the parent.
        gravity = Physics.gravity.y; // Uses gravity from physics system.

        // Getting components from Inspector.
        //anim = GetComponent<Animator>();
        anim = GetComponentInChildren<Animator>();
        cam = Camera.main;
        controller = GetComponent<CharacterController>();

        // Adding RigidBody.
        rb = GetComponent<Rigidbody>();

        // Set Environment Layer
        environmentLayer = LayerMask.GetMask("Environment");
    }

    // Update is called once per frame.
    void Update()
    {
        // For Character controlling.
        InputMagnitude();

        // Gets Terrain Slope angle (sign changes if going up or down).
        GetTerrainSlope();

        // Ground Check: isGrounded is true if the "imaginary" sphere on the empty gameObject hits some collider. If so, gravity stops increasing.
        isGrounded = Physics.CheckSphere(groundChecker.position, groundDistance, ground, QueryTriggerInteraction.Ignore);
        if (isGrounded && characterVelocity.y < 0)
            characterVelocity.y = 0f;
        Debug.Log("Velocity (Y) inc. Gravity: " + characterVelocity.y);

        // If it not grounded , apply gravity and move in Y.
        characterVelocity.y += gravity * Time.deltaTime;
        controller.Move(characterVelocity * Time.deltaTime);
    }

    /// <summary>
    /// Method to calculate input vectors.
    /// </summary>
    void InputMagnitude()
    {
        // Calculate Input Vectors
        InputX = Input.GetAxis("Horizontal");
        InputZ = Input.GetAxis("Vertical");

        // Set Floats into the animator equal to the inputs.
        anim.SetFloat("InputZ", InputZ, 0.0f, Time.deltaTime);
        anim.SetFloat("InputX", InputX, 0.0f, Time.deltaTime);

        // Calculate InputMagnitude (Blend (Speed)) - Better for controller axis - How much you are pressing the key or joystick.
        // Calculate the squared magnitude instead of the magnitude is much faster (it doesn´t need to do the square root).
        Speed = new Vector2(InputX, InputZ).sqrMagnitude;

        // Physically move player w/ animations
        if (Speed > allowPlayerRotation)
        {
            anim.SetFloat("InputMagnitude", Speed, 0.0f, Time.deltaTime);
            PlayerMoveAndRotation();
        }
        else if (Speed < allowPlayerRotation)
        {
            anim.SetFloat("InputMagnitude", Speed, 0.0f, Time.deltaTime);
        }
    }

    /// <summary>
    /// Method in charge of moving the player.
    /// </summary>
    void PlayerMoveAndRotation()
    {
        InputX = Input.GetAxis("Horizontal");
        InputZ = Input.GetAxis("Vertical");

        // Set normalized unit vectors for the camera - var could replace Vector3
        // Vector3.right is a vector facing the world right. It will always be (1, 0, 0)
        // transform.right is a vector facing the local-space right, meaning it is a vector that faces to the right of your object. 
        // This vector will be different depending on which way your object with the transform is facing.
        var camera = Camera.main;
        Vector3 forward = cam.transform.forward; // Z direction with respect to the global axis. 
        Vector3 right = cam.transform.right; // X direction with respect to the global axis.

        // Convert to 2D planar coordinates and normalize.
        forward.y = 0f;
        right.y = 0f;
        forward.Normalize();
        right.Normalize();

        // forward and right are normalized vectors.
        desiredMoveDirection = forward * InputZ + right * InputX;

        // Rotates the character between the current angle (transform.rotation) to the forward direction of the vector in which you are moving according to InputX and InputZ.
        // It can be modified to certain time or speed.
        if(blockRotationPlayer == false)
        {
            transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(desiredMoveDirection), desiredRotationSpeed);
        }
    }

    #region FeetGrounding

    /// <summary>
    /// Update the AdjustFeetTarget method and also find the position of each foot inside our Solver Position.
    /// </summary>
    private void FixedUpdate()
    {
        if(enableFeetIK == false) { return; }
        if(anim == null) { return; }

        AdjustFeetTarget(ref rightFootPosition, HumanBodyBones.RightFoot);
        AdjustFeetTarget(ref leftFootPosition, HumanBodyBones.LeftFoot);

        // Find a raycast to the ground to find positions.
        FeetPositionSolver(rightFootPosition, ref rightFootIKPosition, ref rightFootIKRotation); // Handle the solver for right foot
        FeetPositionSolver(leftFootPosition, ref leftFootIKPosition, ref leftFootIKRotation); // Handle the solver for left foot
    }

    /// <summary>
    /// Called when IK Pass is activated.
    /// </summary>
    /// <param name="layerIndex"></param>
    private void OnAnimatorIK(int layerIndex)
    {
        if (anim == null) { return; }

        MovePelvisHeight();

        // RightFoot IK Position  - Max. IK position.
        anim.SetIKPositionWeight(AvatarIKGoal.RightFoot, 1);

        // RightFoot IK Rotation  - for PRO feature.
        if (useProIKFeature)
        {
            anim.SetIKRotationWeight(AvatarIKGoal.RightFoot, anim.GetFloat(rightFootAnimVariableName));
        }

        // Move RightFoot to the target IK position and rotation.
        MoveFeetToIKPoint(AvatarIKGoal.RightFoot, rightFootIKPosition, rightFootIKRotation, ref lastRightFootPositionY);

        // LeftFoot IK Position  - Max. IK position.
        anim.SetIKPositionWeight(AvatarIKGoal.LeftFoot, 1);

        // LeftFoot IK Rotation  - for PRO feature.
        if (useProIKFeature)
        {
            anim.SetIKRotationWeight(AvatarIKGoal.LeftFoot, anim.GetFloat(leftFootAnimVariableName));
        }

        // Move LeftFoot to the target IK position and rotation.
        MoveFeetToIKPoint(AvatarIKGoal.LeftFoot, leftFootIKPosition, leftFootIKRotation, ref lastLeftFootPositionY);

        BalanceCharacter();
    }

    /// <summary>
    /// Uses PD Controller to balance character.
    /// </summary>
    private void BalanceCharacter()
    {
        float dotRightFoot = Vector3.Dot(rightFootPosition, transform.forward);
        float dotLeftFoot = Vector3.Dot(leftFootPosition, transform.forward);

        if(showTargetDebug)
        {
            /*
            string footAhead = dotRightFoot > dotLeftFoot ? "Right" : "Left";
            Debug.Log("[RightFoot] Dot product: " + dotRightFoot);
            Debug.Log("[LeftFoot] Dot product: " + dotLeftFoot);
            Debug.Log("Foot Ahead: " + footAhead);
            Debug.Log("=====================================================");
            */
        }

        if ((dotRightFoot <= dotLeftFoot)) //Left foot in front of right foot
        {
            targetRbPosition = Vector3.Dot(rightFootPosition, transform.forward) +
                ((leftFootPosition - rightFootPosition).magnitude / 2.0f) * Mathf.Cos(slopeAngleChloe * 3.1415f / 180.0f);

            if (showTargetDebug)
            {
                //Debug.Log("LEFT FOOT in front of RIGHT FOOT");
                //Debug.Log("[RightFoot] Dot product: " + Vector3.Dot(rightFootPosition, transform.forward));
                //Debug.Log("Vector3 of Left Foot wrt Right Foot: " + (leftFootPosition - rightFootPosition));
                //Debug.Log("Half of the magnitude: " + ((leftFootPosition - rightFootPosition).magnitude / 2.0f));
                //Debug.Log("Z-comp of such half magnitude: " + ((leftFootPosition - rightFootPosition).magnitude / 2.0f) * Mathf.Cos(slopeAngle * Mathf.Deg2Rad));
                //Debug.Log("targetRbPosition: " + targetRbPosition);
                //Debug.Log("=====================================================");

                Debug.DrawLine(leftFootPosition, rightFootPosition, Color.red);
                Debug.DrawLine(Vector3.zero, rightFootPosition + ((leftFootPosition - rightFootPosition) / 2.0f), Color.blue);
            }

        }
        else //Right foot in front of left foot
        {
            targetRbPosition = Vector3.Dot(leftFootPosition, transform.forward)
                + ((rightFootPosition - leftFootPosition).magnitude / 2.0f) * Mathf.Cos(slopeAngleChloe * 3.1415f / 180.0f);

            if (showTargetDebug)
            {
                
                //Debug.Log("LEFT FOOT in front of RIGHT FOOT");
                //Debug.Log("[RightFoot] Dot product: " + Vector3.Dot(rightFootPosition, transform.forward));
                //Debug.Log("Vector3 of Left Foot wrt Right Foot: " + (leftFootPosition - rightFootPosition));
                //Debug.Log("Half of the magnitude: " + ((leftFootPosition - rightFootPosition).magnitude / 2.0f));
                //Debug.Log("Z-comp of such half magnitude: " + ((leftFootPosition - rightFootPosition).magnitude / 2.0f) * Mathf.Cos(slopeAngle * Mathf.Deg2Rad));
                //Debug.Log("targetRbPosition: " + targetRbPosition);
                //Debug.Log("=====================================================");

                Debug.DrawLine(leftFootPosition, rightFootPosition, Color.blue);
                Debug.DrawLine(Vector3.zero, leftFootPosition + ((rightFootPosition - leftFootPosition) / 2.0f), Color.red);
            }
        }

        float tau = alpha * (Vector3.Dot(anim.bodyPosition, transform.forward) - targetRbPosition)
            + beta * (Vector3.Dot(rb.angularVelocity, transform.right) - 0);

        /*
        Debug.Log("targetRbPosition: " + targetRbPosition);
        Debug.Log("PosDiff: " + (Vector3.Dot(anim.bodyPosition, transform.forward) - targetRbPosition));
        Debug.Log("AngVelDiff: " + (Vector3.Dot(rb.angularVelocity, transform.right) - 0));
        Debug.Log("tau: " + tau);
        */

        if (addTorque)
        {
            Debug.Log("[INFO] Adding torque (tau:  " + tau + ")  to character");
            rb.AddTorque(-transform.right * tau);
        }
    }

    #endregion

    #region FeetGroundingMethods

    /// <summary>
    /// Move feet to the IK target.
    /// </summary>
    /// <param name="foot"></param>
    /// <param name="positionIKHolder"></param>
    /// <param name="rotationIKHolder"></param>
    /// <param name="lastFootPositionY"></param>
    void MoveFeetToIKPoint(AvatarIKGoal foot, Vector3 positionIKHolder, Quaternion rotationIKHolder, ref float lastFootPositionY)
    {
        //  Get the current position of the foot, which we are going to move.
        Vector3 targetIKPosition = anim.GetIKPosition(foot);

        // If there is a IK target in a different position (not 0 locally) than the position where we have our foot currently.
        if (positionIKHolder != Vector3.zero)
        {
            // Convert the world coordinates for current/target foot positions to local coordinates with respect to the character.
            targetIKPosition = transform.InverseTransformPoint(targetIKPosition);
            positionIKHolder = transform.InverseTransformPoint(positionIKHolder);

            // Calculate the translation in Y necessary to move the last foot position to the target position, by a particular speed.
            float yVariable = Mathf.Lerp(lastFootPositionY, positionIKHolder.y, feetToIKPositionSpeed);

            // Add this desired translation in Y to our current feet position.
            targetIKPosition.y += yVariable;

            // We update the last foot position in Y.
            lastFootPositionY = yVariable;

            // Convert the current foot position to world coordinates.
            targetIKPosition = transform.TransformPoint(targetIKPosition);

            // Set the new goal rotation (world coordinates) for the foot.
            anim.SetIKRotation(foot, rotationIKHolder);
        }

        // Set the new goal position (world coordinates) for the foot.
        anim.SetIKPosition(foot, targetIKPosition);
    }

    /// <summary>
    /// Adapt height of pelvis - TODO: REVIEW
    /// </summary>
    private void MovePelvisHeight()
    {
        if(rightFootIKPosition == Vector3.zero || leftFootIKPosition == Vector3.zero || lastPelvisPositionY == 0)
        {
            lastPelvisPositionY = anim.bodyPosition.y;
            return;
        }

        float leftOffsetPosition = leftFootIKPosition.y - transform.position.y;
        float rightOffsetPosition = rightFootIKPosition.y - transform.position.y;

        float totalOffset = (leftOffsetPosition < rightOffsetPosition) ? leftOffsetPosition: rightOffsetPosition;

        // Hold new pelvis position where we want to move to.
        // Move from last to new position with certain speed.
        Vector3 newPelvisPosition = anim.bodyPosition + Vector3.up * totalOffset;
        newPelvisPosition.y = Mathf.Lerp(lastPelvisPositionY, newPelvisPosition.y, pelvisUpAndDownSpeed);

        // Update current body position.
        anim.bodyPosition = newPelvisPosition;

        // Now the last known pelvis position in Y is the current body position in Y.
        lastPelvisPositionY = anim.bodyPosition.y;
    }

    /// <summary>
    /// Locate the feet position via a raycast and then solving.
    /// </summary>
    /// <param name="fromSkyPosition"></param>
    /// <param name="feetIKPositions"></param>
    /// <param name="feetIKRotations"></param>
    private void FeetPositionSolver(Vector3 fromSkyPosition, ref Vector3 feetIKPositions, ref Quaternion feetIKRotations)
    {
        // To store all the info regarding the hit of the ray
        RaycastHit feetoutHit;

        // To visualize the ray
        if (showSolverDebug)
        {
            Debug.DrawLine(fromSkyPosition, fromSkyPosition + Vector3.down * (raycastDownDistance + heightFromGroundRaycast), Color.yellow);
        }

        // If the ray, starting at the sky position, goes down certain distance and hits an environment layer.
        if (Physics.Raycast(fromSkyPosition, Vector3.down, out feetoutHit, raycastDownDistance + heightFromGroundRaycast, environmentLayer))
        {
            // Position the new IK feet positions parallel to the sky position, and put them where the ray intersects with the environment layer.
            feetIKPositions = fromSkyPosition;
            feetIKPositions.y = feetoutHit.point.y + pelvisOffset;
            // Creates a rotation from the (0,1,0) to the normal of where the feet is placed it in the terrain.
            feetIKRotations = Quaternion.FromToRotation(Vector3.up, feetoutHit.normal) * transform.rotation;

            return;
        }

        feetIKPositions = Vector3.zero; // If we reach this, it didn't work.
    }

    /// <summary>
    /// Adjust the IK target for the feet.
    /// </summary>
    /// <param name="feetPositions"></param>
    /// <param name="foot"></param>
    private void AdjustFeetTarget(ref Vector3 feetPositions, HumanBodyBones foot)
    {
        // Takes the Vector3 transform of that human bone id.
        feetPositions = anim.GetBoneTransform(foot).position;
        feetPositions.y = transform.position.y + heightFromGroundRaycast;
    }

    #endregion

    #region Terrain

    /// <summary>
    /// Gets Terrain Slope angle (sign changes if going up or down).
    /// </summary>
    private void GetTerrainSlope()
    {
        // Get terrain slope
        float pos_x = rb.position.x / terrain_data.size.x;
        float pos_z = rb.position.z / terrain_data.size.z;
        Vector3 normal = terrain_data.GetInterpolatedNormal(pos_x, pos_z);
        float gradient = terrain_data.GetSteepness(pos_x, pos_z);

        // To define if character is climbing up or down with respect to its direction
        Vector3 local_normal = this.transform.InverseTransformDirection(normal);
        slopeAngle = local_normal.z < 0 ? gradient : -gradient;

        // Chloe's method
        if (normal.z < 0)
        {
            //Debug.Log("MONTEE");
            slopeAngleChloe = gradient;
        }
        else
        {
            //Debug.Log("DESCENTE");
            slopeAngleChloe = -gradient;
        }

        // To visualize the ray
        if (showTerrainDebug)
        {
            Debug.DrawLine(rb.position, rb.position + normal, Color.green);
            Debug.Log("slopeAngle: " + slopeAngle);
            Debug.Log("slopeAngleChloe: " + slopeAngleChloe);
        }
    }

    /// <summary>
    /// Takes terrain instance and save terrain information.
    /// </summary>
    private void GetTerrain()
    {
        // Get terrain information.
        if (!terrain)
            terrain = Terrain.activeTerrain;
        terrain_data = terrain.terrainData;
        heightmap_width = terrain_data.heightmapResolution;
        heightmap_height = terrain_data.heightmapResolution;
        heightmap_data = terrain_data.GetHeights(0, 0, heightmap_width, heightmap_height);
        heightmap_buffer = terrain_data.GetHeights(0, 0, heightmap_width, heightmap_height);
        heightmap_filtered = terrain_data.GetHeights(0, 0, heightmap_width, heightmap_height);
        contourmap_data = new int[heightmap_height, heightmap_width];
        contourmap_buffer = new int[heightmap_height, heightmap_width];
    }

    #endregion

}
