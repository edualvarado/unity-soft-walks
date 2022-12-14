using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;

[RequireComponent(typeof(CharacterController))]
public class PlayerControllerIKFeetPlacementTorqueTerrainVeg : MonoBehaviour
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
    private Rigidbody rb;

    private float gravity;
    private Vector3 characterVelocity; // Measures the velocity of the character at each time.

    [Header("External Method to check isGrounded")]
    public LayerMask ground; // In Inspector, Unity capitalizes the label to "Ground".
    [Range(0, 2f)] public float groundDistance = 0.2f;
    [SerializeField] private bool isGrounded;
    [SerializeField] private Transform groundChecker;

    [Header("External Method to check if each foot isGrounded")]
    public bool showGroundedFeetDebug = true;
    [Range(0, 0.5f)] public float groundDistanceFoot = 0.04f;
    [SerializeField] private bool isLeftFootGrounded;
    [SerializeField] private bool isRightFootGrounded;
    [SerializeField] private Transform groundCheckerLeftFoot;
    [SerializeField] private Transform groundCheckerRightFoot;

    [Header("Feet Positions")]
    [SerializeField] private Vector3 rightFootPosition;
    [SerializeField] private Vector3 leftFootPosition;
    [SerializeField] private Vector3 rightFootIKPosition;
    [SerializeField] private Vector3 leftFootIKPosition;

    private Quaternion leftFootIKRotation, rightFootIKRotation;
    private float lastPelvisPositionY, lastRightFootPositionY, lastLeftFootPositionY;

    [Header("Feet Grounder (IK)")]
    public bool showSolverDebug = true; // Visualize the IK Solver for each foot.
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

    [Header("PD Controller")] // PD gains and target position/angular velocity.
    public bool showPDDebug = true; // Visualize feet walking during PD.
    public bool addTorque = true;
    [SerializeField] private float alpha = 30.0f;
    [SerializeField] private float beta = 6.0f;
    [SerializeField] private float tau;
    private float targetRbPosition;
    private float targetRbPosition2;

    [Header("Terrain Information")] // Where we save all data from the terrian.
    public bool showTerrainNormalAngleDebug = false; // Visualize terrain normal.
    [SerializeField] private float slopeAngle = 0.0f;
    private float slopeAngleChloe = 0.0f;
    private Terrain terrain;
    private TerrainData terrain_data;
    private int heightmap_width;
    private int heightmap_height;
    private float[,] heightmap_data;
    private float[,] heightmap_buffer;
    private float[,] heightmap_filtered;
    private int[,] contourmap_data;
    private int[,] contourmap_buffer;

    [Header("Terrain Deformation")] // Variables that define how the terrian is modified.
    public bool modifyTerrain = false;
    [Range(1, 10)] public int filter_coefficient = 1; // Wet Sand: 2; Dry Sand: 10 
    [Range(0, 1.0f)] public float compression_coefficient = 0.5f; // Wet Sand: 0.9f; Dry Sand: 0.3f 
    [Range(0, 1.0f)] public float depth_coefficient = 0.5f; // Wet Sand: 0.5f; Dry Sand: 0.3f

    [Header("Feet for Terrain Deformation")] // GameObjects from feet to modify the terrian.
    public bool showFootprintDebug = true; // Visualize grid for terrain deformation.
    public GameObject shoesRight;
    public GameObject soleRight;
    public GameObject shoesLeft;
    public GameObject soleLeft;

    [Header("Vegetation")] // Grass variables
    public bool showHeightsGrassDebug = true;
    public bool isGrass = false;
    public int heightGrass;
    [Range(0, 2.0f)] public float levelLowHeight = 0.2f;
    [Range(0, 2.0f)] public float levelMiddleHeight = 0.6f;
    [Range(0, 2.0f)] public float levelHighHeight = 1.2f;

    [Header("Feet for Vegetation")] // Variables that define how the terrian is modified.
    public bool showRaiseFeetDebug = true;
    public bool raiseFeet = true;
    public float leftFootDistanceToFloor;
    //public Vector3 leftFootHitToFloor;
    public float rightFootDistanceToFloor;
    //public Vector3 rightFootHitToFloor;
    public bool cubeLeftCreated = false;
    public bool cubeRightCreated = false;
    public float amountRaiseFeetUp;
    public float amountLowerFeetDown;
    public bool alreadyDestroyedLeft = false;
    public bool alreadyDestroyedRigth = false;
    [Range(0, 0.5f)] public float maxHeightThreshold = 0.2f;
    [Range(0, 0.1f)] public float minHeightThreshold = 0.03f;
    private GameObject cubeLeft;
    private GameObject cubeRight;
    private bool leftCubeGoingDown = false;
    private bool rightCubeGoingDown = false;
    public bool simulateFakeGrassGround = false;

    [Header("Hands (IK)")]
    public bool showHandsDebug = false;
    public bool activateIKHands = false;
    public Transform leftHand;
    public Transform rightHand;
    public float aWeight = 0f;
    public float bWeight = 0f;
    public bool sphereLeftCreated = false;
    public bool sphereRightCreated = false;
    private bool flagFrom1 = false;
    private bool flagFrom2 = false;
    private bool flagFrom3 = false;
    private bool flagFrom0 = false;
    private float state = 0f;
    private float elapsedTime = 0f;
    private float timeReaction = 1.5f;
    private GameObject sphereLeft;
    private GameObject sphereRight;

    private Collider soleColliderRight; // Colliders from the soles.
    private Collider soleColliderLeft;
    private int countZeros, countOnes, countTwos;
    private int count;
    private Vector3 prevLeftFootVector;
    private Vector3 prevRightFootVector;

    #endregion

    // Start is called before the first frame update.
    void Start()
    {
        // System Info
        System.Threading.Thread.CurrentThread.CurrentCulture = new System.Globalization.CultureInfo("en-US");

        // Takes terrain instance and save terrain information -> TODO: Check for extensions as well.
        GetTerrain();

        // For groundCheck.
        gravity = Physics.gravity.y; // Uses gravity from physics system.

        // Getting components from Inspector.
        anim = GetComponent<Animator>();
        cam = Camera.main;
        controller = GetComponent<CharacterController>();

        // Adding RigidBody used by PD.
        rb = GetComponent<Rigidbody>();

        // Set Environment Layer that will be affected by IK.
        environmentLayer = LayerMask.GetMask("Environment");

        // To move shoes according to the terrain -> TODO: Check how it works.
        prevLeftFootVector = anim.GetBoneTransform(HumanBodyBones.LeftToes).position - anim.GetBoneTransform(HumanBodyBones.LeftFoot).position;
        prevRightFootVector = anim.GetBoneTransform(HumanBodyBones.RightToes).position - anim.GetBoneTransform(HumanBodyBones.RightFoot).position;

        // Get Mesh Colliders from soles.
        soleColliderRight = soleRight.GetComponent<Collider>();
        soleColliderLeft = soleLeft.GetComponent<Collider>();
    }

    // Update is called once per frame.
    void Update()
    {
        // For Character controlling.
        InputMagnitude();

        // Gets Terrain Slope angle (sign changes if going up or down).
        GetTerrainSlope();

        // Check if whole body is grounded. If not, applies gravity.
        CheckBodyIsGrounded();

        // Check Grass Level
        detectGrassLevel();
    }

    /// <summary>
    /// Update the AdjustFeetTarget method and also find the position of each foot inside our Solver Position.
    /// </summary>
    private void FixedUpdate()
    {
        if (enableFeetIK == false) { return; }
        if (anim == null) { return; }

        AdjustFeetTarget(ref rightFootPosition, HumanBodyBones.RightFoot);
        AdjustFeetTarget(ref leftFootPosition, HumanBodyBones.LeftFoot);

        // Find a raycast to the ground to find positions.
        FeetPositionSolver(rightFootPosition, ref rightFootIKPosition, ref rightFootIKRotation); // Handle the solver for right foot
        FeetPositionSolver(leftFootPosition, ref leftFootIKPosition, ref leftFootIKRotation); // Handle the solver for left foot

        // Check if each foot is grounded.
        CheckFeetAreGrounded();
    }

    #region Motion

    /// <summary>
    /// Check if each foot is grounded.
    /// </summary>
    private void CheckFeetAreGrounded()
    {
        // IMPORTANT BUGFIX: Now, one only is grounded if the other is not. AVOID THAT?
        // isGrounded for each foot - BUGFIX: When walking on plants, do not know why it behaves strange. The if's avoids that.
        
        if(!isRightFootGrounded)
        {
            isLeftFootGrounded = Physics.CheckSphere(groundCheckerLeftFoot.position, groundDistanceFoot, ground, QueryTriggerInteraction.Ignore);
            //isLeftFootGrounded = Physics.CheckSphere(groundCheckerLeftFoot.position + (-0.4f * transform.right), groundDistanceFoot, ground, QueryTriggerInteraction.Ignore);
        }

        if (!isLeftFootGrounded)
        {
            isRightFootGrounded = Physics.CheckSphere(groundCheckerRightFoot.position, groundDistanceFoot, ground, QueryTriggerInteraction.Ignore);
            //isRightFootGrounded = Physics.CheckSphere(groundCheckerRightFoot.position + (0.4f * transform.right), groundDistanceFoot, ground, QueryTriggerInteraction.Ignore);
        }
        

        // New - BUGFIX: GET DISTANCE FROM FEET TO GROUND AVOIDING EXTRA FAKE GROUND - Done by initiating the ray on the sides.
        RaycastHit leftHit;
        Ray downLeftRay = new Ray(groundCheckerLeftFoot.position + (-0.4f * transform.right), Vector3.down);
        if (Physics.Raycast(downLeftRay, out leftHit))
        {
            if (leftHit.collider.CompareTag("Ground"))
            {
                leftFootDistanceToFloor = leftHit.distance;
            }
        }

        RaycastHit rightHit;
        Ray downRightRay = new Ray(groundCheckerRightFoot.position + (0.4f * transform.right), Vector3.down);
        if (Physics.Raycast(downRightRay, out rightHit))
        {
            if (rightHit.collider.CompareTag("Ground"))
            {
                rightFootDistanceToFloor = rightHit.distance;
            }
        }

        if (showGroundedFeetDebug)
        {
            //Debug.Log("leftFootDistanceToFloor: " + leftFootDistanceToFloor);
            //Debug.Log("rightFootDistanceToFloor: " + rightFootDistanceToFloor);

            //Debug.Log("[INFO] isLeftFootGrounded? " + isLeftFootGrounded);
            if (isLeftFootGrounded)
                Debug.DrawRay(groundCheckerLeftFoot.position + (-0.4f * transform.right), Vector3.down, Color.blue);
            else
                Debug.DrawRay(groundCheckerLeftFoot.position + (-0.4f * transform.right), Vector3.down, Color.red);

            //Debug.Log("[INFO] isRightFootGrounded? " + isRightFootGrounded);
            if (isRightFootGrounded)
                Debug.DrawRay(groundCheckerRightFoot.position + (0.4f * transform.right), Vector3.down, Color.blue);
            else
                Debug.DrawRay(groundCheckerRightFoot.position + (0.4f * transform.right), Vector3.down, Color.red);
        }
    }

    /// <summary>
    /// Check if whole body is grounded. If not, applies gravity.
    /// </summary>
    private void CheckBodyIsGrounded()
    {
        // Ground Check: isGrounded is true if the "imaginary" sphere on the groundDistance gameObject hits some collider. If so, gravity stops increasing.
        isGrounded = Physics.CheckSphere(groundChecker.position, groundDistance, ground, QueryTriggerInteraction.Ignore);
        if (isGrounded && characterVelocity.y < 0)
            characterVelocity.y = 0f;

        //Debug.Log("[INFO] Velocity (Y) inc. Gravity: " + characterVelocity.y);

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

    #endregion

    #region FeetGrounding

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

        // RightHand IK Position - Weight is a placeholder, is changed later.
        anim.SetIKPositionWeight(AvatarIKGoal.RightHand, 0.75f);

        // RightFoot IK Rotation  - for PRO feature.
        if (useProIKFeature)
        {
            anim.SetIKRotationWeight(AvatarIKGoal.RightFoot, anim.GetFloat(rightFootAnimVariableName));
        }

        // Move RightFoot to the target IK position and rotation.
        MoveFeetToIKPoint(AvatarIKGoal.RightFoot, rightFootIKPosition, rightFootIKRotation, ref lastRightFootPositionY);

        // Move RightHand to the target IK position.
        if (activateIKHands)
        {
            MoveHandToIKPoint(AvatarIKGoal.RightHand);
        }

        // LeftFoot IK Position  - Max. IK position.
        anim.SetIKPositionWeight(AvatarIKGoal.LeftFoot, 1);

        // RightHand IK Position - Weight is a placeholder, is changed later.
        anim.SetIKPositionWeight(AvatarIKGoal.LeftHand, 0.75f);

        // LeftFoot IK Rotation  - for PRO feature.
        if (useProIKFeature)
        {
            anim.SetIKRotationWeight(AvatarIKGoal.LeftFoot, anim.GetFloat(leftFootAnimVariableName));
        }

        // Move LeftFoot to the target IK position and rotation.
        MoveFeetToIKPoint(AvatarIKGoal.LeftFoot, leftFootIKPosition, leftFootIKRotation, ref lastLeftFootPositionY);

        // Move LeftHand to the target IK position.
        if (activateIKHands)
        {
            MoveHandToIKPoint(AvatarIKGoal.LeftHand);
        }

        // PD to balance character (must be called on OnAnimatorIK).
        BalanceCharacter();

        // Modify the terrain with respect to the foot placement.
        if (modifyTerrain && !isGrass)
        {
            ModifyTerrain();
        }

        // Raising feet
        if (raiseFeet)
        {
            RaiseTerrain();
        }
    }

    #endregion

    #region PDController

    /// <summary>
    /// Uses PD Controller to balance character.
    /// </summary>
    private void BalanceCharacter()
    {
        float dotRightFoot = Vector3.Dot(rightFootPosition, transform.forward);
        float dotLeftFoot = Vector3.Dot(leftFootPosition, transform.forward);

        if(showPDDebug)
        {
            /*
            Debug.Log("=====================================================");
            string footAhead = dotRightFoot > dotLeftFoot ? "Right" : "Left";
            Debug.Log("[INFO] Foot Ahead: " + footAhead);
            */
        }

        if ((dotRightFoot <= dotLeftFoot)) //Left foot in front of right foot
        {
            // Where we want the CoM to be in the horizontal plane.
            targetRbPosition = Vector3.Dot(rightFootPosition, transform.forward) +
                ((leftFootPosition - rightFootPosition).magnitude / 2.0f) * Mathf.Cos(slopeAngleChloe * 3.1415f / 180.0f);

            if (showPDDebug)
            {
                /*
                Debug.Log("[RightFoot] Dot product: " + dotRightFoot);
                Debug.Log("[INFO] Vector3 of Left Foot wrt Right Foot: " + (leftFootPosition - rightFootPosition));
                Debug.Log("[INFO] Half of the magnitude: " + ((leftFootPosition - rightFootPosition).magnitude / 2.0f));
                Debug.Log("[INFO] Z-comp of such half magnitude: " + ((leftFootPosition - rightFootPosition).magnitude / 2.0f) * Mathf.Cos(slopeAngleChloe * Mathf.Deg2Rad));
                Debug.Log("[INFO] targetRbPosition: " + targetRbPosition);
                */

                Debug.DrawLine(leftFootPosition, rightFootPosition, Color.red);
                Debug.DrawLine(Vector3.zero, rightFootPosition + ((leftFootPosition - rightFootPosition) / 2.0f), Color.blue);
            }

        }
        else //Right foot in front of left foot
        {
            // Where we want the CoM to be in the horizontal plane.
            targetRbPosition = Vector3.Dot(leftFootPosition, transform.forward)
                + ((rightFootPosition - leftFootPosition).magnitude / 2.0f) * Mathf.Cos(slopeAngleChloe * 3.1415f / 180.0f);

            if (showPDDebug)
            {
                /*
                Debug.Log("[LeftFoot] Dot product: " + dotLeftFoot);
                Debug.Log("[INFO] Vector3 of Right Foot wrt Left Foot: " + (rightFootPosition - leftFootPosition));
                Debug.Log("[INFO] Half of the magnitude: " + ((rightFootPosition - leftFootPosition).magnitude / 2.0f));
                Debug.Log("[INFO] Z-comp of such half magnitude: " + ((leftFootPosition - rightFootPosition).magnitude / 2.0f) * Mathf.Cos(slopeAngleChloe * Mathf.Deg2Rad));
                Debug.Log("[INFO] targetRbPosition: " + targetRbPosition);
                */

                Debug.DrawLine(leftFootPosition, rightFootPosition, Color.blue);
                Debug.DrawLine(Vector3.zero, leftFootPosition + ((rightFootPosition - leftFootPosition) / 2.0f), Color.red);
            }
        }
           
        // Calculate current position and angular velocity (variables to be controlled by the PD).
        float currentPos = Vector3.Dot(anim.bodyPosition, transform.forward);
        float currentAngVel = Vector3.Dot(rb.angularVelocity, transform.right);

        // PD controller
        float tau = alpha * (currentPos - targetRbPosition) + beta * (currentAngVel - 0);

        if (addTorque)
        {
            //Debug.Log("[INFO] Adding torque (tau:  " + tau + ")  to character!");
            rb.AddTorque(-transform.right * tau);
        }

        if (heightGrass == 3 && isGrass)
        {
            float grassForce = 8.0f;
            float grassForce2 = 5.0f;

            // First force
            rb.AddForceAtPosition(transform.forward * grassForce, transform.position + 0.2f * (transform.up), ForceMode.Force);
            
            // Second force
            rb.AddForceAtPosition(transform.forward * grassForce2, transform.position + 0.4f * (transform.up), ForceMode.Force);

            beta = 3.2f; // before 3.2

            if (showHeightsGrassDebug)
            {
                // First force
                Debug.DrawRay(transform.position + 0.2f * (transform.up), -transform.forward * (grassForce / 10), Color.blue);

                // Second force
                Debug.DrawRay(transform.position + 0.4f * (transform.up), -transform.forward * (grassForce2 / 10), Color.blue);
            }
        }
        else if (heightGrass == 2 && isGrass)
        {

            float grassForce = 6.0f;
            rb.AddForceAtPosition(transform.forward * grassForce, transform.position + 0.2f * (transform.up), ForceMode.Force);
            beta = 4.0f;

            if (showHeightsGrassDebug)
            {
                Debug.DrawRay(transform.position + 0.2f * (transform.up), -transform.forward * (grassForce / 10), Color.blue);
            }
        }
        else if (heightGrass == 1 && isGrass)
        {

            float grassForce = 2.0f;
            rb.AddForceAtPosition(transform.forward * grassForce, transform.position + 0.2f * (transform.up), ForceMode.Force);
            beta = 5.0f;

            if (showHeightsGrassDebug)
            {
                Debug.DrawRay(transform.position + 0.5f * (transform.up), -transform.forward * (grassForce / 10), Color.blue);
            }
        }
        else
        {
            beta = 6.0f;
        }
    }

    #endregion

    #region HandMethods

    void MoveHandToIKPoint(AvatarIKGoal hand)
    {   
        // For future improvement: Extend to more heights of grass and raise hands accordingly with LERP.
        switch(heightGrass)
        {
            case 0:
                if (!flagFrom0)
                {
                    //Debug.Log("ENTERING IN 0 - Do not raise hands");
                    elapsedTime = 0;
                    aWeight = bWeight;
                    bWeight = 0f;
                }

                flagFrom0 = true;
                flagFrom1 = false;
                flagFrom2 = false;
                flagFrom3 = false;

                break;
            case 1:
                if (!flagFrom1)
                {
                    //Debug.Log("ENTERING IN 1 - Do not raise hands");
                    elapsedTime = 0;
                    aWeight = bWeight;
                    bWeight = 0f;
                }

                flagFrom0 = false;
                flagFrom1 = true;
                flagFrom2 = false;
                flagFrom3 = false;

                break;
            case 2:
                // For now, levels 2 and 3 of grass are considered the same for the arms.
                if (!flagFrom2)
                {
                    //Debug.Log("ENTERING IN 2/3 - Raise hands");
                    elapsedTime = 0;
                    aWeight = bWeight;
                    bWeight = 0.1f;
                }

                flagFrom0 = false;
                flagFrom1 = false;
                flagFrom2 = true;
                flagFrom3 = false;

                break;
            case 3:
                // For now, levels 2 and 3 of grass are considered the same for the arms.
                if (!flagFrom3)
                {
                    //Debug.Log("ENTERING IN 2/3 - Raise hands");
                    elapsedTime = 0;
                    aWeight = bWeight;
                    bWeight = 0.7f;
                }

                flagFrom0 = false;
                flagFrom1 = false;
                flagFrom2 = false;
                flagFrom3 = true;

                break;
            default:
                Debug.LogError("[ERROR] Grass height miss-clasified");
                break;
        }

        if (hand == AvatarIKGoal.LeftHand)
        {
            if (!sphereLeftCreated)
            {
                // Create sphere for the first time.
                sphereLeft = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphereLeft.GetComponent<Collider>().enabled = false;
                sphereLeft.transform.SetParent(transform.parent, false);

                MeshRenderer renderLeftSphere = sphereLeft.GetComponent<MeshRenderer>();
                if (showHandsDebug)
                {
                    renderLeftSphere.enabled = true;
                }
                else
                {
                    renderLeftSphere.enabled = false;
                }

                // Original Starting Point
                sphereLeft.transform.position = new Vector3(leftHand.position.x, leftHand.position.y - 0.2f, leftHand.position.z);
                sphereLeft.transform.localPosition = new Vector3(sphereLeft.transform.localPosition.x - 0.1f, sphereLeft.transform.localPosition.y + 0.5f, sphereLeft.transform.localPosition.z + 0.35f);

                sphereLeft.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);

                // Set flag to sphere created.
                sphereLeftCreated = true;
            }

            else if (sphereLeftCreated)
            {
                // If the sphere has been created, we just move it.                    
                sphereLeft.transform.position = new Vector3(leftHand.position.x, leftHand.position.y - 0.2f, leftHand.position.z);
                sphereLeft.transform.localPosition = new Vector3(sphereLeft.transform.localPosition.x - 0.1f, sphereLeft.transform.localPosition.y + 0.5f, sphereLeft.transform.localPosition.z + 0.35f);

                // NEW - SMOOTH WEIGHT
                elapsedTime += Time.deltaTime;
                state = Mathf.Lerp(aWeight, bWeight, elapsedTime / timeReaction);

                anim.SetIKPositionWeight(AvatarIKGoal.LeftHand, state);
               
                anim.SetIKPosition(hand, sphereLeft.transform.position);
            }
        }

        if ((hand == AvatarIKGoal.RightHand))
        {
            if (!sphereRightCreated)
            {
                // Create sphere for the first time.
                sphereRight = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphereRight.GetComponent<Collider>().enabled = false;
                sphereRight.transform.SetParent(transform.parent, false);

                MeshRenderer renderRightSphere = sphereRight.GetComponent<MeshRenderer>();
                if (showHandsDebug)
                {
                    renderRightSphere.enabled = true;
                }
                else
                {
                    renderRightSphere.enabled = false;
                }

                // Original Starting Point
                sphereRight.transform.position = new Vector3(rightHand.position.x, rightHand.position.y - 0.2f, rightHand.position.z);
                sphereRight.transform.localPosition = new Vector3(sphereRight.transform.localPosition.x + 0.1f, sphereRight.transform.localPosition.y + 0.5f, sphereRight.transform.localPosition.z + 0.35f);

                sphereRight.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);

                // Set flag to cube created.
                sphereRightCreated = true;
            }
            else if (sphereRightCreated)
            {
                // If the sphere has been created, we just move it forwards and backwards.                    
                sphereRight.transform.position = new Vector3(rightHand.position.x, rightHand.position.y - 0.2f, rightHand.position.z);
                sphereRight.transform.localPosition = new Vector3(sphereRight.transform.localPosition.x + 0.1f, sphereRight.transform.localPosition.y + 0.5f, sphereRight.transform.localPosition.z + 0.55f);

                // NEW - SMOOTH WEIGHT
                elapsedTime += Time.deltaTime;
                state = Mathf.Lerp(aWeight, bWeight, elapsedTime / timeReaction);

                anim.SetIKPositionWeight(AvatarIKGoal.RightHand, state);
                
                anim.SetIKPosition(hand, sphereRight.transform.position);
            }
        }

        // For now, spheres are always created, and we change the IK weight between grass levels. We do not need to destroy them.
        /*
        if (!isGrass)
        {
            sphereRightCreated = false;
            sphereLeftCreated = false;
            Destroy(sphereRight);
            Destroy(sphereLeft);
        }
        */
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

    #region TerrainExtraction

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
        if (showTerrainNormalAngleDebug)
        {
            Debug.DrawLine(rb.position, rb.position + normal, Color.cyan);
            Debug.Log("[INFO] slopeAngle: " + slopeAngle);
            Debug.Log("[INFO] slopeAngleChloe: " + slopeAngleChloe);
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

    #region TerrainModification

    /// <summary>
    /// Modify the terrain. First, sets a grid of columns around each based on the height map resolution.
    /// Then, cast a ray from each IK foot position going upwards. If the ray (which correspond to each column of the grid)
    /// hits the sole collider, it modifies the terrain in that particular point and around too. Finally saves the terrain data.
    /// </summary>
    private void ModifyTerrain()
    {
        // Change feet positions for Terrain Deformacion -> TODO:  Check why is used.
        MoveShoes();

        // Change terrain as a function of the foot positions TODO
        // TODO: Check why is used.
        count = 0;

        // If we modify the terrain at each moment, some noise appears. We need to only do it when each individual foot is grounded.
        /*
        // Modify contourmap with raycasting.
        SetContourMap(leftFootIKPosition);
        SetContourMap(rightFootIKPosition);

        // Set contours of the foot prints.
        SetOnes(leftFootIKPosition);
        SetOnes(rightFootIKPosition);

        // Holes under the feet.
        SetHeightMap(leftFootIKPosition);
        SetHeightMap(rightFootIKPosition);

        // Blur the contours and iterate.
        FilterHeightMap(heightmap_buffer);
        for (int i = 1; i < filter_coefficient; i++)
        {
            FilterHeightMap(heightmap_filtered);
        }

        // Save changes.
        terrain_data.SetHeights(0, 0, heightmap_filtered);
        */
        
        // Now, only terrain gets modified when the foot is grounded. We can change the distance of the checkSphere to alter this.
        // Also is faster, since updates only happen when the foot are grounded.
        if (isLeftFootGrounded)
        {
            // Modify contourmap with raycasting.
            SetContourMap(leftFootIKPosition);

            // Set contours of the foot prints.
            SetOnes(leftFootIKPosition);

            // Holes under the feet.
            SetHeightMap(leftFootIKPosition);

            // Blur the contours and iterate.
            FilterHeightMap(heightmap_buffer);
            for (int i = 1; i < filter_coefficient; i++)
            {
                FilterHeightMap(heightmap_filtered);
            }

            // Save changes.
            terrain_data.SetHeights(0, 0, heightmap_filtered);
        }

        if (isRightFootGrounded)
        {
            // Modify contourmap with raycasting.
            SetContourMap(rightFootIKPosition);

            // Set contours of the foot prints.
            SetOnes(rightFootIKPosition);

            // Holes under the feet.
            SetHeightMap(rightFootIKPosition);

            // Blur the contours and iterate.
            FilterHeightMap(heightmap_buffer);
            for (int i = 1; i < filter_coefficient; i++)
            {
                FilterHeightMap(heightmap_filtered);
            }

            // Save changes.
            terrain_data.SetHeights(0, 0, heightmap_filtered);
        }
    }

    /// <summary>
    /// TODO: Check why it is used.
    /// </summary>
    private void MoveShoes()
    {
        //Translation
        shoesLeft.transform.Translate((leftFootIKPosition - shoesLeft.transform.position), Space.World);
        shoesRight.transform.Translate((rightFootIKPosition - shoesRight.transform.position), Space.World);

        //Rotation
        float rotationAngleLeft;
        float rotationAngleRight;

        Vector3 currentLeftFootVector = anim.GetBoneTransform(HumanBodyBones.LeftToes).position - anim.GetBoneTransform(HumanBodyBones.LeftFoot).position;
        Vector3 currentRightFootVector = anim.GetBoneTransform(HumanBodyBones.RightToes).position - anim.GetBoneTransform(HumanBodyBones.RightFoot).position;

        if (currentLeftFootVector.z - prevLeftFootVector.z <= 0)
        {
            rotationAngleLeft = Vector3.Angle(prevLeftFootVector, currentLeftFootVector);
        }
        else
        {
            rotationAngleLeft = -Vector3.Angle(prevLeftFootVector, currentLeftFootVector);
        }

        if (currentRightFootVector.z - prevRightFootVector.z <= 0)
        {
            rotationAngleRight = Vector3.Angle(prevRightFootVector, currentRightFootVector);
        }
        else
        {
            rotationAngleRight = -Vector3.Angle(prevRightFootVector, currentRightFootVector);
        }

        shoesLeft.transform.Rotate(transform.forward, rotationAngleLeft);
        shoesRight.transform.Rotate(transform.forward, rotationAngleRight);

        prevLeftFootVector = currentLeftFootVector;
        prevRightFootVector = currentRightFootVector;
    }

    /// <summary>
    /// Creates a grid around each sole.
    /// </summary>
    /// <param name="targetFoot"></param>
    private void SetContourMap(Vector3 targetFoot)
    {
        // Feet location in the terrain with respect to the heighmap resolution (e.g. [0, 512])
        int indX = (int)((targetFoot.x / terrain_data.size.x) * heightmap_width);
        int indZ = (int)((targetFoot.z / terrain_data.size.z) * heightmap_height);

        // Out of bounds: Close to the limits of the terrain in heightmap scale.
        if ((indX <= 20) || (indZ <= 20) || (indX > heightmap_width - 20) || (indZ > heightmap_height - 20))
        {
            Debug.Log("OUT OF BOUNDS");
            return;
        }

        // 20 since is around the foot.
        for (int i = -20; i < 20; i++)
        {
            for (int j = -20; j < 20; j++)
            {
                //Debug.Log("Targetfoot lower than floor");
                // World Space Coordinates of each cell or column on the grid.
                Vector3 columnPos = new Vector3((indX + i) * terrain_data.size.x / heightmap_width, targetFoot.y, (indZ + j) * terrain_data.size.z / heightmap_height);

                //Debug.DrawRay(columnPos, Vector3.up, Color.white);

                Ray ray = new Ray();
                ray.origin = columnPos;
                ray.direction = Vector3.up;
                RaycastHit outHit;

                if (targetFoot == rightFootIKPosition && soleColliderRight.Raycast(ray, out outHit, 2.0f))
                {
                    if (showFootprintDebug)
                    {
                        //Debug.Log("[INFO] Hit right sole");
                        Debug.DrawRay(columnPos, Vector3.up, Color.blue);
                    }

                    contourmap_data[indZ + j, indX + i] = 2;
                    contourmap_buffer[indZ + j, indX + i] = 2;
                    count += 1;
                }
                else if (targetFoot == leftFootIKPosition && soleColliderLeft.Raycast(ray, out outHit, 2.0f))
                {
                    if (showFootprintDebug)
                    {
                        //Debug.Log("[INFO] Hit left sole");
                        Debug.DrawRay(columnPos, Vector3.up, Color.red);
                    }

                    contourmap_data[indZ + j, indX + i] = 2;
                    contourmap_buffer[indZ + j, indX + i] = 2;
                    count += 1;
                }

                // Need to see why this if! Works better without that constraint.
                /*
                // If the foot are below the terrain.
                if (targetFoot.y < heightmap_data[indZ + j, indX + i] * terrain_data.heightmapScale.y)
                {
                    //Debug.Log("Targetfoot lower than floor");
                    // World Space Coordinates of each cell or column on the grid.
                    Vector3 columnPos = new Vector3((indX + i) * terrain_data.size.x / heightmap_width, targetFoot.y, (indZ + j) * terrain_data.size.z / heightmap_height);

                    //Debug.DrawRay(columnPos, Vector3.up, Color.white);

                    Ray ray = new Ray();
                    ray.origin = columnPos;
                    ray.direction = Vector3.up;
                    RaycastHit outHit;

                    if (targetFoot == rightFootIKPosition && soleColliderRight.Raycast(ray, out outHit, 2.0f))
                    {
                        if(showFootprintDebug)
                        {
                            //Debug.Log("[INFO] Hit right sole");
                            Debug.DrawRay(columnPos, Vector3.up, Color.blue);
                        }

                        contourmap_data[indZ + j, indX + i] = 2;
                        contourmap_buffer[indZ + j, indX + i] = 2;
                        count += 1;
                    }
                    else if (targetFoot == leftFootIKPosition && soleColliderLeft.Raycast(ray, out outHit, 2.0f))
                    {
                        if (showFootprintDebug)
                        {
                            //Debug.Log("[INFO] Hit left sole");
                            Debug.DrawRay(columnPos, Vector3.up, Color.red);
                        }

                        contourmap_data[indZ + j, indX + i] = 2;
                        contourmap_buffer[indZ + j, indX + i] = 2;
                        count += 1;
                    }
                }
                */
            }
        }
    }

    //Contourmap cells set to one around the foot
    private void SetOnes(Vector3 targetFoot)
    {
        //Indices of foot transform in grid
        int indX = (int)((targetFoot.x / terrain_data.size.x) * heightmap_width);
        int indZ = (int)((targetFoot.z / terrain_data.size.z) * heightmap_height);

        //Out of bounds
        if ((indX <= 15) || (indZ <= 15) || (indX > heightmap_width - 15) || (indZ > heightmap_height - 15))
        {
            return;
        }

        for (int i = -15; i < 15; i++)
        {
            for (int j = -15; j < 15; j++)
            {
                if (contourmap_data[indZ + j, indX + i] == 0)
                {
                    if ((contourmap_data[indZ + j - 1, indX + i + 1] == 2) || (contourmap_data[indZ + j - 1, indX + i - 1] == 2)
                        || (contourmap_data[indZ + j - 1, indX + i] == 2) || (contourmap_data[indZ + j + 1, indX + i + 1] == 2)
                        || (contourmap_data[indZ + j + 1, indX + i - 1] == 2) || (contourmap_data[indZ + j + 1, indX + i] == 2)
                        || (contourmap_data[indZ + j, indX + i - 1] == 2) || (contourmap_data[indZ + j, indX + i + 1] == 2))
                    {
                        contourmap_data[indZ + j, indX + i] = 1;
                        contourmap_buffer[indZ + j, indX + i] = 1;
                    }
                }
            }
        }
    }

    private void SetHeightMap(Vector3 targetFoot)
    {
        int indX = (int)((targetFoot.x / terrain_data.size.x) * heightmap_width);
        int indZ = (int)((targetFoot.z / terrain_data.size.z) * heightmap_height);

        if ((indX <= 20) || (indZ <= 20) || (indX > heightmap_width - 20) || (indZ > heightmap_height - 20))
        {
            return;
        }

        //Decrease for foot and increase surrounding cells
        for (int i = -20; i < 20; i++)
        {
            for (int j = -20; j < 20; j++)
            {
                if (contourmap_buffer[indZ + j, indX + i] == 2)
                {
                    heightmap_buffer[indZ + j, indX + i] = heightmap_data[indZ + j, indX + i] - ((0.1f * depth_coefficient) / terrain_data.heightmapScale.y);
                    contourmap_buffer[indZ + j, indX + i] = 0;
                }
                else if (contourmap_buffer[indZ + j, indX + i] == 1)
                {
                    heightmap_buffer[indZ + j, indX + i] = heightmap_data[indZ + j, indX + i] + (0.003f * (1.0f - compression_coefficient) * count / terrain_data.heightmapScale.y);
                    contourmap_buffer[indZ + j, indX + i] = 0;
                }
            }
        }
        
        // Without filtering
        //terrain_data.SetHeights(0, 0, heightmap_buffer);
    }

    private void FilterHeightMap(float[,] heightmap)
    {
        float[,] result = terrain_data.GetHeights(0, 0, heightmap_width, heightmap_height);

        int indX = (int)((transform.position.x / terrain_data.size.x) * heightmap_width);
        int indZ = (int)((transform.position.z / terrain_data.size.z) * heightmap_height);

        if ((indX <= 20) || (indZ <= 20) || (indX > heightmap_width - 20) || (indZ > heightmap_height - 20))
        {
            return;
        }

        //Decrease for foot and increase surrounding cells
        for (int i = -20; i < 20; i++)
        {
            for (int j = -20; j < 20; j++)
            {
                //Gaussian filter 3x3
                /*                result[indZ + j, indX + i] = heightmap[indZ + j - 1, indX + i - 1]
                                    + heightmap[indZ + j - 1, indX + i + 1]
                                    + heightmap[indZ + j + 1, indX + i - 1]
                                   + heightmap[indZ + j + 1, indX + i + 1]
                                   + 2 * (heightmap[indZ + j - 1, indX + i]
                                   + heightmap[indZ + j + 1, indX + i]
                                   + heightmap[indZ + j, indX + i - 1]
                                   + heightmap[indZ + j, indX + i + 1])
                                   + 4 * heightmap[indZ + j, indX + i];

                                heightmap[indZ + j, indX + i] *= 1.0f / 16.0f;*/

                //Gaussian filter 5x5
                result[indZ + j, indX + i] =
                    heightmap[indZ + j - 2, indX + i - 2]
                    + 4 * heightmap[indZ + j - 2, indX + i - 1]
                    + 6 * heightmap[indZ + j - 2, indX + i]
                    + heightmap[indZ + j - 2, indX + i + 2]
                    + 4 * heightmap[indZ + j - 2, indX + i + 1]
                    + 4 * heightmap[indZ + j - 1, indX + i + 2]
                    + 16 * heightmap[indZ + j - 1, indX + i + 1]
                    + 4 * heightmap[indZ + j - 1, indX + i - 2]
                    + 16 * heightmap[indZ + j - 1, indX + i - 1]
                    + 24 * heightmap[indZ + j - 1, indX + i]
                    + 6 * heightmap[indZ + j, indX + i - 2]
                    + 24 * heightmap[indZ + j, indX + i - 1]
                    + 6 * heightmap[indZ + j, indX + i + 2]
                    + 24 * heightmap[indZ + j, indX + i + 1]
                    + 36 * heightmap[indZ + j, indX + i]
                    + heightmap[indZ + j + 2, indX + i - 2]
                    + 4 * heightmap[indZ + j + 2, indX + i - 1]
                    + 6 * heightmap[indZ + j + 2, indX + i]
                    + heightmap[indZ + j + 2, indX + i + 2]
                    + 4 * heightmap[indZ + j + 2, indX + i + 1]
                    + 4 * heightmap[indZ + j + 1, indX + i + 2]
                    + 16 * heightmap[indZ + j + 1, indX + i + 1]
                    + 4 * heightmap[indZ + j + 1, indX + i - 2]
                    + 16 * heightmap[indZ + j + 1, indX + i - 1]
                    + 24 * heightmap[indZ + j + 1, indX + i];

                result[indZ + j, indX + i] *= 1.0f / 256.0f;
            }
        }

        heightmap_filtered = result;
    }

    #endregion

    #region Grass

    /// <summary>
    /// Detect the grass height.
    /// </summary>
    void detectGrassLevel()
    {
        Vector3 aboveKneesHeight = transform.position + levelHighHeight * (Vector3.up);
        Vector3 kneesHeight = transform.position + levelMiddleHeight * (Vector3.up);
        Vector3 belowKneesHeight = transform.position + levelLowHeight * (Vector3.up);
        Vector3 feetHeight = transform.position;

        if (showHeightsGrassDebug)
        {
            Debug.DrawRay(aboveKneesHeight, transform.forward, Color.red);
            Debug.DrawRay(kneesHeight, transform.forward, Color.red);
            Debug.DrawRay(belowKneesHeight, transform.forward, Color.red);
            Debug.DrawRay(feetHeight, transform.forward, Color.red);
        }

        if (Physics.CheckSphere(feetHeight, 0.01f, LayerMask.GetMask("Vegetation")))
        {
            isGrass = true;
            if (showHeightsGrassDebug)
                Debug.DrawRay(feetHeight, transform.forward, Color.cyan);
        }
        else
        {
            isGrass = false;
        }

        if (isGrass)
        {
            if (Physics.CheckSphere(aboveKneesHeight, 0.1f, LayerMask.GetMask("Vegetation")))
            {
                if (showHeightsGrassDebug)
                    Debug.DrawRay(aboveKneesHeight, transform.forward, Color.cyan);
                heightGrass = 3;
            }
            else if (Physics.CheckSphere(kneesHeight, 0.1f, LayerMask.GetMask("Vegetation")))
            {
                if (showHeightsGrassDebug)
                    Debug.DrawRay(kneesHeight, transform.forward, Color.cyan);
                heightGrass = 2;
            }
            else if (Physics.CheckSphere(belowKneesHeight, 0.1f, LayerMask.GetMask("Vegetation")))
            {
                if (showHeightsGrassDebug)
                    Debug.DrawRay(belowKneesHeight, transform.forward, Color.cyan);
                heightGrass = 1;
            }
        }
        else
        {
            heightGrass = 0;
        }
    }

    #endregion

    #region RaiseFeet

    /// <summary>
    /// Raise the feet based on the height of the grass.
    /// </summary>
    void RaiseTerrain()
    {
        // Create primitives for each foot-
        createPrimitive(leftFootIKPosition);
        createPrimitive(rightFootIKPosition);   
    }

    /// <summary>
    /// Create the floating platform that will raise the feet when the grass is detected.
    /// </summary>
    /// <param name="targetFoot"></param>
    private void createPrimitive(Vector3 targetFoot)
    {

        // Change the amount with which we raise our feet wrt. grass.
        switch (heightGrass)
        {
            case 0:
                break;
            case 1:
                amountRaiseFeetUp = 0.3f;
                amountLowerFeetDown = 1.2f;
                maxHeightThreshold = 0.15f;
                minHeightThreshold = 0.03f;

                break;
            case 2:
                amountRaiseFeetUp = 0.4f;
                amountLowerFeetDown = 1.2f;
                maxHeightThreshold = 0.20f;
                minHeightThreshold = 0.03f;

                break;
            case 3:
                amountRaiseFeetUp = 0.5f;
                amountLowerFeetDown = 1.2f;
                maxHeightThreshold = 0.19f;
                minHeightThreshold = 0.03f;

                break;
            default:
                Debug.LogError("[ERROR] Grass height miss-clasified");
                break;
        }

        // For each foot...
        if (targetFoot == leftFootIKPosition && isGrass)
        {
            // If left foot is on the air...
            if (!isLeftFootGrounded && isRightFootGrounded && !cubeRight)
            {
                // If we have not created the floating platform yet...
                if (!cubeLeftCreated && !alreadyDestroyedLeft)
                {
                    cubeLeft = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cubeLeft.transform.SetParent(transform.parent, false);

                    // New - Does not work
                    /*
                    Collider colliderCubeLeft = cubeLeft.GetComponent<Collider>();
                    colliderCubeLeft.material.bounciness = 1;
                    colliderCubeLeft.material.bounceCombine = PhysicMaterialCombine.Multiply;
                    soleColliderLeft.material.bounciness = 1;
                    */

                    MeshRenderer renderLeft = cubeLeft.GetComponent<MeshRenderer>();
                    if (showRaiseFeetDebug)
                    {
                        renderLeft.enabled = true;
                    }
                    else
                    {
                        renderLeft.enabled = false;
                    }

                    // Always orient to character's rotation.
                    cubeLeft.transform.Rotate(0f, this.transform.localEulerAngles.y, 0f); 

                    // Original Starting Point
                    cubeLeft.transform.position = new Vector3(leftFootIKPosition.x, leftFootIKPosition.y - 0.1f, leftFootIKPosition.z);
                    cubeLeft.transform.localPosition = new Vector3(cubeLeft.transform.localPosition.x - 0.10f, cubeLeft.transform.localPosition.y, cubeLeft.transform.localPosition.z + 0.10f);

                    cubeLeft.transform.localScale = new Vector3(0.3f, 0.1f, 0.3f);

                    cubeLeft.layer = LayerMask.NameToLayer("Environment");

                    // Set flag to cube created.
                    cubeLeftCreated = true;
                }
                else if (cubeLeftCreated && !alreadyDestroyedLeft)
                {
                    // If the cube has been created, we just move it in x and z based on the IK position. For Y, we raise it later.                    
                    cubeLeft.transform.position = new Vector3(leftFootIKPosition.x, cubeLeft.transform.position.y, leftFootIKPosition.z);
                    cubeLeft.transform.localPosition = new Vector3(cubeLeft.transform.localPosition.x - 0.10f, cubeLeft.transform.localPosition.y, cubeLeft.transform.localPosition.z + 0.10f);

                    // Raise an amount as a function of the grass.
                    cubeLeft.transform.Translate(Vector3.up * Time.deltaTime * amountRaiseFeetUp);
                }

            }

            // Before without the OR
            if ((leftFootDistanceToFloor >= maxHeightThreshold) || leftCubeGoingDown)
            {
                leftCubeGoingDown = true;

                if (cubeLeftCreated && leftCubeGoingDown)
                {
                    cubeLeft.transform.position = new Vector3(leftFootIKPosition.x, cubeLeft.transform.position.y, leftFootIKPosition.z);
                    cubeLeft.transform.localPosition = new Vector3(cubeLeft.transform.localPosition.x - 0.10f, cubeLeft.transform.localPosition.y, cubeLeft.transform.localPosition.z + 0.10f);
                    cubeLeft.transform.Translate(Vector3.down * Time.deltaTime * amountLowerFeetDown);

                    if (leftFootDistanceToFloor <= minHeightThreshold)
                    {
                        if(!simulateFakeGrassGround)
                        {
                            /* Simplest method - Deleting the while would just destroy the cube when maxHeightThreshold is reached */
                            leftCubeGoingDown = false;
                            cubeLeftCreated = false;
                            alreadyDestroyedLeft = true;
                            alreadyDestroyedRigth = false;
                            Destroy(cubeLeft);
                        }
                        else
                        {
                            Debug.LogError("[ERROR] Fake ground not implemented yet");
                        }
                    }
                }
            }
        }

        if (targetFoot == rightFootIKPosition && isGrass)
        {
            // If left foot is on the air...
            if (!isRightFootGrounded && isLeftFootGrounded && !cubeLeft)
            {
                // If we have not created the floating platform yet...
                if (!cubeRightCreated && !alreadyDestroyedRigth)
                {
                    cubeRight = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cubeRight.transform.SetParent(transform.parent, false);

                    // New - Does not work
                    /*
                    Collider colliderCubeRight = cubeRight.GetComponent<Collider>();
                    colliderCubeRight.material.bounciness = 1;
                    colliderCubeRight.material.bounceCombine = PhysicMaterialCombine.Multiply;
                    soleColliderRight.material.bounciness = 1;
                    */

                    MeshRenderer renderRight = cubeRight.GetComponent<MeshRenderer>();
                    if (showRaiseFeetDebug)
                    {
                        renderRight.enabled = true;
                    }
                    else
                    {
                        renderRight.enabled = false;
                    }

                    // Always orient to character's rotation.
                    cubeRight.transform.Rotate(0f, this.transform.localEulerAngles.y, 0f);

                    // Original Starting Point
                    cubeRight.transform.position = new Vector3(rightFootIKPosition.x, rightFootIKPosition.y - 0.1f, rightFootIKPosition.z);
                    cubeRight.transform.localPosition = new Vector3(cubeRight.transform.localPosition.x + 0.10f, cubeRight.transform.localPosition.y, cubeRight.transform.localPosition.z + 0.10f);
                    
                    cubeRight.transform.localScale = new Vector3(0.3f, 0.1f, 0.3f);

                    cubeRight.layer = LayerMask.NameToLayer("Environment");

                    // Set flag to cube created.
                    cubeRightCreated = true;
                }
                else if (cubeRightCreated && !alreadyDestroyedRigth)
                {
                    // If the cube has been created, we just move it in x and z based on the IK position. For Y, we raise it later.                    
                    cubeRight.transform.position = new Vector3(rightFootIKPosition.x, cubeRight.transform.position.y, rightFootIKPosition.z);
                    cubeRight.transform.localPosition = new Vector3(cubeRight.transform.localPosition.x + 0.10f, cubeRight.transform.localPosition.y, cubeRight.transform.localPosition.z + 0.10f);

                    // Raise an amount as a function of the grass.
                    cubeRight.transform.Translate(Vector3.up * Time.deltaTime * amountRaiseFeetUp);
                }
            }


            if ((rightFootDistanceToFloor >= maxHeightThreshold) || rightCubeGoingDown)
            {
                rightCubeGoingDown = true;

                if(cubeRightCreated && rightCubeGoingDown)
                {
                    cubeRight.transform.position = new Vector3(rightFootIKPosition.x, cubeRight.transform.position.y, rightFootIKPosition.z);
                    cubeRight.transform.localPosition = new Vector3(cubeRight.transform.localPosition.x + 0.10f, cubeRight.transform.localPosition.y, cubeRight.transform.localPosition.z + 0.10f);
                    cubeRight.transform.Translate(Vector3.down * Time.deltaTime * amountLowerFeetDown);

                    if (rightFootDistanceToFloor <= minHeightThreshold)
                    {
                        if (!simulateFakeGrassGround)
                        {
                            /* Simplest method - Deleting the while would just destroy the cube when maxHeightThreshold is reached */
                            rightCubeGoingDown = false;
                            cubeRightCreated = false;
                            alreadyDestroyedRigth = true;
                            alreadyDestroyedLeft = false;
                            Destroy(cubeRight);
                        }
                        else
                        {
                            Debug.LogError("[ERROR] Fake ground not implemented yet");
                        }
                    }
                }
            }
        }

        if(!isGrass)
        {
            alreadyDestroyedLeft = false;
            alreadyDestroyedRigth = false;
            cubeLeftCreated = false;
            cubeRightCreated = false;
            Destroy(cubeRight);
            Destroy(cubeLeft);
        }
    }

    #endregion
}
