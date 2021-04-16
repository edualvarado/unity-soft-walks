using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

[RequireComponent(typeof(CharacterController))]
public class MovementXbotNew : MonoBehaviour
{
    #region Variables

    private List<float> angles = new List<float>();

    private int countZeros, countOnes, countTwos;
    public GameObject sphereR;
    public GameObject sphereL;

    public GameObject sphere;
    private Rigidbody sphereRb;

    private Terrain terrain;
    private TerrainData terrain_data;
    private int heightmap_width;
    private int heightmap_height;
    private float[,] heightmap_data;
    private float[,] heightmap_buffer;
    private float[,] heightmap_filtered;
    private int[,] contourmap_data;
    private int[,] contourmap_buffer;

    private int count;

    private float slope_angle = 0.0f;

    private float InputX;
    private float InputZ;
    private Vector3 desiredMoveDirection;
    private bool blockRotationPlayer = false;
    [Range(0, 0.5f)] public float desiredRotationSpeed = 0.1f;

    public Animator anim;
    private float Speed;
    [Range(0, 1f)] public float allowPlayerRotation = 0.1f;

    public Camera cam;
    public CharacterController controller;
    private bool isGrounded;

    public GameObject shoesRight;
    public GameObject soleRight;
    private Collider soleColliderRight;
    public GameObject shoesLeft;
    public GameObject soleLeft;
    private Collider soleColliderLeft;

    private Vector3 prevLeftFootVector;
    private Vector3 prevRightFootVector;

    private Vector3 rightFootPosition, leftFootPosition, leftFootIKPosition, rightFootIKPosition;
    private Quaternion leftFootIKRotation, rightFootIKRotation;
    private float lastPelvisPositionY, lastRightFootPositionY, lastLeftFootPositionY;

    [Header("Ground material")]
    [Range(1, 10)]
    public int filter_coefficient = 1;
    [Range(0, 1.0f)]
    public float compression_coefficient = 0.5f;
    [Range(0, 1.0f)]
    public float depth_coefficient = 0.5f;

    [Header("Feet Grounder")]
    public bool enableFeetIK = true;
    [Range(0, 2)] [SerializeField] private float heightFromGroundRaycast = 1.14f;
    [Range(0, 2)] [SerializeField] private float raycastDownDistance = 1.5f;
    [SerializeField] private LayerMask environmentLayer;
    [SerializeField] private float pelvisOffset = 0f;
    [Range(0, 1)] [SerializeField] private float pelvisUpAndDownSpeed = 0.28f;
    [Range(0, 1)] [SerializeField] private float feetToIKPositionSpeed = 0.5f;

    public string leftFootAnimVariableName = "LeftFootCurve";
    public string rightFootAnimVariableName = "RightFootCurve";

    public bool useProIKFeature = false;
    public bool showSolverDebug = true;


    [Header("Animation Smoothing")]
    [Range(0, 1f)]
    public float HorizontalAnimSmoothTime = 0.2f;
    [Range(0, 1f)]
    public float VerticalAnimTime = 0.2f;
    [Range(0, 1f)]
    public float StartAnimTime = 0.3f;
    [Range(0, 1f)]
    public float StopAnimTime = 0.15f;

    public float raycastUpDistance = 0.275f;

    private Rigidbody rb;
    private float targetRbPosition;
    private float targetRbPosition2;
    public float alpha = 30.0f;
    public float beta = 6.0f;

    #endregion

    void Start()
    {
        System.Threading.Thread.CurrentThread.CurrentCulture = new System.Globalization.CultureInfo("en-US");

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

        //soleColliderRight = soleRight.GetComponent<Collider>();
        //soleColliderLeft = soleLeft.GetComponent<Collider>();

        anim = this.GetComponent<Animator>();
        cam = Camera.main;
        controller = this.GetComponent<CharacterController>();
        rb = this.GetComponent<Rigidbody>();

        //sphereRb = sphere.GetComponent<Rigidbody>();
        sphere.SetActive(false);

        if (anim == null)
        {
            Debug.LogError("Oops no animator");
        }

        prevLeftFootVector = anim.GetBoneTransform(HumanBodyBones.LeftToes).position - anim.GetBoneTransform(HumanBodyBones.LeftFoot).position;
        prevRightFootVector = anim.GetBoneTransform(HumanBodyBones.RightToes).position - anim.GetBoneTransform(HumanBodyBones.RightFoot).position;
    }

    void Update()
    {
        InputMagnitude();

        /* ----------- Terrain slope ----------- */

        //Normalized x and z
        float pos_x = rb.position.x / terrain_data.size.x;
        float pos_z = rb.position.z / terrain_data.size.z;

        Vector3 normal = terrain_data.GetInterpolatedNormal(pos_x, pos_z);
        float gradient = terrain_data.GetSteepness(pos_x, pos_z);

        if (normal.z < 0)
        {
            //Debug.Log("MONTEE");
            slope_angle = gradient;
        }
        else
        {
            //Debug.Log("DESCENTE");
            slope_angle = -gradient;
        }

        /* ----------- Sphere collider ---------- */

        if (Input.GetKeyDown(KeyCode.Space))
        {
            sphere.SetActive(true);
            sphereRb.MovePosition(this.transform.position + new Vector3(0, 2.0f, 0.5f));
            sphereRb.AddForce(new Vector3(0, -10.5f, 1.0f));
        }
    }

    #region PlayerMovement

    void PlayerMoveAndRotation()
    {
        InputX = Input.GetAxis("Horizontal");
        InputZ = Input.GetAxis("Vertical");

        var camera = Camera.main;
        var forward = cam.transform.forward;
        var right = cam.transform.right;

        forward.y = 0f;
        right.y = 0f;

        forward.Normalize();
        right.Normalize();

        desiredMoveDirection = forward * InputZ + right * InputX;

        if (!blockRotationPlayer)
        {
            transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(desiredMoveDirection), desiredRotationSpeed);
        }
    }

    void InputMagnitude()
    {
        InputX = Input.GetAxis("Horizontal");
        InputZ = Input.GetAxis("Vertical");

        anim.SetFloat("InputX", InputX, HorizontalAnimSmoothTime, Time.deltaTime * 2f);
        anim.SetFloat("InputZ", InputZ, VerticalAnimTime, Time.deltaTime * 2f);

        Speed = new Vector2(InputX, InputZ).sqrMagnitude;

        if (Speed > allowPlayerRotation)
        {
            anim.SetFloat("InputMagnitude", Speed, StartAnimTime, Time.deltaTime);
            PlayerMoveAndRotation();
        }
        else if (Speed < allowPlayerRotation)
        {
            anim.SetFloat("InputMagnitude", Speed, StopAnimTime, Time.deltaTime);
        }

    }

    #endregion

    private void FixedUpdate()
    {
        if (!enableFeetIK) { return; }
        if (anim == null) { return; }

        //Change foot position to one above the ground so that ray casting works
        AdjustFeetTarget(ref rightFootPosition, HumanBodyBones.RightFoot);
        AdjustFeetTarget(ref leftFootPosition, HumanBodyBones.LeftFoot);

        //Raycast to find positions
        FeetPositionSolver(rightFootPosition, ref rightFootIKPosition, ref rightFootIKRotation);
        FeetPositionSolver(leftFootPosition, ref leftFootIKPosition, ref leftFootIKRotation);

    }

    private void OnAnimatorIK(int layerIndex)
    {
        if (!enableFeetIK) { return; }
        if (anim == null) { return; }

        MovePelvisHeight();

        //Right foot ik position and rotation
        anim.SetIKPositionWeight(AvatarIKGoal.RightFoot, 1);

        if (useProIKFeature)
        {
            anim.SetIKRotationWeight(AvatarIKGoal.RightFoot, anim.GetFloat(rightFootAnimVariableName));
        }

        MoveFeetToIKPoint(AvatarIKGoal.RightFoot, rightFootIKPosition, rightFootIKRotation, ref lastRightFootPositionY);

        // Left foot ik position and rotation
        anim.SetIKPositionWeight(AvatarIKGoal.LeftFoot, 1);

        if (useProIKFeature)
        {
            anim.SetIKRotationWeight(AvatarIKGoal.LeftFoot, anim.GetFloat(leftFootAnimVariableName));
        }

        MoveFeetToIKPoint(AvatarIKGoal.LeftFoot, leftFootIKPosition, leftFootIKRotation, ref lastLeftFootPositionY);

        /* ----------- Adjust orientation thanks to the slope ----------- */

        if (Vector3.Dot(rightFootPosition, transform.forward) <= Vector3.Dot(leftFootPosition, transform.forward)) //Left foot in front of right foot
        {
            targetRbPosition = Vector3.Dot(rightFootPosition, transform.forward) +
                ((leftFootPosition - rightFootPosition).magnitude / 2.0f) * Mathf.Cos(slope_angle * 3.1415f / 180.0f);
        }
        else //Right foot in front of left foot
        {
            targetRbPosition = Vector3.Dot(leftFootPosition, transform.forward)
                + ((rightFootPosition - leftFootPosition).magnitude / 2.0f) * Mathf.Cos(slope_angle * 3.1415f / 180.0f);
        }

        float tau = alpha * (Vector3.Dot(anim.bodyPosition, transform.forward) - targetRbPosition)
            + beta * (Vector3.Dot(rb.angularVelocity, transform.right) - 0);

        rb.AddTorque(-transform.right * tau);

        targetRbPosition2 = Vector3.Dot(leftFootPosition, transform.right)
            + ((Vector3.Dot(rightFootPosition, transform.right) - Vector3.Dot(leftFootPosition, transform.right)) / 2.0f);

        float tau2 = 0.5f * (Vector3.Dot(anim.bodyPosition, transform.right) - targetRbPosition2)
            + 0.01f * (Vector3.Dot(rb.angularVelocity, transform.forward) - 0);

        rb.AddTorque(transform.forward * tau2);

        float theta = 90.0f
            - Mathf.Acos(Vector3.Dot(rb.transform.up, new Vector3(0, Mathf.Sin(slope_angle * 3.1415f / 180.0f), Mathf.Cos(slope_angle * 3.1415f / 180.0f)))) * 180.0f / Mathf.PI;
        angles.Add(theta);

        //Shoes
        MoveShoes();

        count = 0;
        //Modify contourmap with raycasting
        SetContourMap(leftFootIKPosition);
        SetContourMap(rightFootIKPosition);

        //Set contours of the foot prints
        SetOnes(leftFootIKPosition);
        SetOnes(rightFootIKPosition);

        //Holes under the feet
        SetHeightMap(leftFootIKPosition);
        SetHeightMap(rightFootIKPosition);

        //Blur the contours
        FilterHeightMap(heightmap_buffer);
        //Iterate
        for (int i = 1; i < filter_coefficient; i++)
        {
            FilterHeightMap(heightmap_filtered);
        }

        //Save changes
        terrain_data.SetHeights(0, 0, heightmap_filtered);

    }

    #region FeetGroundingMethods

    void MoveFeetToIKPoint(AvatarIKGoal foot, Vector3 positionIKHolder, Quaternion rotationIKHolder, ref float lastFootPositionY)
    {
        Vector3 targetIKPosition = anim.GetIKPosition(foot);

        if (positionIKHolder != Vector3.zero)
        {
            targetIKPosition = transform.InverseTransformPoint(targetIKPosition);
            positionIKHolder = transform.InverseTransformPoint(positionIKHolder);

            float yVariable = Mathf.Lerp(lastFootPositionY, positionIKHolder.y, feetToIKPositionSpeed);
            targetIKPosition.y += yVariable;

            lastFootPositionY = yVariable;

            targetIKPosition = transform.TransformPoint(targetIKPosition);

            anim.SetIKRotation(foot, rotationIKHolder);
        }

        anim.SetIKPosition(foot, targetIKPosition);
    }

    private void MovePelvisHeight()
    {
        if (rightFootIKPosition == Vector3.zero || leftFootIKPosition == Vector3.zero || lastPelvisPositionY == 0)
        {
            lastPelvisPositionY = anim.bodyPosition.y;
            return;
        }

        float lOffsetPosition = leftFootIKPosition.y - transform.position.y;
        float rOffsetPosition = rightFootIKPosition.y - transform.position.y;

        float totalOffset = (lOffsetPosition < rOffsetPosition) ? lOffsetPosition : rOffsetPosition;

        Vector3 newPelvisPosition = anim.bodyPosition + Vector3.up * totalOffset;
        newPelvisPosition.y = Mathf.Lerp(lastPelvisPositionY, newPelvisPosition.y, pelvisUpAndDownSpeed);

        anim.bodyPosition = newPelvisPosition;
        lastPelvisPositionY = anim.bodyPosition.y;
    }

    private void FeetPositionSolver(Vector3 fromSkyPosition, ref Vector3 feetIKPositions, ref Quaternion feetIKRotations)
    {
        RaycastHit feetOutHit;

        if (showSolverDebug)
            Debug.DrawLine(fromSkyPosition, fromSkyPosition + Vector3.down * (raycastDownDistance + heightFromGroundRaycast), Color.yellow);

        if (Physics.Raycast(fromSkyPosition, Vector3.down, out feetOutHit, raycastDownDistance + heightFromGroundRaycast, environmentLayer))
        {
            if ((fromSkyPosition == rightFootPosition && anim.GetFloat(rightFootAnimVariableName) == 1.0f)
                || (fromSkyPosition == leftFootPosition && anim.GetFloat(leftFootAnimVariableName) > 0.9f))
            {
                feetIKPositions = fromSkyPosition;
                feetIKPositions.y = feetOutHit.point.y - 0.005f + pelvisOffset;
                feetIKRotations = Quaternion.FromToRotation(Vector3.up, feetOutHit.normal) * transform.rotation;
                return;
            }
            else
            {
                feetIKPositions = fromSkyPosition;
                feetIKPositions.y = feetOutHit.point.y + 0.01f + pelvisOffset;
                feetIKRotations = Quaternion.FromToRotation(Vector3.up, feetOutHit.normal) * transform.rotation;
                return;
            }

        }

        feetIKPositions = Vector3.zero;
    }

    private void AdjustFeetTarget(ref Vector3 feetPositions, HumanBodyBones foot)
    {
        feetPositions = anim.GetBoneTransform(foot).position;
        feetPositions.y = transform.position.y + heightFromGroundRaycast;
    }

    #endregion

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

    #region Modify heightmap methods
    private void SetContourMap(Vector3 targetFoot)
    {
        //Indices of foot transform in grid
        int indX = (int)((targetFoot.x / terrain_data.size.x) * heightmap_width);
        int indZ = (int)((targetFoot.z / terrain_data.size.z) * heightmap_height);

        //Out of bounds
        if ((indX <= 20) || (indZ <= 20) || (indX > heightmap_width - 20) || (indZ > heightmap_height - 20))
        {
            return;
        }
        for (int i = -20; i < 20; i++)
        {
            for (int j = -20; j < 20; j++)
            {
                if (targetFoot.y < heightmap_data[indZ + j, indX + i] * terrain_data.heightmapScale.y)
                {
                    //World Space Coordinates of grid cells
                    Vector3 columnPos = new Vector3((indX + i) * terrain_data.size.x / heightmap_width, targetFoot.y, (indZ + j) * terrain_data.size.z / heightmap_height);

                    Ray ray = new Ray();
                    ray.origin = columnPos;
                    ray.direction = Vector3.up;
                    RaycastHit outHit;

                    Debug.DrawRay(columnPos, Vector3.up);

                    if (targetFoot == rightFootIKPosition && soleColliderRight.Raycast(ray, out outHit, 2.0f))
                    {
                        contourmap_data[indZ + j, indX + i] = 2;
                        contourmap_buffer[indZ + j, indX + i] = 2;
                        count += 1;
                    }
                    else if (targetFoot == leftFootIKPosition && soleColliderLeft.Raycast(ray, out outHit, 2.0f))
                    {
                        contourmap_data[indZ + j, indX + i] = 2;
                        contourmap_buffer[indZ + j, indX + i] = 2;
                        count += 1;
                    }
                }
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
        // terrain_data.SetHeights(0, 0, heightmap_buffer);
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

    private void OnApplicationQuit()
    {
        List<List<int>> h = new List<List<int>>();
        for (int i = heightmap_height - 1; i >= 0; i--)
        {
            List<int> list = new List<int>();
            for (int j = 0; j < heightmap_width; j++)
            {
                /* switch (contourmap_data[i, j])
                 {
                     case (0):
                         countZeros += 1;
                         break;
                     case (1):
                         countOnes += 1;
                         break;
                     case (2):
                         countTwos += 1;
                         break;
                 }*/

                list.Add(contourmap_data[i, j]);
            }
            h.Add(list);
        }
        using (TextWriter tw = new StreamWriter("contour.txt"))
        {
            tw.WriteLine(h.Count);
            foreach (List<int> l in h)
            {
                foreach (int p in l)
                    tw.WriteLine(p);
            }
        }

        /* Debug.Log(countZeros);
         Debug.Log(countOnes);
         Debug.Log(countTwos);*/

        using (TextWriter tw = new StreamWriter("AnglesXbot.txt"))
        {
            tw.WriteLine(angles.Count);
            foreach (float p in angles)
                tw.WriteLine(p);
        }
    }
}