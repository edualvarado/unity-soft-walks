using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class interactiveGrass : MonoBehaviour
{
    public Material[] materials;
    public Transform character;
    Vector3 position_ground, position_alt;
    PlayerControllerIKFeetPlacementTorqueTerrainVeg setterIsGrass;

    // Start is called before the first frame update
    void Start()
    {
        //setterIsGrass = character.GetComponent<PlayerControllerIKFeetPlacementTorqueTerrainVeg>();
        StartCoroutine("writeToMaterial");
    }

    IEnumerator writeToMaterial()
    {
        while(true)
        {
            /*
            position_ground = character.position;
            position_alt = character.position + Vector3.up * 0.5f;

            for (int i = 0; i < materials.Length; i++)
            {
                materials[i].SetVector("_position", position_alt);
            }
            */

            yield return null;
        }
    }
}
