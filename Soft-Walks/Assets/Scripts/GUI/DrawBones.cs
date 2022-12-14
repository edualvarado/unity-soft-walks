using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class DrawBones : MonoBehaviour
{
    private SkinnedMeshRenderer m_Renderer;

    void Start()
    {
        m_Renderer = GetComponentInChildren<SkinnedMeshRenderer>();
        if (m_Renderer == null)
        {
            Debug.LogWarning("No SkinnedMeshRenderer found, script removed");
            Destroy(this);
        }
    }

    void LateUpdate()
    {
        var bones = m_Renderer.bones;
        foreach (var B in bones)
        {
            if (B.parent == null)
                continue;
            Debug.DrawLine(B.position, B.parent.position, Color.green);
        }
    }
}
