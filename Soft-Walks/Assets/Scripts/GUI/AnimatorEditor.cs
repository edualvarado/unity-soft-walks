using UnityEditor;
using UnityEngine;
[CustomEditor(typeof(Animator))]
public class AnimatorEditor : Editor
{
	/*
	void OnSceneGUI()
	{
		Animator anim = target as Animator;
		Handles.color = Color.blue;
		Handles.SphereCap(1, anim.bodyPosition, Quaternion.Euler(0f,0f,0f), 0.1f);
	}
	public override void OnInspectorGUI()
	{
		GUI.skin = EditorGUIUtility.GetBuiltinSkin(UnityEditor.EditorSkin.Inspector);
		DrawDefaultInspector();
	}
	*/
}