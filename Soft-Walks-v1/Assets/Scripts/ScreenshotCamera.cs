using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScreenshotCamera : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Take a screenshot
        if (Input.GetKeyDown(KeyCode.Space))
        {
            Debug.Log("Screenshot taken!");
            ScreenCapture.CaptureScreenshot("screenshot-1", 2);
        }
    }
}
