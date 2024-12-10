using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestRotator : MonoBehaviour
{
    

    private float horizontalInput;
    private float verticalInput;

    // Start is called before the first frame update
    void Start()
    {
    }

    private void Update()
    {
        horizontalInput = Input.GetAxisRaw("Horizontal");
        verticalInput = Input.GetAxisRaw("Vertical");

        Vector3 movementInput = new Vector3(horizontalInput, 0, verticalInput).normalized;

        Quaternion quaternion = Quaternion.Euler(movementInput.z, 0, movementInput.x);

        //transform.rotation *= Quaternion.Inverse(transform.rotation) * quaternion * transform.rotation;

        transform.localRotation *= quaternion;
    }


}
