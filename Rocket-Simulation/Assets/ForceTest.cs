using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForceTest : MonoBehaviour
{
    private Rigidbody r_body;
    [SerializeField] private float force;

    // Start is called before the first frame update
    void Start()
    {
        r_body = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate()
    {
        r_body.AddForceAtPosition(transform.up * force, transform.position);
    }
}
