using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotationTester : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private PIDController pid_controller;
    [Space]
    [SerializeField] private float power;

    private Rigidbody r_body;

    private void Start()
    {
        r_body = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        Vector3 targetDir = (target.position - transform.position).normalized;
        targetDir.y = transform.position.y; // Ignore y

        Vector3 currentDir = transform.forward;

        float targetAngle = Mathf.Atan2(targetDir.z, targetDir.x);
        float currentAngle = Mathf.Atan2(currentDir.z, currentDir.x);

        float pid_result = pid_controller.UpdateAngle(Time.fixedDeltaTime, currentAngle, targetAngle);
        r_body.AddTorque(new Vector3(0, pid_result * power, 0));
    }

}
