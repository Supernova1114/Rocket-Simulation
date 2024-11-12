using MichaelWolfGames;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class RocketMovement : MonoBehaviour
{
    [SerializeField] private GameObject imuObj;
    [SerializeField] private GameObject engineGimbal;
    [SerializeField] private GameObject engine;

    [SerializeField] private float thrustForce;
    [SerializeField] private Transform target;

    [SerializeField] private PIDController pid_controller;

    private Rigidbody r_body;
    private bool enable_engine = false;
    
    void Start()
    {

        r_body = GetComponent<Rigidbody>();
        r_body.sleepThreshold = 0.001f;

        this.StartTimer(3, () => { enable_engine = true; engine.SetActive(true); });
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.transform.CompareTag("ground_plane"))
        {
            enable_engine = false;
            engine.SetActive(false);
        }
    }

    private void FixedUpdate()
    {   
        if (enable_engine)
        {
            Vector3 targetDir = (target.position - imuObj.transform.position).normalized;
            targetDir = Vector3.up; // Temp

            Vector3 currentDir = imuObj.transform.up;

            float angleDelta = Vector3.Angle(targetDir, currentDir);

            float pid_result = pid_controller.UpdateAngle(Time.fixedDeltaTime, angleDelta, 0);

            Vector3 rotationAxis = Vector3.Cross(targetDir, currentDir);

            Quaternion rotation_towards_target = Quaternion.LookRotation(targetDir);

            Quaternion engine_gimal_rot_soln = Quaternion.AngleAxis(pid_result, rotationAxis);

            engineGimbal.transform.rotation = Quaternion.AngleAxis(180, transform.up) * engine_gimal_rot_soln * rotation_towards_target;

            // Engine force
            r_body.AddForceAtPosition(engine.transform.up * thrustForce, engine.transform.position);
            Debug.DrawRay(engine.transform.position, engine.transform.up * -1 * 2, Color.red);
        }
    }
}
