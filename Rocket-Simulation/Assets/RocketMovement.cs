using MichaelWolfGames;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class RocketMovement : MonoBehaviour
{
    [SerializeField] private GameObject imuObj;

    [SerializeField] private GameObject engineGimbalPitch;
    [SerializeField] private GameObject engineGimbalYaw;

    [SerializeField] private GameObject engine;

    [SerializeField] private float engineStartDelay;
    [SerializeField] private float thrustForce;
    [SerializeField] private Transform target;

    [SerializeField] private PIDController pid_controller_pitch;

    private Rigidbody r_body;
    private bool enable_engine = false;

    void Start()
    {

        r_body = GetComponent<Rigidbody>();
        r_body.sleepThreshold = 0.001f;

        this.StartTimer(engineStartDelay, () => { enable_engine = true; engine.SetActive(true); });
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
            Vector3 currentPos = imuObj.transform.position;
       
            Debug.DrawRay(currentPos, currentDir, Color.blue);
            Debug.DrawRay(currentPos, targetDir, Color.red);

            Quaternion targetGimbalRot = Quaternion.AngleAxis(180, currentDir) * Quaternion.LookRotation(targetDir);
            Vector3 targetGimbalDir = targetGimbalRot * Vector3.forward;

            if (Vector3.Angle(currentDir, targetGimbalDir) >= 90)
            {
                targetGimbalDir = Vector3.ProjectOnPlane(targetGimbalDir, currentDir).normalized;
                targetGimbalRot = Quaternion.LookRotation(targetGimbalDir);
            }

            Debug.DrawRay(engineGimbalPitch.transform.position, targetGimbalDir, Color.blue);

            // project onto plane with current dir normal
            Vector3 projVec = Vector3.ProjectOnPlane(targetGimbalDir, currentDir);
            Debug.DrawRay(engineGimbalPitch.transform.position, projVec, Color.red);

            float projAngle = Vector3.SignedAngle(imuObj.transform.right, projVec, currentDir);

            float projVecMagnitude = projVec.magnitude;

            float y = projVecMagnitude * Mathf.Sin(projAngle * Mathf.Deg2Rad);
            float x = projVecMagnitude * Mathf.Cos(projAngle * Mathf.Deg2Rad);

            float angleDelta = Vector3.Angle(targetDir, currentDir);
            float pid_result = pid_controller_pitch.UpdateAngle(Time.fixedDeltaTime, angleDelta, 0);

            // Get angles from 90 to -90
            float yawAngle = Mathf.Asin(x * -1 * pid_result) * Mathf.Rad2Deg;
            float pitchAngle = Mathf.Asin(y * -1 * pid_result) * Mathf.Rad2Deg;

            engineGimbalPitch.transform.localRotation = Quaternion.Euler(-pitchAngle, 0, 0);
            engineGimbalYaw.transform.localRotation = Quaternion.Euler(0, yawAngle, 0);

            //print("p: " + pitchAngle + " | y: " + yawAngle);

            //float target_gimbal_angle = Mathf.Asin(tempTest) * Mathf.Rad2Deg;



            // Engine force
            r_body.AddForceAtPosition(engine.transform.up * thrustForce, engine.transform.position);
            //Debug.DrawRay(engine.transform.position, engine.transform.up * -1 * 2, Color.red);
        }
    }
}
