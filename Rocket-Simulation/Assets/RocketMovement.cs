using MichaelWolfGames;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class RocketMovement : MonoBehaviour
{
    [SerializeField] private bool rotateTowardsTargetObj = false;
    [SerializeField] private bool enableManualControl = false;
    [SerializeField] private float targetVelocityDown;
    [SerializeField] private GameObject imuObj;

    [SerializeField] private GameObject engineGimbalPitch;
    [SerializeField] private GameObject engineGimbalYaw;

    [SerializeField] private GameObject engine;

    [SerializeField] private float engineStartDelay;
    [SerializeField] private float thrustForce;
    [SerializeField] private Transform target;

    [SerializeField] private PIDController pid_controller_pitch;
    [SerializeField] private PIDController pid_controller_yaw;
    [SerializeField] private PIDController pid_controller_velocity;


    private Rigidbody r_body;
    private bool enable_engine = false;

    private float horizontalInput;
    private float verticalInput;

    private Vector3 currentRot;
    private Vector3 targetDir;

    private static UIManager ui_manager;

    private RocketTelemetry telemetry = new RocketTelemetry();

    void Start()
    {
        ui_manager = UIManager.GetInstance();

        r_body = GetComponent<Rigidbody>();
        r_body.sleepThreshold = 0.001f;

        currentRot = imuObj.transform.up;

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

    private void Update()
    {
        horizontalInput = Input.GetAxisRaw("Horizontal");
        verticalInput = Input.GetAxisRaw("Vertical");

        targetVelocityDown = ui_manager.get_y_velocity_value();
        r_body.mass = ui_manager.get_mass_value();
        //thrustForce = ui_manager.get_max_thrust_value();
        enableManualControl = ui_manager.get_manual_control_bool();
    }

    

    private void FixedUpdate()
    {   
        if (enable_engine)
        {
            
            if (rotateTowardsTargetObj)
            {
                targetDir = (target.position - imuObj.transform.position).normalized;
            }
            else
            {
                targetDir = Vector3.up; // Temp
            }


            // Enable manual control 

            if (enableManualControl == true)
            {
                Vector3 movementInput = new Vector3(-1 * horizontalInput, 0, -1 * verticalInput).normalized * 0.03f;

                if (movementInput.sqrMagnitude > 0)
                {
                    currentRot += imuObj.transform.rotation * movementInput;
                }

                targetDir = currentRot;
            }
            

            Debug.DrawRay(imuObj.transform.position, targetDir, Color.red);

            Vector3 currentDir = imuObj.transform.up;
            Vector3 currentPos = imuObj.transform.position;

            Quaternion targetGimbalRot = Quaternion.AngleAxis(180, currentDir) * Quaternion.LookRotation(targetDir);
            Vector3 targetGimbalDir = targetGimbalRot * Vector3.forward;

            if (Vector3.Angle(currentDir, targetGimbalDir) >= 90)
            {
                targetGimbalDir = Vector3.ProjectOnPlane(targetGimbalDir, currentDir).normalized;
                targetGimbalRot = Quaternion.LookRotation(targetGimbalDir);
            }

            //Debug.DrawRay(engineGimbalPitch.transform.position, targetGimbalDir, Color.blue);

            // project onto plane with current dir normal
            Vector3 projVec = Vector3.ProjectOnPlane(targetGimbalDir, currentDir);
            //Debug.DrawRay(engineGimbalPitch.transform.position, projVec, Color.red);

            float projAngle = Vector3.SignedAngle(imuObj.transform.right, projVec, currentDir);

            //float projAngleYaw = Vector3.SignedAngle(currentDir, targetDir, imuObj.transform.forward);
            //float projAnglePitch = Vector3.SignedAngle(currentDir, targetDir, imuObj.transform.right);

            //newwwwwww

            Vector3 targetPitchDir = Vector3.ProjectOnPlane(targetDir, imuObj.transform.right);
            Vector3 targetYawDir = Vector3.ProjectOnPlane(targetDir, imuObj.transform.forward);

            /*float pitchAngleDelta = Vector3.SignedAngle(currentDir, targetPitchDir, imuObj.transform.right);
            float yawAngleDelta = Vector3.SignedAngle(currentDir, targetYawDir, imuObj.transform.forward);*/

            float pitchAngleDelta = Vector3.Angle(currentDir, targetPitchDir);
            float yawAngleDelta = Vector3.Angle(currentDir, targetYawDir);

            //Debug.DrawRay(currentPos, currentDir * 2, Color.blue);
            //Debug.DrawRay(currentPos, targetPitchDir * 2, Color.red);

            float pid_result_pitch = pid_controller_pitch.UpdateAngle(Time.fixedDeltaTime, pitchAngleDelta, 0);
            float pid_result_yaw = pid_controller_yaw.UpdateAngle(Time.fixedDeltaTime, yawAngleDelta, 0);

            //float projVecMagnitude = projVec.magnitude;

            float y = Mathf.Sin(projAngle * Mathf.Deg2Rad);
            float x = Mathf.Cos(projAngle * Mathf.Deg2Rad);

            /*float currentPitchAngle = engineGimbalPitch.transform.localEulerAngles.x;
            float targetPitchAngle = Mathf.Asin(y) * Mathf.Rad2Deg;

            float currentYawAngle = engineGimbalYaw.transform.localEulerAngles.y;
            float targetYawAngle = Mathf.Asin(x) * Mathf.Rad2Deg;*/

            // Get angles from 90 to -90
            float yawAngle = Mathf.Asin(x * -1 * pid_result_yaw) * Mathf.Rad2Deg;
            float pitchAngle = -1 * Mathf.Asin(y * -1 * pid_result_pitch) * Mathf.Rad2Deg;

            //print(y * -1 * pid_result_pitch);
            engineGimbalPitch.transform.localRotation = Quaternion.Euler(pitchAngle, 0, 0);
            engineGimbalYaw.transform.localRotation = Quaternion.Euler(0, yawAngle, 0);

            //print("p: " + pitchAngle + " | y: " + yawAngle);

            //float target_gimbal_angle = Mathf.Asin(tempTest) * Mathf.Rad2Deg;

            float targetVelocity = targetVelocityDown;

            float pid_result_velocity = pid_controller_velocity.Update(Time.fixedDeltaTime, r_body.velocity.y, targetVelocity);

            /* if (Vector3.Angle(currentDir,targetDir) > 10)
             {
                 pid_result_velocity = 1;
             }*/


            //print("pid_result: " + pid_result_velocity + " | Vel_Y: " + r_body.velocity.y + " | target_vel: " + targetVelocity);

            float currentThrustForce = thrustForce * pid_result_velocity;

            if (Mathf.Abs(currentThrustForce) > 0)
            {

                // TODO - Take into account both pitch AND yaw
                //currentThrustForce /= Mathf.Cos(Mathf.Deg2Rad * convertAngleRange(imuObj.transform.eulerAngles.z));
                float rocketAngle = Vector3.Angle(Vector3.up, imuObj.transform.up);

                if (rocketAngle < 75)
                {
                    currentThrustForce += r_body.mass * Mathf.Abs(Physics.gravity.y); // Take into account gravity.

                    print(Mathf.Cos(Mathf.Deg2Rad * rocketAngle));
                    currentThrustForce /= Mathf.Cos(Mathf.Deg2Rad * rocketAngle);
                }
            }
            
            //currentThrustForce *= Mathf.Cos

            // limit force output
            if (currentThrustForce > thrustForce)
            {
                currentThrustForce = thrustForce;
            }

            // Engine force
            r_body.AddForceAtPosition(engine.transform.up * currentThrustForce, engine.transform.position);
            //Debug.DrawRay(engine.transform.position, engine.transform.up * -1 * 2, Color.red);

            // Send telemetry to UI
            telemetry.currentThrustForce = currentThrustForce;
        }

        telemetry.currentVelocity = r_body.velocity;
        ui_manager.UpdateTelemetryData(telemetry);
    }


    private float convertAngleRange(float angle)
    {
        return angle > 180 ? angle - 360 : angle;
    }
}
