using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RocketMovement : MonoBehaviour
{
    [SerializeField] private GameObject imu_obj;
    [SerializeField] private GameObject engine_rotator;
    [SerializeField] private GameObject engine;
    [SerializeField] private float engine_force;
    [SerializeField] private float adjustmentFactor;
    [SerializeField] private Transform target;

    private Rigidbody r_body;
    private bool enable_engine = true;

    // Start is called before the first frame update
    void Start()
    {
        r_body = GetComponent<Rigidbody>();
        r_body.sleepThreshold = 0.001f;
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
            Quaternion rotation_towards_target = Quaternion.LookRotation(target.position - imu_obj.transform.position);
            rotation_towards_target = Quaternion.LookRotation(Vector3.up);

            engine_rotator.transform.rotation = Quaternion.AngleAxis(180, transform.up) * rotation_towards_target;

            // Engine force
            r_body.AddForceAtPosition(engine.transform.up * engine_force, engine.transform.position);
            Debug.DrawRay(engine.transform.position, engine.transform.up * -1 * 2, Color.red);
        }
    }
}
