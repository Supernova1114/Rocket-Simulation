using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class UIManager : MonoBehaviour
{
    [SerializeField] private Slider y_velocity_slider;
    [SerializeField] private TextMeshProUGUI y_velocity_text;
    [Space]
    [SerializeField] private Slider mass_slider;
    [SerializeField] private TextMeshProUGUI mass_text;
    [Space]
    [SerializeField] private Slider max_thrust_slider;
    [SerializeField] private TextMeshProUGUI max_thrust_text;
    [Space]
    [SerializeField] private Toggle manual_control_toggle;
    [Space]
    [SerializeField] private TextMeshProUGUI telemetry_text;

    private static UIManager instance;

    private void Awake()
    {
        instance = this;
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        y_velocity_text.text = "Y-Velocity: " + y_velocity_slider.value + " m/s";
        mass_text.text = "Mass: " + mass_slider.value + " kg";
        //max_thrust_text.text = "Max Thrust: " + max_thrust_slider.value + " N";
    }

    public static UIManager GetInstance()
    {
        return instance;
    }

    public float get_y_velocity_value()
    {
        return y_velocity_slider.value;
    }

    public float get_mass_value()
    { 
        return mass_slider.value; 
    }

    public float get_max_thrust_value()
    {
        return max_thrust_slider.value;
    }

    public void UpdateTelemetryData(RocketTelemetry rocket_telemetry)
    {
        telemetry_text.text = "Y-Velocity: " + rocket_telemetry.currentVelocity.y + " m/s\n"
                            + "Z-Velocity: " + rocket_telemetry.currentVelocity.z + "m/s\n"
                            + "Current Thrust: " + rocket_telemetry.currentThrustForce + " N";
    }

    public bool get_manual_control_bool()
    {
        return manual_control_toggle.isOn;
    }

}
