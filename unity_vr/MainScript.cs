/*

Robot controller
----------------

https://github.com/turingbirds/robot-driver

Copyright 2023
Apache License 2.0

*/


// 'throw' using pure acceleration
#define USE_ACCELERATION_RIGHT

// use absolute position?
// #define USE_ABSOLUTE

// use trigger to 'grasp and move'
#define USE_TRIGGER_LEFT

// ---
// when encoder is on motor, tilt HMD up/down to track movement up/down
//#define HMD_CONTROLS_UP_DOWN

// use joystick for up/down
#define JOYSTICK_UP_DOWN


using System;
using System.IO;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR.Interaction;
using UnityEngine.SceneManagement;


public class MainScript : MonoBehaviour
{
    private float global_t = 0f;

    private bool triggerPressedR = false;
    private bool triggerPressedL = false;
    private Vector3 startStartingR;
    private Vector3 startStartingL;
    private Vector3 startingR;
    private Vector3 startingL;

    private const float hmd_pitch_sensitivity_multiplier = 1f;
    private const float hmd_yaw_sensitivity_multiplier = 1f;
    private const float controllers_sensitivity_multiplier = 5f;
    private const float accel_sensitivity_multiplier = 50f;

    private const float hmd_sensitivity_multiplier_joystick = 90f;

    private const int min_yaw = 60;
    private const int max_yaw = 140;

    private const int min_pitch = 40;
    private const int max_pitch = 100;

    private bool joystick_has_moved = false;
    private bool r_limb_has_moved = false;
    private bool l_limb_has_moved = false;

    private double pitchAngle_zero = Double.NaN;
    private double yawAngle_zero = Double.NaN;

    public string aux_pos_1 = "0";
    public string aux_pos_2 = "0";
    public string vel_L = "0";
    public string vel_C = "0";
    public string vel_R = "0";

#if USE_ACCELERATION_LEFT
    private int last_pos_left;
#endif
#if USE_ACCELERATION_RIGHT
    private int scalar_accel_R = 0;
#endif

    /**
     * Network comms
    **/

    public UdpClient client;
    public IPEndPoint remoteEndPoint;
    public string udp_ip = "192.168.1.125";
    public int udp_port = 1234;

    const unsigned int NETWORK_SEND_EVERY_N_TH_FRAME = 2;   // send every N-th frame. You might want to limit fps in addition to tweaking this value!

    void Start()
    {
        Debug.Log("Start()");

        i = 0;
#if USE_ACCELERATION_LEFT
        last_pos_left = 0;
#endif
#if USE_ACCELERATION_RIGHT
        scalar_accel_R = 0;
#endif
        /**
         * init network comms
        **/      
        
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(udp_ip), udp_port);
        client = new UdpClient();
        client.Connect(remoteEndPoint);

        /**
         * init tracking space type
        **/

        var headMountedControllers = new List<UnityEngine.XR.InputDevice>();
        var desiredCharacteristicsHeadMounted = UnityEngine.XR.InputDeviceCharacteristics.HeadMounted;
        UnityEngine.XR.InputDevices.GetDevicesWithCharacteristics(desiredCharacteristicsHeadMounted, headMountedControllers);
        UnityEngine.XR.XRDevice.SetTrackingSpaceType(UnityEngine.XR.TrackingSpaceType.Stationary);
        //XRDevice.SetTrackingSpaceType(TrackingSpaceType.Stationary);
        //InputTracking.Recenter();
    }

    // Update is called once per frame
    void Update()
    {
        global_t += Time.deltaTime;

        const float trigger_drag_multiplier = 15000f;
        bool triggerValue;

        var leftHandedControllers = new List<UnityEngine.XR.InputDevice>();
        var rightHandedControllers = new List<UnityEngine.XR.InputDevice>();
        var headMountedControllers = new List<UnityEngine.XR.InputDevice>();
        //var desiredCharacteristics = UnityEngine.XR.InputDeviceCharacteristics.HeldInHand | UnityEngine.XR.InputDeviceCharacteristics.Left | UnityEngine.XR.InputDeviceCharacteristics.Controller;
        var desiredCharacteristicsLeft = UnityEngine.XR.InputDeviceCharacteristics.HeldInHand | UnityEngine.XR.InputDeviceCharacteristics.Left | UnityEngine.XR.InputDeviceCharacteristics.Controller;
        var desiredCharacteristicsRight = UnityEngine.XR.InputDeviceCharacteristics.HeldInHand | UnityEngine.XR.InputDeviceCharacteristics.Right | UnityEngine.XR.InputDeviceCharacteristics.Controller;
        var desiredCharacteristicsHeadMounted = UnityEngine.XR.InputDeviceCharacteristics.HeadMounted;
        UnityEngine.XR.InputDevices.GetDevicesWithCharacteristics(desiredCharacteristicsLeft, leftHandedControllers);
        UnityEngine.XR.InputDevices.GetDevicesWithCharacteristics(desiredCharacteristicsRight, rightHandedControllers);
        UnityEngine.XR.InputDevices.GetDevicesWithCharacteristics(desiredCharacteristicsHeadMounted, headMountedControllers);


#if JOYSTICK_UP_DOWN
        // joystick controls center motor

        Vector2 joystickVec;
        rightHandedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxis, out joystickVec);
        //Debug.Log("jOYSTICK: " + joystickVec);
        if (joystickVec[1] > .1 || joystickVec[1] < -.1)
        {

            float value = hmd_sensitivity_multiplier_joystick * joystickVec[1];
            // clip values 
            value = System.Math.Min(value, 210f);
            value = System.Math.Max(value, -210f);
            value = System.Math.Abs(value);

            int pos = (int)(value);

            if (!joystick_has_moved && System.Math.Abs(pos) > 10)
            {
                // hide joystick help message
                textFader1.state = 2;
                joystick_has_moved = true;
                //GetComponent<TextFader1>().state = 0;
                //textFader1.global_t = 0f;
            }

            vel_C = System.Convert.ToString(pos);
        }
        else
        {
            vel_C = "0";
        }
#endif


#if USE_TRIGGER_LEFT
        // Left trigger + movement control left motor

        if (leftHandedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.triggerButton, out triggerValue) && triggerValue)
        {
            Debug.Log("left trigger button is pressed.");

            // store controller position upon trigger press
            if (!triggerPressedL)
            {
                Vector3 position;
                if (leftHandedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out position))
                {
                    startStartingL = position;
                    startingL = position;
                    Debug.Log("Starting position left: " + startingL);
                    triggerPressedL = true;
                }
            }
            else
            {
                // trigger was pressed and we moved -- send delta position
                Vector3 hmdPosition;
                headMountedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out hmdPosition);

                Vector3 position;
                if (leftHandedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out position))
                {
                    Vector3 delta_pos = startingL - position;
                    float distance = delta_pos.magnitude;


                    float distanceTrigPressedFromHmd = (startStartingL - hmdPosition).magnitude;
                    float distanceFromHmd = (position - hmdPosition).magnitude;

                    /*
                    // grab direction -- angle from left-right movement
                    Vector3 direction = (position - startStartingR).normalized;
                    Debug.LogWarning("direction = ");
                    Debug.LogWarning(direction);
                    float deg = Vector3.SignedAngle(position, startStartingR, Vector3.up);
                    string strValue1 = System.Convert.ToString(deg);
                    client.Publish(MQTT_PREFIX + "yrdy", System.Text.Encoding.UTF8.GetBytes(strValue1), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, true);
                    */

                    // grab direction -- based on distance from/to headset
                    //float origDist = (hmdPosition - startStartingR).magnitude;
                    Debug.Log("distanceTrigPressedFromHmd " + distanceTrigPressedFromHmd);
                    Debug.Log("distanceFromHmd " + distanceFromHmd);
                    int value;// = (int)Math.Round(trigger_drag_multiplier * distance);

                    if (distanceFromHmd > distanceTrigPressedFromHmd)
                    {
                        value = (int)Math.Round(trigger_drag_multiplier * distance);
                    }
                    else
                    {
                        value = (int)Math.Round(-trigger_drag_multiplier * distance);
                    }

                    // clip values
                    value = System.Math.Min(value, 210);
                    value = System.Math.Max(value, -210);
                    value = System.Math.Abs(value);

                    if (System.Math.Abs(value) > 20)
                    {
                        r_limb_has_moved = true;
                    }

                    vel_R = System.Convert.ToString(value);
                    startingL = position; // keep movements relative after each update
                }
            }
        }
        else
        {
            triggerPressedL = false;
            vel_R = "0";
        }
#endif


#if USE_ACCELERATION_RIGHT

        // Right velocity + movement control right motor

        Vector3 accel;
//        if (rightHandedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceAcceleration, out accel))
        if (rightHandedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceVelocity, out accel))
        {
            double floataccel = System.Math.Abs(accel[0]) + System.Math.Abs(accel[1]) + System.Math.Abs(accel[2]);// System.Math.Max(System.Math.Min(accel[0], one), negone);
                                                                                                                  //double floatpos = System.Math.Max(System.Math.Min(position[1] - 1, one), zero);

            if (System.Math.Abs(floataccel) > .1f)
            {
                scalar_accel_R = System.Math.Max(scalar_accel_R, (int)(System.Math.Sign(floataccel) * accel_sensitivity_multiplier * System.Math.Abs(floataccel)));
            }
            else
            {
                scalar_accel_R = 0;
            }

            // clip values
            scalar_accel_R = System.Math.Min(scalar_accel_R, 210);
            scalar_accel_R = System.Math.Max(scalar_accel_R, -210);
            scalar_accel_R = System.Math.Abs(scalar_accel_R);

            string vel_L = System.Convert.ToString(scalar_accel_R);
        }
#endif

#if USE_ABSOLUTE
        // use absolute position of right-hand controller to control right motor

        Vector3 position;
        if (rightHandedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out position))
        {
            if (!zeroedR && position[0] != 0f)
            {
                startingR = position;
                Debug.Log("Starting position right: " + startingR);
                zeroedR = true;
            }
            double floatpos = .5 + .5 * System.Math.Max(System.Math.Min(controllers_sensitivity_multiplier * (position - startingR)[1], one), negone);
            //double floatpos = System.Math.Max(System.Math.Min(position[1] - 1, one), zero);
            int pos = (int)(359 * floatpos);
            string strValue = System.Convert.ToString(pos);

            // publish a message with the angle in degrees
            if (i == 0)
            {
                client.Publish(MQTT_PREFIX + "right_controller_position", System.Text.Encoding.UTF8.GetBytes(strValue), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, true);
            }
        }

        if (leftHandedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out position))
        {
            if (!zeroedL && position[0] != 0f)
            {
                startingL = position;
                Debug.Log("Starting position left: " + startingL);
                zeroedL = true;
            }

            //double floatpos = System.Math.Max(System.Math.Min(position[1] - 1, one), zero);
            double floatpos = .5 + .5 * System.Math.Max(System.Math.Min(controllers_sensitivity_multiplier * (position - startingL)[1], one), negone);
            int pos = (int)(359 * floatpos);
            string strValue = System.Convert.ToString(pos);

            // publish a message with the angle in degrees
            if (i == 1)
            {
                client.Publish(MQTT_PREFIX + "left_controller_position", System.Text.Encoding.UTF8.GetBytes(strValue), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, true);
            }
        }

#endif
        /*
        // position-based version of the headset tracking
        if (headMountedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out position))
        {
            if (!zeroedC && position[0] != 0f)
            {
                startingC = position;
                Debug.Log("Starting position HMD: " + startingC);
                zeroedC = true;
            }t pos = " + position);

            //Debug.Log("Starting position HMD: " + startingC + ", curren
            double floatpos = .5 + .5 * System.Math.Max(System.Math.Min(hmd_sensitivity_multiplier * (position - startingC)[1], one), negone);
            //Debug.Log("HMD y = " + floatpos);

            int pos = (int)(359 * floatpos);
            string strValue = System.Convert.ToString(pos);

            if (i == 2) 
            {
                client.Publish(MQTT_PREFIX + "head_mounted_position", System.Text.Encoding.UTF8.GetBytes(strValue), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, true);
            }
        }
        */
        Quaternion rotation;
        if (headMountedControllers[0].TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceRotation, out rotation))
        {
            double pitchAngle;
            double yawAngle;

            // yaw
            yawAngle = rotation.eulerAngles[1];

            if (yawAngle > 180f)
            {
                yawAngle -= 360f;
            }

            //Debug.Log("Yaw angle: ");
            //Debug.Log(yawAngle);

            if (Double.IsNaN(yawAngle_zero) && yawAngle != 0f)
            {
                yawAngle_zero = yawAngle;

                //Debug.Log("Yaw angle zero: ");
                //Debug.Log(yawAngle_zero);
            }

            yawAngle -= yawAngle_zero;

            // pitch
            pitchAngle = rotation.eulerAngles[0];

            if (pitchAngle > 180f)
            {
                pitchAngle -= 360f;
            }

            if (Double.IsNaN(pitchAngle_zero) && pitchAngle != 0f)
            {
                pitchAngle_zero = pitchAngle;

            pitchAngle -= pitchAngle_zero;

            int intyaw =  180 - (90 + (int)(.5 * hmd_yaw_sensitivity_multiplier * yawAngle));
            int intpitch = 20 + 150 - (90 + (int)(.5 * hmd_pitch_sensitivity_multiplier * pitchAngle));

            intyaw = System.Math.Max(intyaw, min_yaw);
            intyaw = System.Math.Min(intyaw, max_yaw);

            intpitch = System.Math.Max(intpitch, min_pitch);
            intpitch = System.Math.Min(intpitch, max_pitch);

            aux_pos_2 = System.Convert.ToString(intpitch);
            aux_pos_1 = System.Convert.ToString(intyaw);
        }

        /**
         * Send the command data onto the network
        **/

        if (i == 0)
        {
            string message = "{\"vel_L\": \"" + vel_L + "\", \"vel_R\": \"" + vel_R + "\", \"vel_C\": \"" + vel_C + "\", \"aux_pos_1\": \"" + aux_pos_1 + "\", \"aux_pos_2\": \"" + aux_pos_2 + "\"}";
            Debug.Log("sending message " + message);
            byte[] data = Encoding.UTF8.GetBytes(message);
            client.Send(data, data.Length);
            //client.Send(data, data.Length, remoteEndPoint);
        }

        i = (i + 1) % NETWORK_SEND_EVERY_N_TH_FRAME;
    }
}
