using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

// Attach this component to one persistent GameObject in a Quest 3 Unity scene.
// It transmits controller state only; all robot safety gates stay on the Linux host.
public sealed class Quest3UdpSender : MonoBehaviour
{
    [Serializable] private class Buttons
    {
        public bool grip;
        public float trigger;
        public bool primary;
        public bool secondary;
        public bool menu;
    }

    [Serializable] private class Controller
    {
        public float[] position_m = new float[3];
        public float[] orientation_xyzw = new float[4];
        public bool tracking;
        public Buttons buttons = new Buttons();
    }

    [Serializable] private class Packet
    {
        public string schema = "arx.quest3.controllers.v1";
        public long sequence;
        public long client_time_ns;
        public Controller left = new Controller();
        public Controller right = new Controller();
    }

    [Tooltip("IPv4 address of the ARX collection computer")]
    public string robotHost = "192.168.1.10";
    public int robotPort = 7447;
    [Range(50, 120)] public int sendRateHz = 90;

    private UdpClient udp;
    private IPEndPoint endpoint;
    private InputDevice leftDevice;
    private InputDevice rightDevice;
    private readonly Packet packet = new Packet();
    private double nextSendTime;

    private void Start()
    {
        endpoint = new IPEndPoint(IPAddress.Parse(robotHost), robotPort);
        udp = new UdpClient();
        AcquireDevices();
    }

    private static InputDevice FindController(InputDeviceCharacteristics handedness)
    {
        var devices = new System.Collections.Generic.List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(
            InputDeviceCharacteristics.Controller | handedness, devices);
        return devices.Count == 0 ? default : devices[0];
    }

    private void AcquireDevices()
    {
        if (!leftDevice.isValid)
            leftDevice = FindController(InputDeviceCharacteristics.Left);
        if (!rightDevice.isValid)
            rightDevice = FindController(InputDeviceCharacteristics.Right);
    }

    // Unity/OpenXR is left-handed (+Z forward). Convert to a right-handed frame
    // (+X right, +Y up, +Z backward) before the robot-host calibration matrix.
    private static void Fill(InputDevice device, Controller output)
    {
        bool tracked = device.isValid
            && device.TryGetFeatureValue(CommonUsages.isTracked, out bool isTracked)
            && isTracked
            && device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 p)
            && device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion q);

        output.tracking = tracked;
        if (tracked)
        {
            output.position_m[0] = p.x;
            output.position_m[1] = p.y;
            output.position_m[2] = -p.z;
            output.orientation_xyzw[0] = -q.x;
            output.orientation_xyzw[1] = -q.y;
            output.orientation_xyzw[2] = q.z;
            output.orientation_xyzw[3] = q.w;
        }
        else
        {
            output.position_m[0] = output.position_m[1] = output.position_m[2] = 0f;
            output.orientation_xyzw[0] = output.orientation_xyzw[1] = output.orientation_xyzw[2] = 0f;
            output.orientation_xyzw[3] = 1f;
        }

        device.TryGetFeatureValue(CommonUsages.gripButton, out output.buttons.grip);
        device.TryGetFeatureValue(CommonUsages.trigger, out output.buttons.trigger);
        device.TryGetFeatureValue(CommonUsages.primaryButton, out output.buttons.primary);
        device.TryGetFeatureValue(CommonUsages.secondaryButton, out output.buttons.secondary);
        device.TryGetFeatureValue(CommonUsages.menuButton, out output.buttons.menu);

        // Never authorize motion from a controller whose pose is invalid.
        output.buttons.grip = output.buttons.grip && tracked;
        output.buttons.trigger = tracked ? Mathf.Clamp01(output.buttons.trigger) : 0f;
    }

    private void Update()
    {
        AcquireDevices();
        double now = Time.realtimeSinceStartupAsDouble;
        if (now < nextSendTime) return;
        nextSendTime = now + 1.0 / Math.Max(1, sendRateHz);

        Fill(leftDevice, packet.left);
        Fill(rightDevice, packet.right);
        packet.sequence++;
        packet.client_time_ns = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1_000_000L;
        byte[] bytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(packet));
        udp.Send(bytes, bytes.Length, endpoint);
    }

    private void OnDestroy()
    {
        udp?.Close();
        udp = null;
    }
}
