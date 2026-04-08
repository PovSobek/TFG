using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;

namespace RosSharp.RosBridgeClient
{
    public class JointGripSubscriber : UnitySubscriber<JointState>
    {
        public List<string> JointNames;
        public List<HingeJoint> HingeJoints;

        [Header("Configuración de Fuerza")]
        public float SpringForce = 1000f;
        public float Damper = 100f;

        // Buffer para sincronizar hilos
        private float[] targetAngles;
        private bool hasNewData = false;

        protected override void Start()
        {
            base.Start();
            targetAngles = new float[JointNames.Count];
            InitializeJoints();
        }

        private void InitializeJoints()
        {
            foreach (var hinge in HingeJoints)
            {
                if (hinge == null) continue;

                hinge.useSpring = true;
                JointSpring spring = hinge.spring;
                spring.spring = SpringForce;
                spring.damper = Damper;
                hinge.spring = spring;

                if (hinge.GetComponent<Rigidbody>())
                {
                    hinge.GetComponent<Rigidbody>().isKinematic = false;
                    hinge.GetComponent<Rigidbody>().useGravity = false;
                }
            }
        }

        protected override void ReceiveMessage(JointState message)
        {
            // ROS THREAD: Guardamos los valores en el buffer
            for (int i = 0; i < message.name.Length; i++)
            {
                int index = JointNames.IndexOf(message.name[i]);
                if (index != -1 && index < targetAngles.Length)
                {
                    targetAngles[index] = (float)message.position[i] * Mathf.Rad2Deg;
                    hasNewData = true;
                }
            }
        }

        private void Update()
        {
            // MAIN THREAD: Aplicamos los valores donde Unity sí permite la escritura física
            if (hasNewData)
            {
                for (int i = 0; i < HingeJoints.Count; i++)
                {
                    if (HingeJoints[i] != null)
                    {
                        WritePhysicsPosition(i, targetAngles[i]);
                    }
                }
                hasNewData = false;
            }
        }

        private void WritePhysicsPosition(int index, float angle)
        {
            HingeJoint hinge = HingeJoints[index];
            JointSpring spring = hinge.spring;
            spring.targetPosition = angle;
            hinge.spring = spring;

            // Forzamos el "despertar" del objeto para que procese el cambio de spring
            Rigidbody rb = hinge.GetComponent<Rigidbody>();
            if (rb != null) rb.WakeUp();
        }
    }
}
