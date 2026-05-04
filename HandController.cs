using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;

namespace RosSharp.RosBridgeClient
{
    public class JointGripSubscriber : UnitySubscriber<JointState>
    {
        public List<string> JointNames;
        public List<HingeJoint> HingeJoints;

        public float SpringForce = 1200f; // Fuerza firme pero no infinita
        public float Damper = 100f;

        private float[] targetAngles;
        private bool[] fingerLocked; // Nueva: para bloquear dedos individualmente
        private bool hasNewData = false;

        protected override void Start()
        {
            base.Start();
            targetAngles = new float[JointNames.Count];
            fingerLocked = new bool[HingeJoints.Count];
            InitializeJoints();
        }

        private void InitializeJoints()
        {
            foreach (var hinge in HingeJoints)
            {
                if (hinge == null) continue;
                hinge.useSpring = true;
                if (hinge.GetComponent<Rigidbody>())
                {
                    var rb = hinge.GetComponent<Rigidbody>();
                    rb.isKinematic = false;
                    rb.collisionDetectionMode = CollisionDetectionMode.Continuous; // <--- CRUCIAL
                }
            }
        }

        protected override void ReceiveMessage(JointState message)
        {
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
            if (hasNewData)
            {
                for (int i = 0; i < HingeJoints.Count; i++)
                {
                    // Solo actualizamos el dedo si NO est� bloqueado por contacto
                    if (HingeJoints[i] != null && !fingerLocked[i])
                    {
                        WritePhysicsPosition(i, targetAngles[i]);
                    }
                }
                hasNewData = false;
            }
        }

        // Bloquea el dedo en su posici�n actual
        public void SetLock(int index, bool locked)
        {
            if (index < 0 || index >= fingerLocked.Length) return;
            fingerLocked[index] = locked;
            if (locked) WritePhysicsPosition(index, HingeJoints[index].angle);
        }

        public void ReleaseAllLocks()
        {
            for (int i = 0; i < fingerLocked.Length; i++) fingerLocked[i] = false;
        }

        private void WritePhysicsPosition(int index, float angle)
        {
            HingeJoint hinge = HingeJoints[index];
            JointSpring spring = hinge.spring;
            spring.spring = SpringForce;
            spring.damper = Damper;
            spring.targetPosition = angle;
            hinge.spring = spring;
        }
    }
}
