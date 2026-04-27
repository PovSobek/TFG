using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class HandGrabManager : MonoBehaviour
    {
        [Header("Referencias")]
        public JointGripSubscriber jointSubscriber;
        public Transform grabAnchor;
        public Transform handRoot;

        [Header("Configuración")]
        public float grabThreshold = -75f;
        public float releaseThreshold = -20f;
        public float grabSearchRadius = 0.12f;
        public LayerMask grabbableLayerMask = ~0;

        private GameObject grabbedObject = null;
        private Transform originalParent = null;
        private bool isCurrentlyGrabbing = false;
        private Vector3 relativePos;
        private Quaternion relativeRot;

        void Update()
        {
            bool handIsClosed = CheckIfHandIsClosed();
            if (handIsClosed && !isCurrentlyGrabbing) OnGrabStart();
            else if (CheckIfHandIsOpen() && isCurrentlyGrabbing) OnGrabEnd();
        }

        void LateUpdate()
        {
            if (isCurrentlyGrabbing && grabbedObject != null)
            {
                grabbedObject.transform.localPosition = relativePos;
                grabbedObject.transform.localRotation = relativeRot;
            }
        }

        private bool CheckIfHandIsClosed()
        {
            if (jointSubscriber == null) return false;
            int count = 0;
            foreach (var h in jointSubscriber.HingeJoints)
                if (h != null && h.spring.targetPosition <= grabThreshold) count++;
            return count >= 3;
        }

        private bool CheckIfHandIsOpen()
        {
            if (jointSubscriber == null) return true;
            int count = 0;
            foreach (var h in jointSubscriber.HingeJoints)
                if (h != null && h.spring.targetPosition >= releaseThreshold) count++;
            return count >= 3;
        }

        public bool IsTargetObject(GameObject obj) => grabbedObject != null && (obj == grabbedObject || obj.transform.IsChildOf(grabbedObject.transform));

        public void LockFinger(int index)
        {
            if (isCurrentlyGrabbing) jointSubscriber.SetLock(index, true);
        }

        private void OnGrabStart()
        {
            Collider[] hits = Physics.OverlapSphere(grabAnchor.position, grabSearchRadius, grabbableLayerMask);
            GameObject target = null;
            foreach (var hit in hits)
            {
                if (handRoot != null && hit.transform.IsChildOf(handRoot)) continue;

                // Identificamos el posible objetivo (objeto o su Rigidbody)
                GameObject potentialTarget = hit.attachedRigidbody ? hit.attachedRigidbody.gameObject : hit.gameObject;

                // SOLO si tiene la etiqueta "Interactuable", lo asignamos como target y salimos del bucle
                if (potentialTarget.CompareTag("Interactuable"))
                {
                    target = potentialTarget;
                    break;
                }
            }

            if (target != null)
            {
                grabbedObject = target;
                originalParent = grabbedObject.transform.parent;

                Rigidbody rb = grabbedObject.GetComponent<Rigidbody>();
                if (rb != null) { rb.isKinematic = true; rb.detectCollisions = true; }

                grabbedObject.transform.SetParent(grabAnchor, true);
                relativePos = grabbedObject.transform.localPosition;
                relativeRot = grabbedObject.transform.localRotation;

                isCurrentlyGrabbing = true;
            }
        }

        private void OnGrabEnd()
        {
            if (grabbedObject == null) return;
            Rigidbody rb = grabbedObject.GetComponent<Rigidbody>();
            if (rb != null) { rb.isKinematic = false; rb.detectCollisions = true; rb.useGravity = true; }

            grabbedObject.transform.SetParent(originalParent, true);
            jointSubscriber.ReleaseAllLocks(); // <--- IMPORTANTE: Desbloquear dedos
            grabbedObject = null;
            isCurrentlyGrabbing = false;
        }
    }
}
