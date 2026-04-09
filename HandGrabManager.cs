using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class HandGrabManager : MonoBehaviour
    {
        [Header("Referencias principales")]
        public JointGripSubscriber jointSubscriber;
        public Transform grabAnchor; 
        public Transform handRoot;

        [Header("Configuración de Cierre (-90 = Cerrado, 0 = Abierto)")]
        public float grabThreshold = -75f; 
        public float releaseThreshold = -20f; 
        public int minJointsRequired = 3;
        public float grabSearchRadius = 0.08f;
        public LayerMask grabbableLayerMask = ~0;

        private GameObject grabbedObject = null;
        private Transform originalParent = null;
        private bool isCurrentlyGrabbing = false;

        // Variables para guardar la pose relativa
        private Vector3 relativePosition;
        private Quaternion relativeRotation;

        void Update()
        {
            bool handIsClosed = CheckIfHandIsClosed();

            if (handIsClosed && !isCurrentlyGrabbing)
            {
                OnGrabStart();
            }
            else if (CheckIfHandIsOpen() && isCurrentlyGrabbing)
            {
                OnGrabEnd();
            }
        }

        void LateUpdate()
        {
            // AQUÍ ESTÁ EL CAMBIO: Usamos las coordenadas LOCALES guardadas
            // para mantener el objeto exactamente donde se agarró
            if (isCurrentlyGrabbing && grabbedObject != null && grabAnchor != null)
            {
                grabbedObject.transform.localPosition = relativePosition;
                grabbedObject.transform.localRotation = relativeRotation;
            }
        }

        private bool CheckIfHandIsClosed()
        {
            if (jointSubscriber == null || jointSubscriber.HingeJoints == null) return false;
            int count = 0;
            foreach (var hinge in jointSubscriber.HingeJoints)
            {
                if (hinge != null && hinge.spring.targetPosition <= grabThreshold) count++;
            }
            return count >= minJointsRequired;
        }

        private bool CheckIfHandIsOpen()
        {
            if (jointSubscriber == null || jointSubscriber.HingeJoints == null) return true;
            int count = 0;
            foreach (var hinge in jointSubscriber.HingeJoints)
            {
                if (hinge != null && hinge.spring.targetPosition >= releaseThreshold) count++;
            }
            return count >= minJointsRequired;
        }

        private void OnGrabStart()
        {
            Collider[] hits = Physics.OverlapSphere(grabAnchor.position, grabSearchRadius, grabbableLayerMask);
            GameObject target = null;
            float minDist = float.MaxValue;

            foreach (var hit in hits)
            {
                if (handRoot != null && hit.transform.IsChildOf(handRoot)) continue;
                float dist = Vector3.Distance(grabAnchor.position, hit.transform.position);
                if (dist < minDist)
                {
                    minDist = dist;
                    target = hit.attachedRigidbody ? hit.attachedRigidbody.gameObject : hit.gameObject;
                }
            }

            if (target != null)
            {
                grabbedObject = target;
                originalParent = grabbedObject.transform.parent;

                Rigidbody rb = grabbedObject.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    rb.isKinematic = true; 
                    rb.detectCollisions = false; 
                }

                // 1. Lo emparentamos manteniendo su posición en el mundo (true)
                grabbedObject.transform.SetParent(grabAnchor, true);

                // 2. CAPTURAMOS su posición y rotación LOCALES respecto al ancla
                // Esto guarda la "distancia" y "giro" que tiene el objeto frente a la palma
                relativePosition = grabbedObject.transform.localPosition;
                relativeRotation = grabbedObject.transform.localRotation;

                isCurrentlyGrabbing = true;
                Debug.Log("<color=green>[HandGrab] AGARRADO con orientación original</color>");
            }
        }

        private void OnGrabEnd()
        {
            if (grabbedObject == null) return;

            Rigidbody rb = grabbedObject.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = false;
                rb.detectCollisions = true; 
                rb.useGravity = true;
            }

            grabbedObject.transform.SetParent(originalParent, true);
            grabbedObject = null;
            isCurrentlyGrabbing = false;
            Debug.Log("<color=red>[HandGrab] SOLTADO</color>");
        }
    }
}
