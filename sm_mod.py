#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import yasmin
from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_ros import ActionState

from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math

###########################################
#             ESTRUCTURA NODO             #
###########################################

class HandNode(Node):
    """
    Estructura de la configuración de los estados.
    """
    def __init__(self):
        super().__init__("state_machine_mod")
        self.continuar = None
        self._pub_cmd = self.create_publisher(JointState, '/fingers', 10)
        self._sub_cmd = self.create_subscription(String, '/transition', self.cb_transition, 10)
    
    def cb_transition(self, msg):
        cmd = msg.data.lower()
        if cmd == "a":
            self.continuar = True
        elif cmd == "b":
            self.continuar = False
        else:
            self._node.get_logger().warn(f"Comando '{cmd}' no reconocido (usar a o b)")

###########################################
#                   ABRIR                 #
###########################################

class abrir(State):
    """
    Estado que sirve para abrir la mano modelada en Unity.
    """
    def __init__(self, node):
        super().__init__(['abierto_c', 'abierto_e'])
        self._node = node
    
    def execute(self, blackboard):
        print('ABRIENDO MANO')
        self._node.continuar = None

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        
        # Nombres articulaciones Unity
        msg.name = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p'] 
        
        pos = math.radians(0.0)
        msg.position = [pos] * len(msg.name) # Un valor por cada nombre en msg.name

        while rclpy.ok() and self._node.continuar is None:
            self._node._pub_cmd.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if self._node.continuar:
            return 'abierto_c'
        else:
            return 'abierto_e'

###########################################
#                   CERRAR                #
###########################################

class cerrar(State):
    """
    Estado que sirve para cerrar la mano modelada en Unity.
    """
    def __init__(self, node):
        super().__init__(['cerrado_a', 'cerrado_e'])
        self._node = node
    
    def execute(self, blackboard):
        print('CERRANDO MANO')
        self._node.continuar = None

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        
        # Nombres articulaciones Unity
        msg.name = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p'] 
        
        pos = math.radians(90.0)
        msg.position = [pos] * len(msg.name) # Un valor por cada nombre en msg.name

        while rclpy.ok() and self._node.continuar is None:
            self._node._pub_cmd.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if self._node.continuar:
            return 'cerrado_a'
        else:
            return 'cerrado_e'
        
###########################################
#                   MAIN                  #
###########################################

def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
    node.get_logger().info("Para cambiar de estado, lanzar el topic /transition con a para cambiar de estado y b para salir de la aplicación.")

    sm = StateMachine(outcomes=['end'])

    sm.add_state('abrir', abrir(node),
                 transitions={'abierto_c':'cerrar',
                              'abierto_e':'end'})
    
    sm.add_state('cerrar', cerrar(node),
                 transitions={'cerrado_a':'abrir',
                              'cerrado_e':'end'})
    
    sm.set_start_state('cerrar')
    sm.validate()

    outcome = sm()
    node.get_logger().info(f"FSM terminada con outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
