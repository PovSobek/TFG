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
        
        #pos = math.radians(0.0)
        i = -90

        while rclpy.ok() and self._node.continuar is None:
            if i < 0:
                pos = math.radians(i)
                msg.position = [pos] * len(msg.name) # Un valor por cada nombre en msg.name
                self._node._pub_cmd.publish(msg)
                i += 1
            else:
                self._node._pub_cmd.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if self._node.continuar:
            return 'abierto_c'
        else:
            return 'abierto_e'

###########################################
#                 CERRAR                  #
###########################################

class cerrar(State):
    """
    Estado que sirve para cerrar la mano de 4 en 4.
    """
    def __init__(self, node):
        super().__init__(['cerrado_a', 'cerrado_e'])
        self._node = node
    
    def execute(self, blackboard):
        print('CERRANDO MANO')
        self._node.continuar = None

        msg = JointState()
        # Nombres de las 16 articulaciones (a-p)
        msg.name = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p']
        
        # Inicializamos todas en 0 (abierto)
        current_positions = [0.0] * len(msg.name)
        
        # Definimos los 4 grupos de índices (0-3, 4-7, 8-11, 12-15)
        grupos = [range(0, 4), range(4, 8), range(8, 12), range(12, 16)]

        for grupo in grupos:
            # Si el usuario mandó una transición durante el proceso, paramos
            if self._node.continuar is not None:
                break
                
            # Animación de cierre para el grupo actual
            i = 0
            while rclpy.ok() and i > -90:
                pos_rad = math.radians(i)
                
                # Actualizamos solo los dedos del grupo actual
                for idx in grupo:
                    current_positions[idx] = pos_rad
                
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.position = current_positions
                self._node._pub_cmd.publish(msg)
                
                i -= 1  # Velocidad de cierre
                time.sleep(0.01)
            
            # Pausa entre el cierre de un grupo y el siguiente
            time.sleep(0.5)
            rclpy.spin_once(self._node, timeout_sec=0.1)

        # Esperar a que el usuario decida el siguiente estado si no lo hizo ya
        while rclpy.ok() and self._node.continuar is None:
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
    
    sm.set_start_state('abrir')
    sm.validate()

    outcome = sm()
    node.get_logger().info(f"FSM terminada con outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
