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
        self.pos = None
        self._pub_cmd = self.create_publisher(JointState, '/fingers', 10)
        self._sub_cmd = self.create_subscription(String, '/transition', self.cb_transition, 10)
        self._sub_pos = self.create_subscription(JointState, '/pos', self.cb_pos, 10)

    def cb_pos(self, msg):
        pos_deg = [-math.degrees(p) for p in msg.position]
        self.pos = [round(p, 0) for p in pos_deg]
        print(self.pos)
    
    def cb_transition(self, msg):
        cmd = msg.data.lower()
        if cmd == "a":
            self.continuar = "cerrar"
        elif cmd == "b":
            self.continuar = "abrir"
        elif cmd == "c":
            self.continuar = "pinza"
        elif cmd == "d":
            self.continuar = "agarre_completo"
        elif cmd == "e":
            self.continuar = "end"
        else:
            self._node.get_logger().warn(f"Comando '{cmd}' no reconocido (usar a, b, c, d o e)")

###########################################
#                   ABRIR                 #
###########################################

class abrir(State):
    """
    Estado que sirve para abrir la mano modelada en Unity.
    """
    def __init__(self, node):
        super().__init__(['abierto_c', 'abierto_e', 'abierto_p', 'abierto_ac'])
        self._node = node
    
    def execute(self, blackboard):
        print('ABRIENDO MANO')
        self._node.continuar = None

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        
        # Nombres articulaciones Unity
        msg.name = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p'] 
        
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

        if self._node.continuar == "cerrar":
            return 'abierto_c'
        elif self._node.continuar == "pinza":
            return 'abierto_p'
        elif self._node.continuar == "agarre_completo":
            return 'abierto_ac'
        elif self._node.continuar == "end":
            return 'abierto_e'

###########################################
#             AGARRE COMPLETO             #
###########################################

class agarre_completo(State):
    """
    Estado que sirve para agarrar con la mano cerrada.
    """
    def __init__(self, node):
        super().__init__(['agarre_a', 'agarre_e', 'agarre_p', 'agarre_c'])
        self._node = node
    
    def execute(self, blackboard):
        print('CERRANDO MANO: AGARRE COMPLETO')
        self._node.continuar = None

        msg = JointState()
        # Nombres de las 16 articulaciones (a-p)
        msg.name = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p']
        
        # Inicializamos todas en 0 (abierto)
        current_positions = [0.0] * len(msg.name)
        
        grupos = [range(0, 5), range(5, 10), range(10, 15), range(16, 16)]

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

        if self._node.continuar == "abrir":
            return 'agarre_a'
        elif self._node.continuar == "pinza":
            return 'agarre_p'
        elif self._node.continuar == "cerrar":
            return 'agarre_c'
        elif self._node.continuar == "end":
            return 'cerrado_e'
        
###########################################
#                 PINZA                   #
###########################################

class pinza(State):
    """
    Estado que sirve para agarrar en modo pinza.
    """
    def __init__(self, node):
        super().__init__(['pinza_a', 'pinza_e', 'pinza_c', 'pinza_ac'])
        self._node = node
    
    def execute(self, blackboard):
        print('PINZA SECUENCIAL')
        self._node.continuar = None

        msg = JointState()
        msg.name = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p']
        indices_especiales = {5, 9, 10, 14, 15}
        
        # Inicializamos todas en 0.0
        current_positions = [0.0] * len(msg.name)

        # --- FASE 1: Articulaciones generales ---
        i = 0
        while rclpy.ok() and i >= -90 and self._node.continuar is None:
            pos_rad = math.radians(i)
            # Solo actualizamos las que NO son especiales
            for idx in range(len(msg.name)):
                if idx not in indices_especiales:
                    current_positions[idx] = pos_rad
            
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.position = current_positions
            self._node._pub_cmd.publish(msg)
            
            i -= 1
            time.sleep(0.01)
            rclpy.spin_once(self._node, timeout_sec=0.01)

        # --- FASE 2: Articulaciones especiales ---
        time.sleep(1)
        j = 0
        while rclpy.ok() and j >= -15 and self._node.continuar is None:
            pos_rad = math.radians(j)
            # Solo actualizamos las especiales
            for idx in indices_especiales:
                current_positions[idx] = pos_rad
            
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.position = current_positions
            self._node._pub_cmd.publish(msg)
            
            j -= 1
            time.sleep(0.01)
            rclpy.spin_once(self._node, timeout_sec=0.01)

        # --- ESPERA FINAL (Bucle original para transición) ---
        while rclpy.ok() and self._node.continuar is None:
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.position = current_positions
            self._node._pub_cmd.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.1)

        # Estructura de retorno original
        if self._node.continuar == "cerrar":
            return 'pinza_c'
        elif self._node.continuar == "abrir":
            return 'pinza_a'
        elif self._node.continuar == "agarre_completo":
            return 'pinza_ac'
        elif self._node.continuar == "end":
            return 'pinza_e'
        
###########################################
#                 CERRAR                  #
###########################################

class cerrar(State):
    """
    Estado que sirve para cerrar la mano.
    """
    def __init__(self, node):
        super().__init__(['cerrado_ac', 'cerrado_e', 'cerrado_p', 'cerrado_a'])
        self._node = node
    
    def execute(self, blackboard):
        print('CERRAR MANO')
        self._node.continuar = None

        msg = JointState()
        msg.name = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p']
        indices_especiales = {9, 14, 15}
        
        # Inicializamos todas en 0.0
        current_positions = [0.0] * len(msg.name)

        # --- FASE 1: Articulaciones generales ---
        i = 0
        while rclpy.ok() and i >= -90 and self._node.continuar is None:
            pos_rad = math.radians(i)
            # Solo actualizamos las que NO son especiales
            for idx in range(len(msg.name)):
                if idx not in indices_especiales:
                    current_positions[idx] = pos_rad
            
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.position = current_positions
            self._node._pub_cmd.publish(msg)
            
            i -= 1
            time.sleep(0.01)
            rclpy.spin_once(self._node, timeout_sec=0.01)

        # --- FASE 2: Articulaciones especiales ---
        time.sleep(1)
        j = 0
        while rclpy.ok() and j >= -90 and self._node.continuar is None:
            pos_rad = math.radians(j)
            # Solo actualizamos las especiales
            for idx in indices_especiales:
                current_positions[idx] = pos_rad
            
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.position = current_positions
            self._node._pub_cmd.publish(msg)
            
            j -= 1
            time.sleep(0.01)
            rclpy.spin_once(self._node, timeout_sec=0.01)

        # --- ESPERA FINAL (Bucle original para transición) ---
        while rclpy.ok() and self._node.continuar is None:
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.position = current_positions
            self._node._pub_cmd.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if self._node.continuar == "agarre_completo":
            return 'cerrado_ac'
        elif self._node.continuar == "abrir":
            return 'cerrado_a'
        elif self._node.continuar == "pinza":
            return 'cerrado_p'
        elif self._node.continuar == "end":
            return 'cerrado_e'

###########################################
#                   MAIN                  #
###########################################

def main(args=None):
    rclpy.init(args=args)
    node = HandNode()

    sm = StateMachine(outcomes=['end'])

    sm.add_state('abrir', abrir(node),
                 transitions={'abierto_c':'cerrar',
                              'abierto_p':'pinza',
                              'abierto_ac':'agarre_completo',
                              'abierto_e':'end'})
    
    sm.add_state('cerrar', cerrar(node),
                 transitions={'cerrado_a':'abrir',
                              'cerrado_p':'pinza',
                              'cerrado_ac':'agarre_completo',
                              'cerrado_e':'end'})
    
    sm.add_state('pinza', pinza(node),
                 transitions={'pinza_a':'abrir',
                              'pinza_c':'cerrar',
                              'pinza_ac':'agarre_completo',
                              'pinza_e':'end'})
    
    sm.add_state('agarre_completo', agarre_completo(node),
                 transitions={'agarre_c':'cerrar',
                              'agarre_p':'pinza',
                              'agarre_a':'abrir',
                              'agarre_e':'end'})
    
    sm.set_start_state('abrir')
    sm.validate()

    outcome = sm()
    node.get_logger().info(f"FSM terminada con outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
