#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
from behavior_tree_multi_robot.BehaviourTree import build_tree

class BTMainNode(Node):
    def __init__(self):
        super().__init__('bt_main_node')

def main(args=None):
    rclpy.init(args=args)
    node = BTMainNode()

    # No crees Blackboard, solo configura claves globales una vez
    py_trees.blackboard.Blackboard.set("order_info", None)
    py_trees.blackboard.Blackboard.set("goal_coords", None)
    py_trees.blackboard.Blackboard.set("other_robot", None)

    # Construir el árbol
    behaviour_tree = build_tree(node)
    behaviour_tree.setup(timeout=15)

    # 3. Bucle principal
    try:
        # Ejecutar el árbol hasta completar un ciclo
        while rclpy.ok():
            behaviour_tree.tick()
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Verificar si el ciclo se completó
            if behaviour_tree.root.status == py_trees.common.Status.SUCCESS:
                node.get_logger().info("Ciclo completado con éxito. Reiniciando...")
                behaviour_tree.interrupt()
                # Resetear el Blackboard
                py_trees.blackboard.Blackboard.set("order_info", None)
                py_trees.blackboard.Blackboard.set("goal_coords", None)
                py_trees.blackboard.Blackboard.set("other_robot", None)
        
                # Reinicialización completa
                behaviour_tree.setup(timeout=15)
                behaviour_tree.root.tick_once()  # Prepara para nuevo ciclo
            elif behaviour_tree.root.status == py_trees.common.Status.INVALID:
                node.get_logger().error("Estado inválido detectado. Reiniciando árbol...")

    except KeyboardInterrupt:
        node.get_logger().info("Ejecución finalizada")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

"""def main(args=None):
    rclpy.init(args=args)
    node = BTMainNode()

    # No crees Blackboard, solo configura claves globales una vez
    py_trees.blackboard.Blackboard.set("robot_ns", None)
    py_trees.blackboard.Blackboard.set("goal_coords", None)
    py_trees.blackboard.Blackboard.set("other_robot", None)

    
    # 3. Bucle principal
    try:
        while rclpy.ok():
             # Construir el árbol
            behaviour_tree = build_tree(node)
            behaviour_tree.setup(timeout=15)
            
            # Ejecutar el árbol hasta completar un ciclo
            while rclpy.ok():
                behaviour_tree.tick()
                rclpy.spin_once(node, timeout_sec=0.1)
                
                # Verificar si el ciclo se completó
                if behaviour_tree.root.status == py_trees.common.Status.SUCCESS:
                    node.get_logger().info("Ciclo completado con éxito. Reiniciando...")
                    break
                elif behaviour_tree.root.status == py_trees.common.Status.INVALID:
                    node.get_logger().error("Estado inválido detectado. Reiniciando árbol...")
                    break
            
            # Limpiar antes de reiniciar
            behaviour_tree.interrupt()
            
            # Resetear el Blackboard
            py_trees.blackboard.Blackboard.set("robot_ns", None)
            py_trees.blackboard.Blackboard.set("goal_coords", None)
            py_trees.blackboard.Blackboard.set("other_robot", None)
            
    except KeyboardInterrupt:
        node.get_logger().info("Ejecución finalizada")
    finally:
        rclpy.shutdown()"""