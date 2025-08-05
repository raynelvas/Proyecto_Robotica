from behavior_tree_multi_robot.tree_nodes import (
    WaitForStart,
    WaitForOrder,
    ResetTransformNode,
    InputCoordinates,
    GoToPosition, 
    DetectObject,
    MoveArmNode,
    CheckOtherRobotPresent
)
import py_trees

#===========================secuencias de navegación===========================
def create_navigation_sequences(node):    
    nav_seq1 = py_trees.composites.Sequence("NavigateToObject", memory=True)
    nav_seq1.add_children([
        InputCoordinates(name="GetCoordinatesObject",node=node,target_type='cubo'),
        GoToPosition(name="NavegateObj",node=node, goal_tolerance=0.1)
        #DetectObject(name="DetectObject",node=node)
    ])

    nav_seq2 = py_trees.composites.Sequence("NavigateToDeposit", memory=True)
    nav_seq2.add_children([
        InputCoordinates(name="GetCoordinatesDeposit",node=node,target_type='deposito'),
        GoToPosition(name="NavegateDep",node=node, goal_tolerance=0.15)
    ])

    nav_seq3 = py_trees.composites.Sequence("NavigateToOrigin", memory=True)
    nav_seq3.add_children([
        InputCoordinates(name="GetCoordinatesOrigin",node=node,target_type='origen'),
        GoToPosition(name="NavegateOrg",node=node, goal_tolerance=0.04) 
    ])
    return nav_seq1, nav_seq2, nav_seq3
#==================================================================================

#===================secuencias de manipulación del brazo======================
def create_arm_sequences(node):
    arm_seq1 = py_trees.composites.Sequence("HoldObject", memory=True)
    arm_seq1.add_children([
        MoveArmNode("Grasp", node=node,q1=95,q2=0,q3=80,efector=1),              
        MoveArmNode("SafePosObj", node=node,q1=95,q2=70,q3=40,efector=1)
    ]) 

    arm_seq2 = py_trees.composites.Sequence("ReleaseObject", memory=True)
    arm_seq2.add_children([
        MoveArmNode("RealeasePos1", node=node,q1=95,q2=70,q3=80,efector=1),
        MoveArmNode("RealeasePos2", node=node,q1=95,q2=0,q3=80,efector=0),
        MoveArmNode("SafePosHome", node=node,q1=180,q2=70,q3=80,efector=0)
    ]) 
    return arm_seq1, arm_seq2
#==================================================================================

#===================secuencia para enviar al otro robot al origen=====================
def create_other_robot(node):
    # Subrama para mover al otro robot al origen
    move_other_robot_home = py_trees.composites.Sequence("MoveOtherRobotHome", memory=True)
    move_other_robot_home.add_children([
        InputCoordinates(name="GetOtherRobotOrigin", node=node, target_type='other_origin'),
        GoToPosition(name="MoveOtherRobotOrigin", node=node, position='other', goal_tolerance=0.1)
    ])

    # Secuencia condicional
    handle_other_robot = py_trees.composites.Sequence("HandleOtherRobot", memory=True)
    handle_other_robot.add_children([
        CheckOtherRobotPresent(name="CheckOtherRobot",node=node),
        move_other_robot_home
    ])

    # Selector para que si no hay otro robot, salte igual con SUCCESS
    optional_other_robot = py_trees.composites.Selector("OptionalOtherRobot", memory=False)
    optional_other_robot.add_children([
        handle_other_robot,
        py_trees.behaviours.Success(name="SkipOtherRobot")
    ])
    return optional_other_robot
#==================================================================================

#===========================CONSTRUCCIÓN DEL ARBOL PRINCIPAL===========================
def build_tree(node):    
    """Construye y retorna el árbol de comportamiento completo"""
    nav_seq1, nav_seq2, nav_seq3 = create_navigation_sequences(node)
    arm_seq1, arm_seq2 = create_arm_sequences(node)
    optional_other_robot = create_other_robot(node)

    main_sequence = py_trees.composites.Sequence("MainSequence", memory=True)
    main_sequence.add_children([
        WaitForStart(name="Start", node=node),
        WaitForOrder(name="SelectRobot", node=node), 
        ResetTransformNode(name="RobotTransform", node=node),
        optional_other_robot,   # <<--- AQUI VA EL BLOQUE OPCIONAL                     
        nav_seq1,
        arm_seq1,                          
        nav_seq2,                                    
        arm_seq2,                                                                                                       # PosiciónSegura 2
        nav_seq3
    ])
    # Visualizar estructura del árbol (solo una vez)
    py_trees.display.render_dot_tree(main_sequence, name="bt")

    return py_trees.trees.BehaviourTree(main_sequence)
#==================================================================================
