# Reporte de laboratorio 3
En este reporte, se elaboraron dos actividades complementarias. En la primera parte se desarrolló un código que una tortuga llegue a una posición y orintación proporcionadas por el usuario al matar a la tortuga y generar una nueva en la posición acordada. El código quedó de la siguiente forma:

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from math import atan2, sqrt

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_x')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Inicializar variables de posición
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def get_desired_x_from_user(self):
        # Función para obtener la posición deseada del usuario
        print("Ingrese la posición deseada en el eje x:")
        return float(input("Coordenada x: "))
    
    def get_desired_y_from_user(self):
        # Función para obtener la posición deseada del usuario
        print("Ingrese la posición deseada en el eje y:")
        return float(input("Coordenada y: "))
    
    def get_desired_theta_from_user(self):
        # Función para obtener el ángulo deseado del usuario
        print("Ingrese el ángulo deseado theta:")
        return float(input("Ángulo theta: "))

    def move_turtle_interactively(self):
        current_turtle_name = 'turtle1'  # Nombre inicial de la tortuga
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_x = self.get_desired_x_from_user()
            desired_y = self.get_desired_y_from_user()
            desired_theta = self.get_desired_theta_from_user()

            # Eliminar la tortuga actual
            rospy.wait_for_service('/kill')
            try:
                kill_turtle = rospy.ServiceProxy('/kill', Kill)
                kill_turtle(current_turtle_name)
            except rospy.ServiceException as e:
                rospy.logerr(f"Error al eliminar la tortuga: {e}")
                return

            # Crear una nueva tortuga en la posición deseada
            rospy.wait_for_service('/spawn')
            try:
                new_turtle_name = 'turtle2'  # Nombre de la nueva tortuga
                spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
                spawn_turtle(desired_x, desired_y, desired_theta, new_turtle_name)
                rospy.loginfo("Nueva tortuga creada en la posición deseada")
                current_turtle_name = new_turtle_name  # Actualizar el nombre de la tortuga actual
            except rospy.ServiceException as e:
                rospy.logerr(f"Error al crear la nueva tortuga: {e}")
                return

            # Actualizar los tópicos para la nueva tortuga
            self.pose_subscriber = rospy.Subscriber(f'/{current_turtle_name}/pose', Pose, self.pose_callback)
            self.velocity_publisher = rospy.Publisher(f'/{current_turtle_name}/cmd_vel', Twist, queue_size=10)

if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
```

