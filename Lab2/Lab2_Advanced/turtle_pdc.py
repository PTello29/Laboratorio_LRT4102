#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt

class MoveTurtlePDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_pd')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.last_error_distance = 0
        self.last_error_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_position(self, desired_x, desired_y):
        # Constantes de proporcionalidad y derivativa del controlador (ajustables)
        Kp_linear = 1
        Kd_linear = 0.1
        Kp_angular = 4
        Kd_angular = 0.1

        while not rospy.is_shutdown():
            # Calcular el error de posición
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            
            # Calcular la distancia al objetivo
            distance = sqrt(error_x**2 + error_y**2)
            
            # Calcular el ángulo deseado
            desired_theta = atan2(error_y, error_x)
            
            # Calcular el error angular
            error_theta = desired_theta - self.current_theta
            
            # Calcular las velocidades lineal y angular con términos derivativos
            vel_linear = Kp_linear * distance + Kd_linear * (distance - self.last_error_distance)
            vel_angular = Kp_angular * error_theta + Kd_angular * (error_theta - self.last_error_theta)
            
            # Guardar los errores actuales para la próxima iteración
            self.last_error_distance = distance
            self.last_error_theta = error_theta
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_linear
            twist_msg.angular.z = vel_angular
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y las variables de velocidad en la terminal
            rospy.loginfo("Posición actual: (%f, %f), Error: (%f, %f), Vel_linear: %f, Vel_angular: %f", 
                          self.current_x, self.current_y, error_x, error_y, vel_linear, vel_angular)
            
            # Verificar si se alcanza la posición deseada
            if distance < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

    def get_desired_x_from_user(self):
        print("Ingrese la posición deseada en el eje x:")
        return float(input("Coordenada x: "))
    
    def get_desired_y_from_user(self):
        print("Ingrese la posición deseada en el eje y:")
        return float(input("Coordenada y: "))

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_x = self.get_desired_x_from_user()
            desired_y = self.get_desired_y_from_user()

            # Mover la tortuga a la posición deseada
            self.move_turtle_to_desired_position(desired_x, desired_y)

if __name__ == '__main__':
    try:
        move_turtle_pd = MoveTurtlePDControl()
        move_turtle_pd.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass

