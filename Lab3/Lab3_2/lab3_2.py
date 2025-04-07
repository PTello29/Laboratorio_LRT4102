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

    def normalize_angle(self, angle):
        # Normaliza un ángulo al rango [-π, π] y así evitar problemas de giros indefinidos
        while angle > 3.14159:
            angle -= 2 * 3.14159
        while angle < -3.14159:
            angle += 2 * 3.14159
        return angle

    def move_turtle_to_desired_position(self, desired_x, desired_y, desired_theta):
        # Constantes de proporcionalidad del controlador (ajustables)
        Kp_linear = 1.0
        Kp_angular = 4.0

        # Inicializar el mensaje de velocidad
        vel_msg = Twist()

        while not rospy.is_shutdown():
            # Calcular la distancia y el ángulo hacia la posición deseada
            distance = sqrt((desired_x - self.current_x)**2 + (desired_y - self.current_y)**2)
            angle_to_goal = atan2(desired_y - self.current_y, desired_x - self.current_x)
            angular_error = self.normalize_angle(angle_to_goal - self.current_theta)

            # Control proporcional para el movimiento lineal y angular
            vel_msg.linear.x = Kp_linear * distance
            vel_msg.angular.z = Kp_angular * angular_error

            # Publicar el mensaje de velocidad
            self.velocity_publisher.publish(vel_msg)

            # Verificar si la tortuga ha alcanzado la posición deseada
            if distance < 0.1:  # Umbral para la distancia
                break

            # Esperar hasta la siguiente iteración
            self.rate.sleep()

        # Ajustar la orientación final de la tortuga
        while not rospy.is_shutdown():
            # Calcular el error angular
            angular_error = self.normalize_angle(desired_theta - self.current_theta)
            if abs(angular_error) < 0.01:  # Umbral para el ángulo
                break

            # Control proporcional para el movimiento angular
            vel_msg.linear.x = 0
            vel_msg.angular.z = Kp_angular * angular_error
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Detener la tortuga
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def get_desired_x_from_user(self):
        # Solicitar al usuario la posición deseada en el eje x
        print("Ingrese la posición deseada en el eje x:")
        return float(input("Coordenada x: "))
    
    def get_desired_y_from_user(self):
        # Solicitar al usuario la posición deseada en el eje y
        print("Ingrese la posición deseada en el eje y:")
        return float(input("Coordenada y: "))
    
    def get_desired_theta_from_user(self):
        # Solicitar al usuario el ángulo deseado theta
        print("Ingrese el ángulo deseado theta:")
        return float(input("Ángulo theta: "))

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_x = self.get_desired_x_from_user()
            desired_y = self.get_desired_y_from_user()
            desired_theta = self.get_desired_theta_from_user()

            # Mover la tortuga a la posición deseada
            rospy.loginfo(f"Moviendo la tortuga a x: {desired_x}, y: {desired_y}, theta: {desired_theta}")
            rospy.loginfo("-----------------------------------------------")
            self.move_turtle_to_desired_position(desired_x, desired_y, desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass

