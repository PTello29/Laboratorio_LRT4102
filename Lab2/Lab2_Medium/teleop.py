#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Lee un solo caracter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def draw_cube(pub):
    """Mueve la tortuga"""
    msg = Twist()
    side_length = 2.0
    turn_angle = 1.57

    for _ in range(4):
        # Avanza
        msg.linear.x = side_length
        msg.angular.z = 0.0
        pub.publish(msg)
        rospy.sleep(1)

        msg.linear.x = 0.0
        msg.angular.z = turn_angle
        pub.publish(msg)
        rospy.sleep(1)

def draw_triangle(pub):
    """Mueve la tortuga para dibujar un triángulo equilátero"""
    msg = Twist()
    side_length = 2.0  # Longitud de cada lado del triángulo
    turn_angle = 2.09  # Ángulo de giro en radianes (120 grados = 2π/3)

    for _ in range(3):
        # Avanza en línea recta
        msg.linear.x = side_length
        msg.angular.z = 0.0
        pub.publish(msg)
        rospy.sleep(2)  # Tiempo para recorrer el lado

        # Gira para formar el ángulo del triángulo
        msg.linear.x = 0.0
        msg.angular.z = turn_angle
        pub.publish(msg)
        rospy.sleep(1)  # Tiempo para girar

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Controla la tortuga con las teclas:")
    print("  d -> Mover en X")
    print("  a -> Mover en -X")
    print("  w -> Mover en Y")
    print("  s -> Mover en -Y")
    print("  p -> Detenerse")
    print("  c -> Hacer un cuadrado")
    print("  t -> Hacer un triángulo")
    print("Presiona 'q' para salir.")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()
        
        if key == 'd':
            msg.linear.x = 2.0  # Avanza en X
            # msg.linear.y = 0.0 para asegurarse que no se mueva en y
        elif key == 'a':
            msg.linear.x = -2.0  # Avanza en -X
        elif key == 'w':
            msg.linear.y = 2.0  # Avanza en Y
        elif key == 's':
            msg.linear.y = -2.0  # Avanza en -Y
        elif key == 'p':
            msg.linear.x = 0.0
            msg.linear.y = 0.0  # Detiene el movimiento
        elif key == 'c': # hace un cuadrado
            draw_cube(pub)
        elif key == 't': # hace un triángulo
            draw_triangle(pub)
        elif key == 'q':  
            print("Saliendo...")
            break  # Sale del loop
        
        pub.publish(msg)

if __name__ == '__main__':
    main()
