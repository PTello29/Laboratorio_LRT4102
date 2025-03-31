# Reporte de laboratorio 2
En este reporte se hicieron tres prácticas. La primera siendo sobre el funcionamiento del talker.py y el listener.py; la segunda sobre el movimiento de la tortuga de turtlesim con base en el teclado, a su vez que realiza un movimiento en forma de cuadrado y de triángulo mediante una tecla, y el tercero son tres archivos con movimiento de la tortuga hacia una posición mediante controladores proporcionales, PD y PID.

## Práctica 1. Listener y Talker

En esta práctica se analizaron dos códigos: el listener y el talker. El talker es un código que manda un "hello world" cada ciertos microsegundos mediante el rospy.Publisher. 

```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
El listener es un código que se suscribe al "chatter" que es el publicador del código del talker, por lo que todo lo que mencione el talker, el listener los escuchará de la siguiente forma:

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

![Listener funcionando](/Lab2/Images/Captura%20de%20pantalla%201.png)

Como se aprecia, el listener se inicializó pero no sucede nada, esto debido a que el talker aún no se ha inicializado, por lo que el listener no tiene nada que "escuchar".

![Talker funcionando](/Lab2/Images/Captura%20de%20pantalla%202.png)

Ahora como se aprecia, el listener escucha todo lo que el talker manda. Para esta práctica específicamente se realizó el listener con un argumento de anonymous=True, para que rospy escoja un nombre único para este listener, por lo que puedan existir varios listeners escuchando al mismo talker.

## Práctica 2. Control de tortuga con teclado

Para la segunda parte de este reporte, se realizó un código que hace que la tortuga de turtlesim pueda controlarse mediante teclas del teclado, especicamente las teclas W, A, S y D. Este código hace que la tortuga se mueva dos unidades en x si se aprieta D, dos unidades en y si se aprieta W, menos dos unidades en x si se aprieta A y menos dos unidades en y si se aprieta en S. A su vez, igual implementa 4 teclas más: q, c, t y p. La letra q sirve para salirse del programa; p sirve para que la tortuga se deje de mover de inmediato; c para que la tortuga dibuje un cuadrado dónde esté y t para que la tortuga dibuje un triángulo.

```python
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
```

![Inicio del programa](/Lab2/Images/Captura%20de%20pantalla%203.png)

Así es como se ve la interfaz de este código.

![Ejemplo con WASD](/Lab2/Images/Captura%20de%20pantalla%204.png)

Ejemplo de movimiento con las teclas W, A, S, D.

![Ejemplo con WASD](/Lab2/Images/Captura%20de%20pantalla%205.png)

Ejemplo del cuadrado.

![Ejemplo con WASD](/Lab2/Images/Captura%20de%20pantalla%206.png)

Ejemplo de triángulo.