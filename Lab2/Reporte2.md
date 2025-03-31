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

![Ejemplo del cuadrado](/Lab2/Images/Captura%20de%20pantalla%205.png)

Ejemplo del cuadrado.

![Ejemplo del triángulo](/Lab2/Images/Captura%20de%20pantalla%206.png)

Ejemplo de triángulo.

## Práctica 3. Controlador P, PD y PID

Para esta última parte del reporte, se realizaron 3 controladores que harán que la tortuga se mueva a unas coordenadas dadas. Estso controladores son un proporcional, un proporcional derivativo y un proporcional integral derivativo. 

Código del controlador proporcional:
``` python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
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
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_position(self, desired_x, desired_y):
        # Constantes de proporcionalidad del controlador (ajustables)
        Kp_linear = 1
        Kp_angular = 4

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
            
            # Calcular las velocidades lineal y angular
            vel_linear = Kp_linear * distance
            vel_angular = Kp_angular * error_theta
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_linear
            twist_msg.angular.z = vel_angular
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y las variables de velocidad en la terminal
            rospy.loginfo("Posición actual: (%f, %f), Error: (%f, %f), Vel_linear: %f, Vel_angular: %f", self.current_x, self.current_y, error_x, error_y, vel_linear, vel_angular)
            
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
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
```

Código del controlador derivativo:

``` python
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
```

Código del controlador integral derivativo: 

``` python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt

class MoveTurtlePIDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_pid')
        
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
        self.error_accumulation_distance = 0
        self.error_accumulation_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_position(self, desired_x, desired_y):
        # Constantes de proporcionalidad, integral y derivativa del controlador (ajustables)
        Kp_linear = 1
        Ki_linear = 0.01
        Kd_linear = 0.1
        Kp_angular = 4
        Ki_angular = 0.01
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
            
            # Acumular los errores para el término integral
            self.error_accumulation_distance += distance
            self.error_accumulation_theta += error_theta
            
            # Calcular las velocidades lineal y angular con términos PID
            vel_linear = (Kp_linear * distance +
                          Ki_linear * self.error_accumulation_distance +
                          Kd_linear * (distance - self.last_error_distance))
            
            vel_angular = (Kp_angular * error_theta +
                           Ki_angular * self.error_accumulation_theta +
                           Kd_angular * (error_theta - self.last_error_theta))
            
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
        move_turtle_pid = MoveTurtlePIDControl()
        move_turtle_pid.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
```
Para los tres códigos, la tortuga logra llegar a la posición deseada (x, y) la cual es proporcionada por el usuario pero no logra verse correctamente las diferencias entre cada controlador, por lo que se uso Plot Juggler, que es un programa con el que podemos graficar datos de rospy, y en este programa podremos ver las diferencias entre cada controlador:

Controlador proporcional:

![Gráfico del controlador proporcional](/Lab2/Images/Captura%20de%20pantalla%20PC.png)

Controlador PD:

![Gráfico del controlador PD](/Lab2/Images/Captura%20de%20pantalla%20PDC.png)

Controlador PID:

![Gráfico del controlador PID](/Lab2/Images/Captura%20de%20pantalla%20PIDC.png)

