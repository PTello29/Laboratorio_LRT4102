# Reporte de laboratorio 3
En este reporte, se elaboraron dos actividades complementarias sobre el movimiento de una tortuga en el espacio de trabajo para llegar a una posición y orientación introducidas por el usuario.

## Primera parte
En la primera parte se desarrolló un código que una tortuga llegue a una posición y orintación proporcionadas por el usuario al matar a la tortuga y generar una nueva en la posición acordada. El código quedó de la siguiente forma:

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
### Explicación del código
Después de inicializar las librerías, se inicializa la clase encargada del movimiento de a tortuga con ROS, a la vez que se inicializa la función `__init__`:

```python
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
```
Primeramente se inicializa el nodo, que es el punto de entrada para interactuar con el sistema y permite la suscripción a tópicos y llamar servicios, el cual es el siguiente paso, ya que se suscribe al tópico `/turtle1/pose` que proprociona información sobre la posición y orientación de la tortuga. A su vez, se crea un publicador para el tópico `/turtle1/cmd_vel` para poder enviar comando de velocidad lineal y angular a la tortuga. Igualmente se inicilizan las variables de posición y la tasa de publicación de mensajes.

```python
    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta
```

La función `pose_callback` se ejecuta automáticamente cada vez que se recibe un mensaje en el tópico `/turtle1/pose` y registra la posición en cada actualización.

```python
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
```

Esta parte del código solo son funciones en las cuáles sale un mensaje para indicar que el usuario ingrese la coordenada x, y y orienatción theta que se quiera.

```python
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
```

Esta parte es el verdadero fucnionamiento del código, ya que es aquí donde, primeramente, se crea un ciclo `while not rospy.is_shutdown` que indica que mientras el ambiente siga corriendo, se realice todo lo que está dentro del ciclo que es: obtener las coordenadas deseadas por el usuario, matar a la tortuga en la posición anterior y generar una nueva tortuga en la posición y orientación proporcionadas.


```python
if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
```

Esta parte del código asegura que el programa se ejecute correctamente como un script independiente. Inicializa la clase encargada del control de la tortuga y ejecuta el método principal para interactuar con el usuario.

### Resultados

Para comprobar que el código funciona, se inicializa `roscore` y el launch `lab3.launch` dando el siguiente inicio

![Inicio del programa](/Lab3/Images/Captura%20de%20pantalla%20inicioLab3_1.png)

Al ingresar unas coordenadas y una orientación, la tortuga muere y aparece de manera efectiva en la posición acordada:

![Nueva posición de la tortuga](/Lab3/Images/Captura%20de%20pantalla%20movimiento.png)

Para verificar que este proceso puede seguir indefinidamente, se ingresan otras coordenadas y orientación para que una nueva tortuga aparezca en esa posición, lo cual resulta satisfactorio evidenciado en la siguiente imagen:

![Bucle infinito de posiciones](/Lab3/Images/Captura%20de%20pantalla%20movimiento2.png)

## Segunda parte

Para esta segunda parte, en lugar de hacer que la tortuga desaparezca y aparezca en la posición deseada, la tortuga se mueve hacia la posición y mediante un controlador proporcional se corregirá el error de la posición actual y la deseada hasta que llegué a esta última. Como última parte, igual se corregirá la orientación para que llegue a la requerida, quedando del código de la siguiente forma:

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
```

### Explicación del código

El código inicia de igual forma a la primera  parte, con el suscriptor y el publicador al igual que la función `pose_callback`. Después, se inicializa la función `normalize_angle`:

```python
    def normalize_angle(self, angle):
        # Normaliza un ángulo al rango [-π, π] y así evitar problemas de giros indefinidos
        while angle > 3.14159:
            angle -= 2 * 3.14159
        while angle < -3.14159:
            angle += 2 * 3.14159
        return angle
```

Esta función es necesaria para normalizar el ángulo y que la tortuga no gire indeterminadamente tratando de corregir el ángulo de orientación. De ahí, el código entra a la parte de calcular la posición y orientación usando el teorema de pitágoras para calcular la distancia euclidiana y `atan2` para calcular la orientación necesaria:

```python
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
```

Como se puede apreciar, se inicializan `Kp_linear` y `Kp_angular` que serán necesarias para el controlador proporcional que corrija el error de posición y orientación. Después, se calcula `distance` con el teorema de Pitágoras y `angle_to_goal` con `atan2`. Igualmente se pone `angular_error` para calcular el error de la orientación de la tortuga respecto a la orientación dada, todo esto normalizado.

Después de esto, se publican las velocidades para que la tortuga se mueva durante todo el ciclo `while`. Mientras la distancia sea mayor a 0.1, el ciclo seguirá hasta que esta condición se rompa, lo que significa que el error prácticamente es nuelo, es decir, la tortuga llega a la posición acordada. 

Después de esto, se inicializa otro `while` para ahora corregir la orientación de la tortuga a la proporcionada por el usuario.

El resto del código termina exactaente igual al anterior de la parte 1.

### Resultados

Para comporbar el código, se inicaliza nuevamente `roscore` y `lab3_2.launch`.

![Inicio del lab3_2](/Lab3/Images/Captura%20de%20pantalla%20inicioLab3_2.png)

Se insertan unas coordenadas y posición y se espera a que el programa termine

![Movimiento de la tortuga del lab3_2](/Lab3/Images/Captura%20de%20pantalla%202movimiento.png)

Para verificar que el código puede correr indefinidamente, se inserta unas nuevas coordenadas y se verifica que llegue a la posición introducida

![Movimiento indefinida](/Lab3/Images/Captura%20de%20pantalla%202movimiento%202.png)