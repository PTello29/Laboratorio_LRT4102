# Reporte de laboratorio 2
En este reporte se hicieron tres prácticas. La primera siendo sobre el funcionamiento del talker.py y el listener.py; la segunda sobre el movimiento de la tortuga de turtlesim con base en el teclado, a su vez que realiza un movimiento en forma de cuadrado y de triángulo mediante una tecla, y el tercero son tres archivos con movimiento de la tortuga hacia una posición mediante controladores proporcionales, PD y PID.

## Práctica 1. Listener y Talker

En esta práctica se analizaron dos códigos: el listener y el talker. El talker es un código que manda un "hello world" cada ciertos microsegundos mediante el rospy.Publisher. El listener es un código que se suscribe al "chatter" que es el publicador del código del talker, por lo que todo lo que mencione el talker, el listener los escuchará de la siguiente forma:

