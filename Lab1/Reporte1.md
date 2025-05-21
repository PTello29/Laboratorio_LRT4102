# Reporte de laboratorio 1
## *Introducción*

### Python
Python es un lenguaje de programación de alto nivel, interpretado y de propósito general. Es conocido por su sintaxis sencilla y legible, lo que lo hace ideal para principiantes, y al mismo tiempo es lo suficientemente potente para aplicaciones avanzadas en inteligencia artificial, desarrollo web, ciencia de datos, automatización, robótica y más.

#### Características de Python

* Fácil de aprender y usar: Su sintaxis es clara y similar al lenguaje humano.
* Multiplataforma: Funciona en Windows, macOS, Linux, etc.
* Multiparadigma: Soporta programación orientada a objetos, estructurada y funcional.
* Gran comunidad y librerías: Tiene miles de módulos disponibles para todo tipo de tareas (por ejemplo, NumPy, Pandas, Matplotlib, Flask, TensorFlow).
* Código abierto: Es gratuito y de código abierto, mantenido por una comunidad global.

### ROS
ROS, o Sistema Operativo de Robots, es un marco flexible para escribir software de robots. No es un sistema operativo en el sentido tradicional, sino un conjunto de bibliotecas y herramientas de software que ayudan a los desarrolladores a crear aplicaciones robóticas de forma eficiente.

* Middleware para robótica: ROS proporciona una capa de comunicación estandarizada que permite que las diferentes partes de un robot (p. ej., sensores, actuadores, algoritmos de control) se comuniquen fluidamente.

* Modular y distribuido: ROS utiliza nodos (procesos independientes) que se comunican mediante mensajes, lo que permite la computación distribuida en múltiples computadoras si es necesario.

* Herramientas y bibliotecas: ROS ofrece herramientas para simulación, visualización, depuración y abstracción de hardware, lo que facilita el desarrollo de sistemas robóticos complejos.

#### ¿Por qué usar ROS?
* Código abierto y basado en la comunidad: ROS cuenta con una gran comunidad global, documentación extensa y numerosos paquetes reutilizables.

* Abstracción de hardware: ROS permite utilizar el mismo código con diferentes componentes de hardware de robots.

* Simulación: Se integra bien con simuladores como Gazebo para probar algoritmos sin robots físicos.

* Escalabilidad: Adecuado para robots simples y sistemas complejos con múltiples robots.

## *Prácticas*
Como parte del desarrollo de la clase, se hicieron  una serie de prácticas las cuales tienen el objetivo de familizarizarse con el entorno de python y de ROS:

### Problema 1
Escribir un programa que lea un entero positivo “n” introducido por el usuario y después muestre en pantalla la suma de todos los enteros desde 1 hasta n . La suma de los primeros enteros positivos puede ser calculada de la siguiente forma:
suma = $\frac{n(n+1)}{2}$

Este problema se resolvió con el siguiente código: 

```python
# Problema 1: Escribe un programa que lea un entero positivo, n, introducido 
# por el usuario y después calcule la suma de todos los enteros desde 1 hasta n.

# Solicitar al usuario un entero positivo
n = int(input("Introduce un entero positivo: "))

# Calcular la suma de todos los enteros desde 1 hasta n
if n > 0:
    suma = sum(range(1, n + 1))
    print(f"La suma de todos los enteros desde 1 hasta {n} es: {suma}")
else:
    print("Por favor, introduce un número entero positivo.")
```

El programa lee un número entero positivo introducido por el usuario y luego calcula la suma de todos los números enteros desde 1 hasta ese número. L ametodología parte de que primero se le pide al usuario que introduzca un número para después verificar si ese número es mayor que cero (es decir, si es un entero positivo). Si el número es válido, se calcula la suma de todos los enteros desde 1 hasta ese número. Si el número no es positivo, el programa muestra un mensaje pidiendo un número válido.

Resultado del programa: 
```
Introduce un entero positivo:
5
La suma de todos los enteros desde 1 hasta 5 es: 15
```

### Problema 2
Escribir un programa que pregunte al usuario por el número de horas trabajadas y el costo por hora. Después debe mostrar por pantalla la paga que le corresponde.

```pyhton
# Problema 2: Escribir un programa que pregunte al usuario por el número de horas trabajadas 
# y el costo por hora. Después debe mostrar por pantalla la paga que le corresponde.

# Solicitar al usuario el número de horas trabajadas y el costo por hora
horas = float(input("Introduce el número de horas trabajadas: "))
costo = float(input("Introduce el costo por hora: "))

# Calcular la paga que le corresponde
salario = horas * costo
print(f"Tu paga es de {salario} pesos")
```

El programa primero pide al usuario que ingrese dos valores: el número de horas trabajadas (puede ser un número decimal, por eso se usa float para permitir valores como 7.5 horas) y el costo o tarifa por hora (también como número decimal para mayor precisión). Después hace el cálculo del salario multiplicando las horas trabajadas por el costo por hora para obtener el salario total y finalmente, imprime el resultado mostrando el salario que le corresponde al usuario, utilizando un mensaje claro y formateado.

Resultado del programa: 
```
Introduce el número de horas trabajadas: 10
Introduce el costo por hora: 50
Tu paga es de 500.0 pesos
```

### Problema 3
Crea una lista de nombre + sueldo por hora + horas trabajadas de al menos seis operadores. Imprime el nombre y el sueldo a pagar de cada operador.

```python
# Problema 3: Crea una lista de nombre + sueldo por hora + horas trabajadas de 
# al menos seis operadores. Imprime el nombre y el sueldo a pagar de cada operador.

# Lista de operadores
operadores = [
    ["Juan", 10, 40],
    ["María", 12, 35],
    ["Pedro", 8, 45],
    ["Ana", 15, 30],
    ["Luis", 20, 20],
    ["Sofía", 25, 15]
]   

# Imprimir nombre y sueldo a pagar de cada operador
for operador in operadores:
    nombre = operador[0]
    sueldo = operador[1] * operador[2]
    print(f"{nombre} debe recibir ${sueldo} por su trabajo")
```
Este programa maneja una lista de operadores, donde cada operador está representado por una sublista que contiene tres elementos: su nombre, su sueldo por hora y las horas que ha trabajado. Se crea una lista llamada operadores con seis sublistas, cada una correspondiente a un operador diferente. Cada sublista contiene:

* El nombre del operador (una cadena de texto).
* El sueldo por hora (un número entero o decimal).
* Las horas trabajadas (un número entero o decimal).

El programa recorre cada sublista dentro de operadores usando un ciclo for. Para cada operador:

* Extrae el nombre.
* Calcula el sueldo total multiplicando el sueldo por hora por las horas trabajadas.
* Imprime un mensaje que indica cuánto debe recibir ese operador por su trabajo.
