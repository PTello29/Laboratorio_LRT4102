# Reporte de laboratorio 1
## *Introducción*

Python es un lenguaje de programación de alto nivel, interpretado y de propósito general. Es conocido por su sintaxis sencilla y legible, lo que lo hace ideal para principiantes, y al mismo tiempo es lo suficientemente potente para aplicaciones avanzadas en inteligencia artificial, desarrollo web, ciencia de datos, automatización, robótica y más.

### Características de Python

* Fácil de aprender y usar: Su sintaxis es clara y similar al lenguaje humano.
* Multiplataforma: Funciona en Windows, macOS, Linux, etc.
* Multiparadigma: Soporta programación orientada a objetos, estructurada y funcional.
* Gran comunidad y librerías: Tiene miles de módulos disponibles para todo tipo de tareas (por ejemplo, NumPy, Pandas, Matplotlib, Flask, TensorFlow).
* Código abierto: Es gratuito y de código abierto, mantenido por una comunidad global.

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
