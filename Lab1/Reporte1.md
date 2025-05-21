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

Resultado del programa:
```
Juan debe recibir $400 por su trabajo
María debe recibir $420 por su trabajo
Pedro debe recibir $360 por su trabajo
Ana debe recibir $450 por su trabajo
Luis debe recibir $400 por su trabajo
Sofía debe recibir $375 por su trabajo
```

### Problema 4.
Crea una lista llamada numeros que contenga al menos 10 números. Calcula el promedio de los números pares y el producto de los números impares e imprime los resultados.

```python
# Crea una lista llamada numeros que contenga al menos 10 números.
# Calcula el promedio de los números pares y el producto de los números impares.
# Imprime los resultados

# Lista de números
numero = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
pares = []
impares = []

# Clasificación de pares e impares
for numero in numero:
    if numero % 2 == 0:
        pares.append(numero)
    else:
        impares.append(numero)

# Cálculo del promedio de los pares y el producto de los impares
promediopar = sum(pares) / len(pares)
producto_impar = 1
for num in impares:
    producto_impar *= num

print(f"El promedio de los pares es: {promediopar}")
print(f"El producto de los impares es: {producto_impar}")
```

Este programa trabaja con una lista de números enteros y realiza dos operaciones distintas según si los números son pares o impares. SSe define una lista llamada numero que contiene 10 números enteros del 1 al 10 y se crean dos listas vacías, pares y impares. Luego, el código recorre cada número en la lista numero y usa el operador módulo (%) para verificar si es par o impar:

* Si el número es divisible por 2 (numero % 2 == 0), se agrega a la lista pares.
* Si no, se agrega a la lista impares.

Luego, calcula el promedio de los números pares sumando todos los elementos de pares y dividiendo por la cantidad de elementos y hace un ciclo for para los impares donde se van multiplicando. Después se imprime el promedio de los números pares y el resultado del producto de los impares.

Resultado del programa:
```
El promedio de los pares es: 6.0
El producto de los impares es: 945
```

### Problema 5
Crea un programa que solicite al usuario adivinar un número secreto. El programa debe generar un número aleatorio entre 1 y 10, y el usuario debe intentar adivinarlo. El programa debe proporcionar pistas si el número ingresado por el usuario es demasiado alto o bajo. El bucle while debe continuar hasta que el usuario adivine correctamente. Al final se debe imprimir en cuantos intentos el usuario logró adivinar el número.

```python
# Crea un programa que solicite al usuario adivinar un número secreto. El programa debe generar
# un número aleatorio entre 1 y 10, y el usuario debe intentar adivinarlo. El programa debe
# proporcionar pistas si el número ingresado por el usuario es demasiado alto o bajo. El bucle while
# debe continuar hasta que el usuario adivine correctamente. Al final se debe imprimir en cuantos
# intentos el usuario logró adivinar el número.

# Creación del númerp secreto
import random
numero_secreto = random.randint(1, 10)
contador = 1

# Adivinar el número secreto
while True:
    numero = int(input("Adivina el número secreto (entre 1 y 10): "))
    if numero == numero_secreto:
        # Imprimir el número de intentos
        print(f"¡Felicidades! ¡Adivinaste el número secreto en {contador} intentos!")
        break
    elif numero < numero_secreto:
        # Pista si el número es muy bajo
        print("El número es más alto. ¡Sigue intentando!")
        contador = contador + 1
    else:
        # Pista si el número es muy alto
        print("El número es más bajo. ¡Sigue intentando!")
        contador = contador + 1
```

El programa importa el módulo random y utiliza random.randint(1, 10) para crear un número entero aleatorio entre 1 y 10, que será el número secretoe igual se crea una variable contador para llevar el registro de cuántos intentos ha hecho el usuario, empezando en 1. El programa entra en un bucle infinito donde:

* Solicita al usuario que ingrese un número (su intento de adivinar el número secreto).
* Compara el número ingresado con el número secreto:
* Si el usuario adivina correctamente (numero == numero_secreto), imprime un mensaje de felicitación con el número de intentos y termina el bucle con break.
* Si el número ingresado es menor que el secreto, le indica al usuario que el número es más alto y le pide que siga intentando, y aumenta el contador de intentos.
* Si el número ingresado es mayor que el secreto, le indica que el número es más bajo y también aumenta el contador.

Cuando el usuario adivina el número, el programa termina y muestra cuántos intentos fueron necesarios.

Resultado del programa:
```
Adivina el número secreto (entre 1 y 10): 5
El número es más bajo. ¡Sigue intentando!
Adivina el número secreto (entre 1 y 10): 4
¡Felicidades! ¡Adivinaste el número secreto en 2 intentos!
```

### Problema 6
El programa debe generar una matriz de al menos 5x5. El robot inicia su camino en la posición (0,0) de la matriz y debe salir en la posición (4,4) o la máxima posición si se cambia el tamaño de matriz. El número y la posición de los obstáculos es aleatoria. El robot solo puede avanzar, girar a la izquierda o a la derecha para buscar un camino libre, en el eventual caso que el robot no pueda salir debe imprimir en pantalla “Imposible llegar al destino”. En caso de que el robot llegue a su destino final deberá imprimir el mapa, con los espacios libres y obstáculos de la siguiente forma: X obstáculo, o libre

# o o o X o
# o o o o o
# o o o o X
# o o o o o
# o X X X o

Deberá imprimir también la ruta que siguió y mostrar un segundo mapa con el “camino” seguido por el robot mediante flechas

```python
# El programa debe generar una matriz de al menos 5x5.
# El robot inicia su camino en la posición (0,0) de la matriz y debe salir en la posición (4,4) o la
# máxima posición si se cambia el tamaño de matriz.
# El numero y la posición de los obstáculos es aleatoria.
# El robot solo puede avanzar, girar a la izquierda o a la derecha para buscar un camino libre, en el
# eventual caso que el robot no pueda salir debe imprimir en pantalla “Imposible llegar al destino”
# En caso de que el robot llegue a su destino final deberá imprimir el mapa, con los espacios libres y
# obstáculos de la siguiente forma: X obstáculo, o libre
# o o o X o
# o o o o o
# o o o o X
# o o o o o
# o X X X o
# Deberá imprimir también la ruta que siguió.
# Mostrar un segundo mapa con el “camino” seguido por el robot mediante flechas

import random
from collections import deque

def generar_matriz(n=5):
    
    #Genera una matriz de tamaño n x n con obstáculos aleatorios ('X').
    #El punto inicial (0,0) y final (n-1,n-1) siempre están libres ('o').
    #Número de obstáculos: entre 1 y la mitad de las celdas disponibles.

    # Inicializar matriz con todas las celdas libres
    matriz = [['o' for _ in range(n)] for _ in range(n)]
    matriz[0][0] = 'o'  # Punto inicial (no se marca como obstáculo)
    matriz[-1][-1] = 'o'  # Punto final (no se marca como obstáculo)

    # Generar obstáculos aleatorios evitando inicio y fin
    celdas_disponibles = n * n - 2  # Excluir inicio y fin
    num_obstaculos = random.randint(1, celdas_disponibles // 2)  # Límite razonable

    obstaculos = set()
    while len(obstaculos) < num_obstaculos:
        fila = random.randint(0, n-1)
        col = random.randint(0, n-1)
        # Asegurar que no se coloquen obstáculos en inicio/fin
        if (fila, col) not in [(0,0), (n-1, n-1)]:
            obstaculos.add((fila, col))
            matriz[fila][col] = 'X'

    return matriz


def buscar_camino(matriz):
    
    # Busca un camino desde (0,0) hasta (n-1,n-1) usando BFS.
    # El robot puede girar o avanzar.
    # Devuelve: lista de pasos [(fila, columna, dirección)] si hay camino.
    # Direcciones: 0=Norte(↑), 1=Este(→), 2=Sur(↓), 3=Oeste(←).
    
    n = len(matriz)
    movimientos = [(-1,0), (0,1), (1,0), (0,-1)]  # Correspondencia direcciones: [N, E, S, O]
    flechas = ['↑', '→', '↓', '←']  # Símbolos para visualización

    cola = deque()  # Cola para BFS
    visitado = set()  # Estados visitados (fila, col, dirección)

    # Inicializar con todas las direcciones posibles en el inicio
    for direccion_inicial in range(4):
        estado_inicial = (0, 0, direccion_inicial, [])  # (fila, col, dir, camino)
        cola.append(estado_inicial)
        visitado.add((0, 0, direccion_inicial))

    while cola:
        fila, col, direccion, camino = cola.popleft()

        # Verificar si llegó al destino (esquina inferior derecha)
        if (fila, col) == (n-1, n-1):
            return camino + [(fila, col, direccion)]  # Camino completo

        # Explorar posibles acciones: -1=Girar izquierda, 1=Girar derecha, 0=Avanzar
        for accion in [-1, 1, 0]:
            if accion != 0:  # Giro (cambia dirección pero no posición)
                nueva_dir = (direccion + accion) % 4  # Cálculo modular para dirección
                nuevo_estado = (fila, col, nueva_dir, camino.copy())
                if (fila, col, nueva_dir) not in visitado:
                    visitado.add((fila, col, nueva_dir))
                    cola.append(nuevo_estado)
            else:  # Avanzar en la dirección actual
                df, dc = movimientos[direccion]  # Delta fila/columna
                nf, nc = fila + df, col + dc  # Nueva posición

                # Verificar límites de la matriz y celda libre
                if 0 <= nf < n and 0 <= nc < n and matriz[nf][nc] == 'o':
                    if (nf, nc, direccion) not in visitado:
                        # Registrar movimiento y añadir a la cola
                        nuevo_camino = camino + [(fila, col, direccion)]
                        visitado.add((nf, nc, direccion))
                        cola.append((nf, nc, direccion, nuevo_camino))

    return None  # No hay camino


def imprimir_mapa(mapa):
    """Imprime la matriz en formato grid con espacios entre caracteres."""
    for fila in mapa:
        print(" ".join(fila))


def main():
    n = 5  # Tamaño de la matriz (puede modificarse)
    matriz = generar_matriz(n)
    camino = buscar_camino(matriz)

    if not camino:
        print("Imposible llegar al destino")
        return

    # Mapa original con obstáculos
    print("Mapa original:")
    imprimir_mapa(matriz)

    # Mapa con la ruta seguida (flechas)
    print("\nRuta seguida:")
    mapa_ruta = [fila.copy() for fila in matriz]  # Copia para no modificar original

    for paso in camino:
        f, c, d = paso
        # No sobreescribir el destino final
        if (f, c) != (n-1, n-1):
            # Asignar símbolos según dirección: ↑ → ↓ ←
            mapa_ruta[f][c] = '→' if d == 1 else '←' if d == 3 else '↑' if d == 0 else '↓'

    imprimir_mapa(mapa_ruta)


if __name__ == "__main__":
    main()
```

El programa crea una matriz cuadrada de tamaño 5x5 (o cualquier otro tamaño que se defina) que representa un espacio por donde un robot debe desplazarse. En esta matriz, algunas posiciones contienen obstáculos representados con una "X", colocados aleatoriamente, mientras que el resto son espacios libres marcados con "o". El robot comienza su recorrido en la esquina superior izquierda y debe llegar hasta la esquina inferior derecha.

Para encontrar un camino desde el inicio hasta el destino, el programa utiliza un algoritmo de búsqueda en anchura (BFS), que toma en cuenta no solo la posición del robot, sino también la dirección hacia la que está mirando (norte, este, sur u oeste). Esto es importante porque el robot no puede moverse libremente en cualquier dirección; solo puede avanzar en la dirección en la que está orientado o girar a la izquierda o a la derecha para cambiar su orientación. La búsqueda explora todas las combinaciones posibles de movimientos y giros para hallar un camino viable que evite los obstáculos.

Si se encuentra un camino, el programa imprime el mapa original con los obstáculos y luego muestra un segundo mapa donde se indica la ruta que siguió el robot con flechas que apuntan en la dirección de cada paso. Esto permite visualizar claramente cómo el robot navegó por la matriz. En caso de que no exista un camino posible, el programa muestra un mensaje que indica que es imposible llegar al destino. De esta manera, el código simula el proceso de navegación y toma de decisiones de un robot en un entorno con obstáculos y limitaciones de movimiento.

Primer resultado del programa: 
```
Mapa original:
o o o o o
o o o o o
o o o o o
o o o o o
X o o o o

Ruta seguida:
→ → → → ↓
o o o o ↓
o o o o ↓
o o o o ↓
X o o o o
```

Segundo resultado del programa: 
```
Mapa original:
o o o X o
o X o o X
X X X o o
X o X o o
X X o X o

Ruta seguida:
→ → ↓ X o
o X → ↓ X
X X X → ↓
X o X o ↓
X X o X o
```

Tercer resultado del programa:
```
Imposible llegar al destino
```

## *Conclusión*
En este primer laboratorio, se logró familiarizarse con la programación en Python y con el entorno de ROS, fundamentales para el desarrollo de aplicaciones robóticas. A través de la resolución de problemas prácticos, se consolidaron conceptos básicos de programación, como el manejo de entradas y salidas, estructuras de datos, ciclos y condiciones, además de la implementación de algoritmos sencillos para operaciones matemáticas y simulaciones de comportamiento.

Los ejercicios permitieron comprender cómo usar Python para manipular datos y tomar decisiones lógicas, habilidades esenciales para la programación de robots y sistemas automatizados. También se abordó el diseño de un algoritmo para la navegación de un robot en un entorno con obstáculos, lo que conecta la teoría con aplicaciones reales de robótica.

En resumen, este laboratorio sentó las bases para el uso de Python y ROS en proyectos más complejos, destacando la importancia de la modularidad, la lógica de programación y la capacidad para simular comportamientos antes de implementarlos en hardware real. El aprendizaje obtenido facilitará el desarrollo futuro de sistemas robóticos más sofisticados y la integración de software con componentes físicos.

## *Referencias*
* 1. Python Software Foundation. (s.f.). Documentación oficial de Python. Recuperado de https://docs.python.org/es/3/

* 2. "Programming Robots with ROS” by Morgan Quigley, Brian Gerkey, and William D. Smart.

* 3. “Learning ROS for Robotics Programming” by Aaron Martinez and Enrique Fernandez.
