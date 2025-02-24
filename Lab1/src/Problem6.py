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

matriz = [["o", "o", "o", "o", "o"],
          ["o", "o", "o", "o", "o"],
          ["o", "o", "o", "o", "o"],
          ["o", "o", "o", "o", "o"],
          ["o", "o", "o", "o", "o"],]

obstaculo = int(input("¿Cuántos obstáculos quieres poner?"))

for i in range(obstaculo):
    fila = random.randint(0, 4)
    columna = random.randint(0, 4)
    matriz[fila][columna] = "X"

for fila in matriz:
    print(" ".join(fila))

posicion = [0, 0]
ruta = []
ruta.append(posicion.copy())

