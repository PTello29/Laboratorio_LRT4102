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