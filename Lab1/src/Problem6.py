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
    matriz = [['o' for _ in range(n)] for _ in range(n)]
    matriz[0][0] = 'o'  # Punto inicial
    matriz[-1][-1] = 'o'  # Punto final
    
    # Generar obstáculos aleatorios
    celdas_disponibles = n * n - 2  # Excluir inicio y fin
    num_obstaculos = random.randint(1, celdas_disponibles // 2)
    
    obstaculos = set()
    while len(obstaculos) < num_obstaculos:
        fila = random.randint(0, n-1)
        col = random.randint(0, n-1)
        if (fila, col) not in [(0,0), (n-1, n-1)]:
            obstaculos.add((fila, col))
            matriz[fila][col] = 'X'
    
    return matriz

def buscar_camino(matriz):
    n = len(matriz)
    movimientos = [(-1,0), (0,1), (1,0), (0,-1)]  # Norte, Este, Sur, Oeste
    flechas = ['↑', '→', '↓', '←']
    
    cola = deque()
    visitado = set()
    
    # Iniciar en (0,0) con todas direcciones posibles
    for direccion_inicial in range(4):
        estado = (0, 0, direccion_inicial, [])
        cola.append(estado)
        visitado.add((0, 0, direccion_inicial))
    
    while cola:
        fila, col, direccion, camino = cola.popleft()
        
        # Verificar si llegó al destino
        if (fila, col) == (n-1, n-1):
            return camino + [(fila, col, direccion)]
        
        # Generar nuevos estados
        for accion in [-1, 1, 0]:  # Izq, Der, Avanzar
            nueva_dir = (direccion + accion) % 4 if accion != 0 else direccion
            
            if accion != 0:  # Giro
                nuevo_estado = (fila, col, nueva_dir, camino.copy())
                if (fila, col, nueva_dir) not in visitado:
                    visitado.add((fila, col, nueva_dir))
                    cola.append(nuevo_estado)
            else:  # Avance
                df, dc = movimientos[direccion]
                nf, nc = fila + df, col + dc
                if 0 <= nf < n and 0 <= nc < n and matriz[nf][nc] == 'o':
                    if (nf, nc, direccion) not in visitado:
                        nuevo_camino = camino + [(fila, col, direccion)]
                        visitado.add((nf, nc, direccion))
                        cola.append((nf, nc, direccion, nuevo_camino))
    
    return None

def imprimir_mapa(mapa):
    for fila in mapa:
        print(" ".join(fila))

def main():
    n = 5
    matriz = generar_matriz(n)
    camino = buscar_camino(matriz)
    
    if not camino:
        print("Imposible llegar al destino")
        return
    
    # Mapa original
    print("Mapa original:")
    imprimir_mapa(matriz)
    print("\nRuta seguida:")
    
    # Mapa de ruta
    mapa_ruta = [fila.copy() for fila in matriz]
    for paso in camino:
        f, c, d = paso
        if (f, c) != (n-1, n-1):  # No marcar el destino
            mapa_ruta[f][c] = '→' if d == 1 else '←' if d == 3 else '↑' if d == 0 else '↓'
    
    imprimir_mapa(mapa_ruta)

if __name__ == "__main__":
    main()