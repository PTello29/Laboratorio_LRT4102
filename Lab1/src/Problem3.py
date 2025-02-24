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