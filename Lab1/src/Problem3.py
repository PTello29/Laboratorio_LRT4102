# Problema 3: Crea una lista de nombre + sueldo por hora + horas trabajadas de 
# al menos seis operadores. Imprime el nombre y el sueldo a pagar de cada operador.

preguntar = int(input("¿Cuántos operadores deseas agregar? "))

for i in range(preguntar):
    valores = input("Introduce el nombre del operador, horas trabajadas y sueldo por hora: ")
    lista = valores.split()

print(lista)
    
