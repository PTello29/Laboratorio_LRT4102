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
