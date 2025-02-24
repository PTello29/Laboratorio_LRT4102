# Crea una lista llamada numeros que contenga al menos 10 números.
# Calcula el promedio de los números pares y el producto de los números impares.
# Imprime los resultados

numero = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
pares = []
impares = []

for numero in numero:
    if numero % 2 == 0:
        pares.append(numero)
    else:
        impares.append(numero)


promediopar = sum(pares) / len(pares)
promedioimpar = sum(impares) / len(impares)

print(f"El promedio de los pares es: {promediopar}")
print(f"El producto de los impares es: {promedioimpar}")