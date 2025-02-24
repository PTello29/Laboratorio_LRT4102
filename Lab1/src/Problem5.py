# Crea un programa que solicite al usuario adivinar un número secreto. El programa debe generar
# un número aleatorio entre 1 y 10, y el usuario debe intentar adivinarlo. El programa debe
# proporcionar pistas si el número ingresado por el usuario es demasiado alto o bajo. El bucle while
# debe continuar hasta que el usuario adivine correctamente. Al final se debe imprimir en cuantos
# intentos el usuario logró adivinar el número.

import random
numero_secreto = random.randint(1, 10)
contador = 1

while True:
    numero = int(input("Adivina el número secreto (entre 1 y 10): "))
    if numero == numero_secreto:
        print(f"¡Felicidades! ¡Adivinaste el número secreto en {contador} intentos!")
        break
    elif numero < numero_secreto:
        print("El número es más alto. ¡Sigue intentando!")
        contador = contador + 1
    else:
        print("El número es más bajo. ¡Sigue intentando!")
        contador = contador + 1