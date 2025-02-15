# Problema 2: Escribir un programa que pregunte al usuario por el número de horas trabajadas 
# y el costo por hora. Después debe mostrar por pantalla la paga que le corresponde.

horas = float(input("Introduce el número de horas trabajadas: "))
costo = float(input("Introduce el costo por hora: "))

salario = horas * costo

print(f"Tu paga es de {salario} pesos")