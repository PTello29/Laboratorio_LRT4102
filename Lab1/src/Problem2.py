# Problema 2: Escribir un programa que pregunte al usuario por el número de horas trabajadas 
# y el costo por hora. Después debe mostrar por pantalla la paga que le corresponde.

# Solicitar al usuario el número de horas trabajadas y el costo por hora
horas = float(input("Introduce el número de horas trabajadas: "))
costo = float(input("Introduce el costo por hora: "))

# Calcular la paga que le corresponde
salario = horas * costo
print(f"Tu paga es de {salario} pesos")