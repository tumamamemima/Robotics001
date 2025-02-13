import math

radio = float(input("Enter the radius of the circle: "))

perimeter = 2 * math.pi * radio
area = math.pi * radio ** 2

print(f"The perimeter of the circle is: {perimeter}")
print(f"The area of the circle is: {area}")

if perimeter > 20:
    print("The perimeter is greater than 20")
else:
    print("The perimeter is less than 20")