import math

print("[", end = "")
for i in range(48):
    angle = (i * 0 + (48-i) * 2 * math.pi) / 48

    #dist_percent = generator.random()
    #dist = 0.03 * dist_percent + 0.06 * (1-dist_percent)

    dist = 0.1

    x = dist * math.cos(angle)
    y = dist * math.sin(angle)
    print("(" + str(x) + "," + str(y) + ")" + ",", end= "")
print("]")