import math


inputData = []

for x in range(270):
    distance = 20*math.sin(((math.pi)*x)/270)
    if distance > 12:
        distance = 12
    #inputData.append(str('%.5f' % distance) + " is the distance at angle " + str(x+1))
    inputData.append(x)
for y in range(len(inputData)):
  	print(inputData[y])
