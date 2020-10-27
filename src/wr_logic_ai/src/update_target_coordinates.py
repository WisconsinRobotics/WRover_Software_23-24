#!/usr/bin/env python

while True:
    print("=============================================")
    print("If a coordinate value is missing the program will exit")
    lat = input("Input lat: ")
    lng = input("Input long: ")
    if lat is None or lng is None:
        break
    coordinates_f = open('coordinates.txt', 'w')
    coordinates_f.write('{"lat":' + str(lat) + ' , "long":' + str(lng) + '}\n')
    coordinates_f.close()
