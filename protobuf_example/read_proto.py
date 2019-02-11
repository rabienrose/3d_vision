#! /usr/bin/python

import chamo_map_pb2
import sys

map_data = chamo_map_pb2.MapData()
f = open(sys.argv[1], "rb")
map_data.ParseFromString(f.read())
print(map_data)
