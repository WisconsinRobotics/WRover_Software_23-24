import json
from pathlib import Path

class CoordinateManager:
    coordinates = []
    ptr = 0
    maxPtr = 0

    def __init__(self) -> None:
        pass

    @staticmethod
    def get_coordinate():
        print(CoordinateManager.ptr)
        return CoordinateManager.coordinates[CoordinateManager.ptr]
        
    @staticmethod
    def previous_line():
        print(CoordinateManager.ptr)
        CoordinateManager.ptr = CoordinateManager.ptr - 1
        print(CoordinateManager.ptr)

    @staticmethod
    def next_line():
        CoordinateManager.ptr = CoordinateManager.ptr + 1
        CoordinateManager.maxPtr = max(CoordinateManager.ptr, CoordinateManager.maxPtr)

    @staticmethod
    def short_range_complete() -> bool:
        return CoordinateManager.ptr < CoordinateManager.maxPtr

    @staticmethod
    def read_coordinates_file():
        dirname = Path(__file__).parents[1]
        file_name = Path.joinpath(dirname, 'coordinates.json')
        file = open(file_name, 'r').read()
        CoordinateManager.coordinates = json.loads(file)
   
    