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
        return CoordinateManager.coordinates[CoordinateManager.ptr]
        
    @staticmethod
    def previous_line():
        CoordinateManager.ptr = max(0,CoordinateManager.ptr - 1)

    @staticmethod
    def next_line() -> bool:
        CoordinateManager.ptr = CoordinateManager.ptr + 1
        return CoordinateManager.ptr >= len(CoordinateManager.coordinates)

    @staticmethod
    def short_range_complete() -> bool:
        return CoordinateManager.ptr < CoordinateManager.maxPtr

    @staticmethod
    def read_coordinates_file():
        dirname = Path(__file__).parents[0]
        file_name = Path.joinpath(dirname, 'coordinates.json')
        file = open(file_name, 'r').read()
        CoordinateManager.coordinates = json.loads(file)
   
    