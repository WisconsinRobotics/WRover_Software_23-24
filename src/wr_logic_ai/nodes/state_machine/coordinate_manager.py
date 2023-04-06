import json
from pathlib import Path

class Coordinate_manager:
    coordinates = []
    count = 0

    def __init__(self) -> None:
        pass

    @staticmethod
    def get_coordinate():
        return Coordinate_manager.coordinates[Coordinate_manager.count]
        
    def previous_line():
        Coordinate_manager.count = Coordinate_manager.count - 1

    @staticmethod
    def next_line():
        Coordinate_manager.count = Coordinate_manager.count + 1

    @staticmethod
    def read_coordinates_file():
        dirname = Path(__file__).parents[1]
        file_name = Path.joinpath(dirname, 'coordinates.json')
        file = open(file_name, 'r').read()
        Coordinate_manager.coordinates = json.loads(file)
   
    