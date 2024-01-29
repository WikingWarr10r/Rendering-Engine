import numpy as np

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        if isinstance(other, np.ndarray):
            return Point(self.x + other[0], self.y + other[1], self.z + other[2])
        elif isinstance(other, Point):
            return Point(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            raise TypeError("Unsupported type for addition")
        
    def __iter__(self):
        return iter([self.x, self.y, self.z])

def parse_obj_file(file_path):
    vertices = []
    triangles = []

    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('v '):
                x, y, z = map(float, line.split()[1:])
                vertices.append(Point(x, y, z))
            elif line.startswith('f '):
                indices = [int(vertex.split('/')[0]) - 1 for vertex in line.split()[1:]]
                triangle = [vertices[index] for index in indices]
                triangles.append(triangle)

    return triangles

