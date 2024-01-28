class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

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

