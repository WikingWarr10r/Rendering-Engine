import pygame
import sys
import math
import numpy as np
from object import *
import random
import time

class Player:
    def __init__(self):
        self.position = np.array([0, 0, 5])
        self.orientation = np.array([0, math.pi, math.pi])

    def move(self, delta_position):
        self.position = (self.position + delta_position).astype(float)

    def rotate(self, delta_orientation):
        self.orientation += delta_orientation

def rotate_point(point, angles):
    rotation_matrix = np.array([
        [math.cos(angles[1])*math.cos(angles[2]), -math.cos(angles[0])*math.sin(angles[2]) + math.sin(angles[0])*math.sin(angles[1])*math.cos(angles[2]), math.sin(angles[0])*math.sin(angles[2]) + math.cos(angles[0])*math.sin(angles[1])*math.cos(angles[2])],
        [math.cos(angles[1])*math.sin(angles[2]), math.cos(angles[0])*math.cos(angles[2]) + math.sin(angles[0])*math.sin(angles[1])*math.sin(angles[2]), -math.sin(angles[0])*math.cos(angles[2]) + math.cos(angles[0])*math.sin(angles[1])*math.sin(angles[2])],
        [-math.sin(angles[1]), math.sin(angles[0])*math.cos(angles[1]), math.cos(angles[0])*math.cos(angles[1])]
    ])
    rotated_point = np.dot(rotation_matrix, np.array([point.x, point.y, point.z]))
    return Point(rotated_point[0], rotated_point[1], rotated_point[2])

def rotate_point_around_player(point, angles, player):
    translated_point = Point(point.x - player.position[0], point.y - player.position[1], point.z - player.position[2])

    rotation_matrix = np.array([
        [math.cos(angles[1])*math.cos(angles[2]), -math.cos(angles[0])*math.sin(angles[2]) + math.sin(angles[0])*math.sin(angles[1])*math.cos(angles[2]), math.sin(angles[0])*math.sin(angles[2]) + math.cos(angles[0])*math.sin(angles[1])*math.cos(angles[2])],
        [math.cos(angles[1])*math.sin(angles[2]), math.cos(angles[0])*math.cos(angles[2]) + math.sin(angles[0])*math.sin(angles[1])*math.sin(angles[2]), -math.sin(angles[0])*math.cos(angles[2]) + math.cos(angles[0])*math.sin(angles[1])*math.sin(angles[2])],
        [-math.sin(angles[1]), math.sin(angles[0])*math.cos(angles[1]), math.cos(angles[0])*math.cos(angles[1])]
    ])

    rotated_point = np.dot(rotation_matrix, np.array([translated_point.x, translated_point.y, translated_point.z]))
    rotated_point += player.position

    return Point(rotated_point[0], rotated_point[1], rotated_point[2])

p = Player()

pygame.init()

width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Renderer")

def perspective_projection_matrix(fov, aspect_ratio, near, far):
    f = 1 / math.tan(math.radians(fov / 2))
    projection_matrix = np.array([
        [f / aspect_ratio, 0, 0, 0],
        [0, f, 0, 0],
        [0, 0, (far + near) / (near - far), 2 * far * near / (near - far)],
        [0, 0, -1, 0]
    ])
    return projection_matrix

def project_point(player, point, projection_matrix):
    rotated_point = rotate_point(point, player.orientation)
    translated_point = Point(rotated_point.x + player.position[0], rotated_point.y + player.position[1], rotated_point.z + player.position[2])
    homogeneous_point = np.array([translated_point.x, translated_point.y, translated_point.z, 1])
    result = np.dot(projection_matrix, homogeneous_point)
    projected_point = result[:3] / result[3]
    return projected_point, result[2]

def add_projected_point_to_list(player, point_list, projection_matrix, points):
    for point in points:
        projected_point, depth = project_point(player, point, projection_matrix)
        x_screen = int((projected_point[0] + 1) * width / 2)
        y_screen = int((1 - projected_point[1]) * height / 2)
        point_list.append(((x_screen, y_screen), depth)) 

def render_triangle(screen, color, vertices):
    pygame.draw.polygon(screen, color, vertices)

def render_scene(screen, colors, objects, player):
    all_triangles = [triangle for obj in objects for triangle in obj]

    sorted_triangles = sorted(all_triangles, key=lambda t: sum(v[1] for v in t) / 3, reverse=False)

    i = 0
    for triangle in sorted_triangles:
        normal = calculate_triangle_normal(triangle)
        view_direction = np.array([0, 0, 1])

        if np.dot(normal, view_direction).any() < 0:
            continue

        if all(0 - width * 2 <= x < width * 2 and 0 - height * 2 <= y < height * 2 for (x, y), _ in triangle):
            object_index = next(index for index, obj in enumerate(objects) if triangle in obj)

            try:
                base_color = colors[object_index]
            except IndexError:
                base_color = (255, 0, 255)
            
            lighting_intensity = min(1.0, (i + 1) / len(sorted_triangles))
            color = (int(base_color[0] * lighting_intensity), int(base_color[1] * lighting_intensity), int(base_color[2] * lighting_intensity))
            render_triangle(screen, color, [vertex for vertex, _ in triangle])
            i += 1

def calculate_triangle_normal(triangle):
    v1, v2, v3 = map(lambda vertex: np.array(vertex[0]), triangle[:3])
    edge1 = v2 - v1
    edge2 = v3 - v1
    normal = np.cross(edge1, edge2)
    return normal / np.linalg.norm(normal)

fov = 60
aspect_ratio = width / height
near = 0.1
far = 100.0
projection_matrix = perspective_projection_matrix(fov, aspect_ratio, near, far)

model = parse_obj_file("car.obj")

delta_time = 0

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()

    movement_speed = 50 * delta_time
    rotation_speed = 10 * delta_time

    p.move(np.array([
        -movement_speed * (keys[pygame.K_a] - keys[pygame.K_d]),
        -movement_speed * (keys[pygame.K_e] - keys[pygame.K_q]),
        movement_speed * (keys[pygame.K_w] - keys[pygame.K_s])
    ]))

    p.rotate(np.array([
        rotation_speed * (keys[pygame.K_UP] - keys[pygame.K_DOWN]),
        rotation_speed * (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]),
        0
    ]))

    screen.fill((0, 0, 0))

    projected_model = [[] for _ in range(len(model))]

    for i, triangle in enumerate(model):
        add_projected_point_to_list(p, projected_model[i], projection_matrix, triangle)

    scene = [projected_model]

    start = time.time()
    render_scene(screen, [(255, 255, 255)], scene, p)
    end = time.time()
    delta_time = end - start

    pygame.display.set_caption(f"Renderer | FPS: {1 / (delta_time)}")

    pygame.display.flip()

pygame.quit()
sys.exit()
