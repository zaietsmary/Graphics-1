import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

try:
    x_center, y_center, z_center = map(float, input("Введіть X, Y, Z центру квадрата: ").split())
    side_length = float(input("Введіть довжину сторони квадрата: "))
    if side_length <= 0:
        raise ValueError("Довжина сторони квадрата повинна бути більше 0!")
    x1, y1, z1 = map(float, input("Введіть X, Y, Z першої точки прямої: ").split())
    x2, y2, z2 = map(float, input("Введіть X, Y, Z другої точки прямої: ").split())
except ValueError as e:
    print(f"Помилка: {e}. Будь ласка, введіть числа коректно.")
    exit()

half_side = side_length / 2

square_vertices = np.array([
    [-half_side, -half_side, 0], [half_side, -half_side, 0],
    [half_side, half_side, 0], [-half_side, half_side, 0],
    [-half_side, -half_side, 0]
])

def perpendicular_projection(p, p1, p2):
    line_vector = np.array([p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]])
    point_vector = np.array([p[0] - p1[0], p[1] - p1[1], p[2] - p1[2]])
    
    t = np.dot(point_vector, line_vector) / np.dot(line_vector, line_vector)
    
    projection = np.array([p1[0] + t * line_vector[0], 
                           p1[1] + t * line_vector[1], 
                           p1[2] + t * line_vector[2]])
    return projection

def rotate_square(vertices, angle):
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    return np.dot(vertices, rotation_matrix)

def line_extension(p1, p2, length=20):
    direction = np.array([p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]])
    unit_direction = direction / np.linalg.norm(direction)
    
    p1_extended = p1 - unit_direction * length
    p2_extended = p2 + unit_direction * length
    return p1_extended, p2_extended

def distance_point_to_line(p, p1, p2):
    line_vector = np.array([p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]])
    point_vector = np.array([p[0] - p1[0], p[1] - p1[1], p[2] - p1[2]])
    
    cross_product = np.cross(point_vector, line_vector)
    distance = np.linalg.norm(cross_product) / np.linalg.norm(line_vector)
    return distance

angle = 0
center = np.array([x_center, y_center, z_center])
move_step = 0.05
rotation_step = np.radians(3)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot([-20, 20], [0, 0], [0, 0], 'r', label='X-axis')
ax.plot([0, 0], [-20, 20], [0, 0], 'g', label='Y-axis')
ax.plot([0, 0], [0, 0], [-20, 20], 'b', label='Z-axis')

p1_extended, p2_extended = line_extension(np.array([x1, y1, z1]), np.array([x2, y2, z2]))

ax.plot([p1_extended[0], p2_extended[0]], [p1_extended[1], p2_extended[1]], [p1_extended[2], p2_extended[2]], 'k-', linewidth=2, label='Extended Line')

perimeter_points = square_vertices 
is_intersecting = False

for point in perimeter_points:
    distance = distance_point_to_line(point, np.array([x1, y1, z1]), np.array([x2, y2, z2]))
    if distance <= half_side:
        is_intersecting = True
        break

if is_intersecting:
    print("Квадрат перетинає пряму. Не буду малювати квадрат.")
else:
    print("Квадрат не перетинає пряму. Малюю квадрат.")

square_lines = []
stop_moving = False

def update(frame):
    global angle, center, square_lines, stop_moving
    
    projection = perpendicular_projection(center, np.array([x1, y1, z1]), np.array([x2, y2, z2]))
    
    ax.plot([x_center, projection[0]], [y_center, projection[1]], [z_center, projection[2]], '--', color='purple', label='Perpendicular')
    
    if not stop_moving:
        direction = np.array([projection[0] - center[0], projection[1] - center[1], projection[2] - center[2]])
        distance = np.linalg.norm(direction)
        if distance > move_step:
            direction = direction / distance
            center += direction * move_step
        else:
            center[:] = projection
            stop_moving = True
    
    if stop_moving:
        return
    
    angle += rotation_step
    rotated_square = rotate_square(square_vertices, angle)
    moved_square = rotated_square + center
    
    for line in square_lines:
        line.remove()
    square_lines.clear()
    
    for i in range(4):
        line = ax.plot([moved_square[i, 0], moved_square[i + 1, 0]], 
                       [moved_square[i, 1], moved_square[i + 1, 1]], 
                       [moved_square[i, 2], moved_square[i + 1, 2]], 'brown')
        square_lines.append(line[0])
    
    return square_lines

if not is_intersecting:
    ani = animation.FuncAnimation(fig, update, frames=100, interval=50)
    plt.show()

input("Натисніть Enter для виходу...")
