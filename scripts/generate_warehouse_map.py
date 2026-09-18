#!/usr/bin/env python3
"""Generate a Nav2 occupancy map directly from the Gazebo warehouse SDF."""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SDF = ROOT / 'src/agv_worlds/worlds/warehouse_40x50.sdf'
OUTPUT = ROOT / 'src/agv_navigation/maps/warehouse.pgm'
RESOLUTION = 0.05
MIN_X, MIN_Y = -20.0, -25.0
MAX_X, MAX_Y = 20.0, 25.0


def pose_xyz(element):
    pose = element.find('pose')
    if pose is None or not pose.text:
        return 0.0, 0.0, 0.0
    values = [float(value) for value in pose.text.split()]
    return values[0], values[1], values[2]


def collision_boxes(root):
    boxes = []
    for model in root.findall('model'):
        model_x, model_y, model_z = pose_xyz(model)
        if model.get('name') == 'floor':
            continue
        for link in model.findall('link'):
            link_x, link_y, link_z = pose_xyz(link)
            for collision in link.findall('collision'):
                size_node = collision.find('geometry/box/size')
                if size_node is None or not size_node.text:
                    continue
                size_x, size_y, size_z = map(float, size_node.text.split())
                x, y, z = pose_xyz(collision)
                if model_z + link_z + z + size_z / 2.0 <= 0.1:
                    continue
                center_x = model_x + link_x + x
                center_y = model_y + link_y + y
                boxes.append((
                    center_x - size_x / 2.0,
                    center_x + size_x / 2.0,
                    center_y - size_y / 2.0,
                    center_y + size_y / 2.0,
                ))
    return boxes


def main():
    root = ET.parse(SDF).getroot().find('world')
    boxes = collision_boxes(root)
    width = round((MAX_X - MIN_X) / RESOLUTION)
    height = round((MAX_Y - MIN_Y) / RESOLUTION)
    pixels = bytearray([254]) * (width * height)

    for row in range(height):
        y = MAX_Y - (row + 0.5) * RESOLUTION
        for column in range(width):
            x = MIN_X + (column + 0.5) * RESOLUTION
            if any(x0 <= x <= x1 and y0 <= y <= y1
                   for x0, x1, y0, y1 in boxes):
                pixels[row * width + column] = 0

    with OUTPUT.open('wb') as stream:
        stream.write(
            f'P5\n# Generated from {SDF.name}\n{width} {height}\n255\n'.encode())
        stream.write(pixels)
    print(f'Generated {OUTPUT}: {width}x{height}, {len(boxes)} obstacles')


if __name__ == '__main__':
    main()
