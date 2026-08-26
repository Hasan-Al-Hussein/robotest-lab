#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generate the identity-aligned RoboTest Lab occupancy map from SDF collisions."""

from __future__ import annotations

import argparse
from collections.abc import Iterable
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

RESOLUTION_M = 0.05
ORIGIN_X_M = -6.2
ORIGIN_Y_M = -6.2
WIDTH_CELLS = 248
HEIGHT_CELLS = 248
FREE = 254
OCCUPIED = 0


@dataclass(frozen=True)
class Pose2D:
    """Planar part of an SDF pose."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Shape:
    """A supported static collision projected onto the map plane."""

    name: str
    kind: str
    pose: Pose2D
    size_x: float
    size_y: float

    def contains(self, x: float, y: float) -> bool:
        """Return whether a world-coordinate point is inside the footprint."""
        dx = x - self.pose.x
        dy = y - self.pose.y
        cos_yaw = math.cos(self.pose.yaw)
        sin_yaw = math.sin(self.pose.yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        if self.kind == 'box':
            return abs(local_x) <= self.size_x / 2.0 and abs(local_y) <= self.size_y / 2.0
        if self.kind == 'cylinder':
            return local_x * local_x + local_y * local_y <= self.size_x * self.size_x
        raise ValueError(f'unsupported shape kind: {self.kind}')


def parse_pose(text: str | None) -> Pose2D:
    """Parse the planar coordinates of an SDF pose string."""
    if not text or not text.strip():
        return Pose2D()
    values = [float(value) for value in text.split()]
    if len(values) != 6:
        raise ValueError(f'expected six SDF pose values, got {len(values)}')
    return Pose2D(x=values[0], y=values[1], yaw=values[5])


def compose(parent: Pose2D, child: Pose2D) -> Pose2D:
    """Compose two planar poses using the SDF parent-child convention."""
    cos_yaw = math.cos(parent.yaw)
    sin_yaw = math.sin(parent.yaw)
    return Pose2D(
        x=parent.x + cos_yaw * child.x - sin_yaw * child.y,
        y=parent.y + sin_yaw * child.x + cos_yaw * child.y,
        yaw=parent.yaw + child.yaw,
    )


def collision_shapes(world_path: Path) -> list[Shape]:
    """Load supported static box and cylinder collisions from the SDF world."""
    world = ET.parse(world_path).getroot().find('world')
    if world is None:
        raise ValueError(f'{world_path} does not contain an SDF world')

    shapes: list[Shape] = []
    for model in world.findall('model'):
        if model.findtext('static', default='false').strip().lower() != 'true':
            continue
        model_name = model.get('name', '')
        if model_name == 'ground_plane':
            continue
        model_pose = parse_pose(model.findtext('pose'))
        for link in model.findall('link'):
            link_pose = compose(model_pose, parse_pose(link.findtext('pose')))
            for collision in link.findall('collision'):
                pose = compose(link_pose, parse_pose(collision.findtext('pose')))
                geometry = collision.find('geometry')
                if geometry is None:
                    continue
                name = '/'.join(
                    (
                        model_name,
                        link.get('name', ''),
                        collision.get('name', ''),
                    )
                )
                box = geometry.find('box')
                cylinder = geometry.find('cylinder')
                if box is not None:
                    size = [float(value) for value in box.findtext('size', '').split()]
                    if len(size) != 3:
                        raise ValueError(f'{name} box must contain three size values')
                    shapes.append(Shape(name, 'box', pose, size[0], size[1]))
                elif cylinder is not None:
                    radius = float(cylinder.findtext('radius', 'nan'))
                    shapes.append(Shape(name, 'cylinder', pose, radius, radius))
    if not shapes:
        raise ValueError(f'no supported static collision shapes found in {world_path}')
    return shapes


def rasterize(shapes: Iterable[Shape]) -> list[list[int]]:
    """Rasterize collision footprints using each map cell center."""
    shape_list = list(shapes)
    rows: list[list[int]] = []
    for row in range(HEIGHT_CELLS):
        y = ORIGIN_Y_M + (HEIGHT_CELLS - row - 0.5) * RESOLUTION_M
        pixels: list[int] = []
        for column in range(WIDTH_CELLS):
            x = ORIGIN_X_M + (column + 0.5) * RESOLUTION_M
            occupied = any(shape.contains(x, y) for shape in shape_list)
            pixels.append(OCCUPIED if occupied else FREE)
        rows.append(pixels)
    return rows


def pgm_text(rows: list[list[int]]) -> str:
    """Encode the occupancy grid as a deterministic ASCII PGM."""
    header = (
        'P2\n'
        '# Generated by robotest_navigation/tools/generate_map.py; do not hand edit.\n'
        f'{WIDTH_CELLS} {HEIGHT_CELLS}\n'
        '255\n'
    )
    body = ''.join(' '.join(str(pixel) for pixel in row) + '\n' for row in rows)
    return header + body


def yaml_text() -> str:
    """Return the matching Nav2 map metadata."""
    return (
        '# Generated by robotest_navigation/tools/generate_map.py; do not hand edit.\n'
        'image: robotest_lab.pgm\n'
        'mode: trinary\n'
        f'resolution: {RESOLUTION_M:.2f}\n'
        f'origin: [{ORIGIN_X_M:.1f}, {ORIGIN_Y_M:.1f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
    )


def write_map(world_path: Path, output_dir: Path) -> None:
    """Generate both map files in the requested directory."""
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = rasterize(collision_shapes(world_path))
    (output_dir / 'robotest_lab.pgm').write_text(pgm_text(rows), encoding='ascii')
    (output_dir / 'robotest_lab.yaml').write_text(yaml_text(), encoding='ascii')


def main() -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    arguments = parser.parse_args()
    write_map(arguments.world.resolve(), arguments.output_dir.resolve())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
