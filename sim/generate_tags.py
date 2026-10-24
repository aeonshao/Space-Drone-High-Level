#!/usr/bin/env python3
'''
Generates the ArUco tag textures + gazebo models for the "tools" in the sim world.
Each tool gets its own model folder in sim/models/tool_tag_<id>/ so we can place them
in the world file with an <include>.

Run this from anywhere:
    python3 sim/generate_tags.py

NOTE: the printed versions for the real field test are made with the same dict
(DICT_4X4_50), just print the pngs at 20cm or whatever and update marker_size in
the detector config.
'''

import os
import cv2
import numpy as np

# tag id -> tool name. keep this in sync with src/spacedrone_vision/config/tools.yaml !!
TOOLS = {
    0: "wrench",
    3: "hammer",
    7: "drill",
    12: "toolbox",
}

# 4x4 is a lot easier to pick up from altitude than the 5x5_100 we used at first
ARUCO_DICT = cv2.aruco.DICT_4X4_50
# ARUCO_DICT = cv2.aruco.DICT_5X5_100

MARKER_PX = 500       # size of the black square part
# BORDER_PX = 50
BORDER_PX = 100       # white quiet zone around it. 50 was too thin, see minMarkerDistanceRate in aruco_utils.py
MARKER_SIZE_M = 0.5   # physical size of the black square in the sim (meters)

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(HERE, "models")

MODEL_SDF = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="tool_tag_{id}">
    <static>true</static>
    <link name="base">
      <!-- small grey box so it looks a bit like a crate/tool case -->
      <visual name="case_visual">
        <pose>0 0 0.05 0 0 0</pose>
        <geometry>
          <box>
            <size>{case} {case} 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
      <visual name="tag_visual">
        <pose>0 0 0.101 0 0 0</pose>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{plane} {plane}</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.4 0.4 0.4 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://tool_tag_{id}/materials/textures/tag_{id}.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>tool_tag_{id}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>NURobotics Space Drone</name>
  </author>
  <description>ArUco tag {id} ({name}) for the tool finding mission</description>
</model>
"""


def make_tag_image(tag_id):
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    # opencv changed the aruco api in 4.7 and ubuntu 24.04 apt still ships 4.6...
    if hasattr(cv2.aruco, "generateImageMarker"):
        tag = cv2.aruco.generateImageMarker(aruco_dict, tag_id, MARKER_PX)
    else:
        tag = cv2.aruco.drawMarker(aruco_dict, tag_id, MARKER_PX)
    img = np.full((MARKER_PX + 2 * BORDER_PX, MARKER_PX + 2 * BORDER_PX), 255, dtype=np.uint8)
    img[BORDER_PX:BORDER_PX + MARKER_PX, BORDER_PX:BORDER_PX + MARKER_PX] = tag
    # gazebo wants a color texture otherwise it shows up weird
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def main():
    # plane has to be bigger than the marker so the black part is exactly MARKER_SIZE_M
    plane_size = MARKER_SIZE_M * (MARKER_PX + 2 * BORDER_PX) / MARKER_PX

    for tag_id, name in TOOLS.items():
        model_dir = os.path.join(MODELS_DIR, f"tool_tag_{tag_id}")
        tex_dir = os.path.join(model_dir, "materials", "textures")
        os.makedirs(tex_dir, exist_ok=True)

        cv2.imwrite(os.path.join(tex_dir, f"tag_{tag_id}.png"), make_tag_image(tag_id))

        with open(os.path.join(model_dir, "model.sdf"), "w") as f:
            f.write(MODEL_SDF.format(id=tag_id, plane=round(plane_size, 4), case=round(plane_size + 0.1, 4)))
        with open(os.path.join(model_dir, "model.config"), "w") as f:
            f.write(MODEL_CONFIG.format(id=tag_id, name=name))

        print(f"made tool_tag_{tag_id} ({name})")

    print("done, marker size is", MARKER_SIZE_M, "m")


if __name__ == "__main__":
    main()
