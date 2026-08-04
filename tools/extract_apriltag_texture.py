#!/usr/bin/env python3
"""Rectify an AprilTag photographed at an angle into a Gazebo texture."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")
    args = parser.parse_args()

    image = cv2.imread(args.input, cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit("cannot read input image: %s" % args.input)

    # Corners of the printed square in the supplied 1279 x 1705 photograph.
    source = np.float32([[455, 506], [907, 521], [873, 925], [466, 912]])
    destination = np.float32([[0, 0], [511, 0], [511, 511], [0, 511]])
    rectified = cv2.warpPerspective(
        image, cv2.getPerspectiveTransform(source, destination), (512, 512)
    )
    gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Gazebo's camera benefits from a quiet white border around the code.
    texture = cv2.copyMakeBorder(binary, 64, 64, 64, 64, cv2.BORDER_CONSTANT, value=255)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), texture):
        raise SystemExit("cannot write output image: %s" % output)
    print("wrote %s (%dx%d)" % (output, texture.shape[1], texture.shape[0]))

    families = {
        "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
    }
    for family, dictionary_id in families.items():
        dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        _, ids, _ = cv2.aruco.detectMarkers(texture, dictionary)
        if ids is not None:
            print("detected %s id=%d" % (family, int(ids[0][0])))


if __name__ == "__main__":
    main()
