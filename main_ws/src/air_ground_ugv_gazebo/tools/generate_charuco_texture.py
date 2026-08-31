#!/usr/bin/env python3
"""Generate the exact ChArUco texture used by the detector configuration."""

import os

import cv2


SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH_M = 0.075
MARKER_LENGTH_M = 0.055
DICTIONARY_ID = cv2.aruco.DICT_5X5_100
OUTPUT = os.path.join(
    os.path.dirname(__file__), "..", "models", "ugv_mvp", "materials", "textures", "charuco_7x5.png"
)


def main():
    board = cv2.aruco.CharucoBoard_create(
        SQUARES_X, SQUARES_Y, SQUARE_LENGTH_M, MARKER_LENGTH_M,
        cv2.aruco.getPredefinedDictionary(DICTIONARY_ID),
    )
    # 0.565 x 0.415 m mesh at 2000 px/m: 40 px margins leave an exact
    # 1050 x 750 px (0.525 x 0.375 m) 7-by-5 ChArUco board.
    image = board.draw((1130, 830), marginSize=40, borderBits=1)
    if not cv2.imwrite(OUTPUT, image):
        raise RuntimeError("Unable to write {}".format(OUTPUT))


if __name__ == "__main__":
    main()
