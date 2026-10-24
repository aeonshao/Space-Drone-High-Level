'''
Tests the detection + pose estimation on synthetic images (renders a tag with a known pose
using a homography, then checks we get that pose back). No ROS or camera needed.
'''
import math
import cv2
import numpy as np
import pytest

from spacedrone_vision.aruco_utils import (TagDetector, pose_estimation, rvec_to_quaternion,
                                           tag_pixel_area, intrinsics_from_fov)

W, H = 1280, 960
K = intrinsics_from_fov(W, H, 1.74)
DIST = np.zeros(5)


def make_tag(tag_id, px=400, border=80):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        tag = cv2.aruco.generateImageMarker(d, tag_id, px)
    else:
        tag = cv2.aruco.drawMarker(d, tag_id, px)
    return cv2.copyMakeBorder(tag, border, border, border, border, cv2.BORDER_CONSTANT, value=255), px, border


def facing_camera(yaw=0.0):
    '''
    rvec for a tag facing the camera, spun by yaw. rvec=0 is NOT facing the camera,
    the tag z axis has to point back at the lens (spent way too long on this, the tag
    was rendered mirrored and obviously never got detected)
    '''
    R_flip, _ = cv2.Rodrigues(np.array([math.pi, 0.0, 0.0]))
    R_yaw, _ = cv2.Rodrigues(np.array([0.0, 0.0, yaw]))
    rvec, _ = cv2.Rodrigues(R_flip @ R_yaw)
    return rvec.flatten()


def render(tag_id, tvec, rvec=None, size=0.5):
    '''put a tag of `size` meters at tvec (camera frame) in a blank grey image'''
    if rvec is None:
        rvec = facing_camera()
    tag, px, border = make_tag(tag_id)
    s = size / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
    img_pts, _ = cv2.projectPoints(obj, np.array(rvec, dtype=np.float64), np.array(tvec, dtype=np.float64), K, DIST)
    src = np.array([[border, border], [border + px, border], [border + px, border + px], [border, border + px]],
                   dtype=np.float32)
    Hm = cv2.getPerspectiveTransform(src, img_pts.reshape(4, 2).astype(np.float32))
    bg = np.full((H, W), 128, np.uint8)
    warped = cv2.warpPerspective(tag, Hm, (W, H), borderValue=0)
    mask = cv2.warpPerspective(np.full_like(tag, 255), Hm, (W, H))
    bg[mask > 0] = warped[mask > 0]
    return bg


@pytest.mark.parametrize('tvec', [(0.0, 0.0, 6.0), (1.5, -2.0, 6.0), (-0.5, 0.8, 3.0)])
def test_pose_recovers_translation(tvec):
    img = render(7, tvec)
    det = TagDetector('DICT_4X4_50')
    corners, ids, _ = det.detect(img)
    assert ids is not None and ids.flatten().tolist() == [7]
    rvecs, tvecs, oks = pose_estimation(corners, 0.5, K, DIST)
    assert oks[0]
    t = tvecs[0].flatten()
    # within 5cm at 6m is good enough for us
    assert np.allclose(t, tvec, atol=0.05), f'{t} vs {tvec}'


def test_rotated_tag_still_detected():
    img = render(3, (0.5, 0.5, 5.0), rvec=facing_camera(1.2))
    corners, ids, _ = TagDetector('DICT_4X4_50').detect(img)
    assert ids is not None and 3 in ids.flatten()


def test_wrong_dict_doesnt_detect():
    img = render(7, (0.0, 0.0, 4.0))
    corners, ids, _ = TagDetector('DICT_6X6_250').detect(img)
    assert ids is None or 7 not in ids.flatten()


def test_quaternion_identity():
    q = rvec_to_quaternion([0, 0, 0])
    assert q == pytest.approx((0, 0, 0, 1))


def test_quaternion_90_about_z():
    q = rvec_to_quaternion([0, 0, math.pi / 2])
    assert q == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)), abs=1e-6)


def test_pixel_area_square():
    c = np.array([[[0, 0], [10, 0], [10, 10], [0, 10]]], dtype=np.float32)
    assert tag_pixel_area(c) == pytest.approx(100)


def test_thin_border_not_detected_on_grey():
    # documents the quiet zone problem, if this starts passing opencv fixed it
    tag, _, _ = make_tag(7, border=40)
    small = cv2.resize(tag, (150, 150), interpolation=cv2.INTER_AREA)
    bg = np.full((H, W), 180, np.uint8)
    bg[100:250, 100:250] = small
    _, ids, _ = TagDetector('DICT_4X4_50').detect(bg)
    if ids is not None:
        pytest.skip('opencv version handles thin borders fine')
