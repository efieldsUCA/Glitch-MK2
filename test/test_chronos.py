import numpy as np
import pytest

from glitchmk2_bringup.chronos_utils import frame_to_input, labels_to_twist, twist_to_labels


def test_labels_round_trip():
    steering, throttle = twist_to_labels(0.2, -0.25, 0.35, 0.5)
    assert steering == pytest.approx(-0.5)
    assert throttle == pytest.approx(0.2 / 0.35)
    lin_vel, ang_vel = labels_to_twist(steering, throttle, 0.35, 0.5)
    assert lin_vel == pytest.approx(0.2)
    assert ang_vel == pytest.approx(-0.25)


def test_labels_are_clipped():
    assert twist_to_labels(1.0, -2.0, 0.35, 0.5) == (-1.0, 1.0)
    assert labels_to_twist(3.0, -3.0, 0.35, 0.5) == pytest.approx((-0.35, 0.5))


def test_frame_to_input_is_rgb_chw():
    bgr = np.zeros((480, 640, 3), dtype=np.uint8)
    bgr[:, :, 2] = 255  # pure red in BGR
    tensor = frame_to_input(bgr, 180, 200)
    assert tensor.shape == (1, 3, 200, 180)
    assert tensor.dtype == np.float32
    assert tensor[0, 0].min() == pytest.approx(1.0)  # R channel first
    assert tensor[0, 1:].max() == pytest.approx(0.0)


def test_model_output_shape():
    torch = pytest.importorskip('torch')
    from glitchmk2_bringup.chronos_net import ChronosResNetLite
    model = ChronosResNetLite().eval()
    with torch.no_grad():
        out = model(torch.from_numpy(np.zeros((1, 3, 200, 180), dtype=np.float32)))
    assert out.shape == (1, 2)
