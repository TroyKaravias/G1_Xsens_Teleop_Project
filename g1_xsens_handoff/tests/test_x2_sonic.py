import numpy as np
import pytest

from xsens_bridge.x2_sonic import (
    ACTION_SIZE,
    HISTORY_LENGTH,
    OBSERVATION_SIZE,
    PROPRIOCEPTION_SIZE,
    TOKENIZER_SIZE,
    X2SonicProprioceptionBuffer,
    assemble_observation,
    build_recorded_tokenizer_observation,
)


def test_proprioception_is_primed_and_term_major():
    buffer = X2SonicProprioceptionBuffer()
    angvel = np.array([1, 2, 3], dtype=np.float32)
    joint_pos = np.arange(31, dtype=np.float32)
    joint_vel = joint_pos + 100
    action = joint_pos + 200
    gravity = np.array([4, 5, 6], dtype=np.float32)
    buffer.append(angvel, joint_pos, joint_vel, action, gravity)

    flat = buffer.flattened()
    assert flat.shape == (PROPRIOCEPTION_SIZE,)
    np.testing.assert_array_equal(flat[:3], angvel)
    np.testing.assert_array_equal(flat[3:6], angvel)
    joint_pos_start = HISTORY_LENGTH * 3
    np.testing.assert_array_equal(flat[joint_pos_start : joint_pos_start + 31], joint_pos)
    np.testing.assert_array_equal(flat[-3:], gravity)


def test_unprimed_buffer_fails_closed():
    with pytest.raises(RuntimeError, match="not been primed"):
        X2SonicProprioceptionBuffer().flattened()


def test_assemble_fused_observation_order_and_shape():
    tokenizer = np.arange(TOKENIZER_SIZE, dtype=np.float32)
    proprio = np.arange(PROPRIOCEPTION_SIZE, dtype=np.float32) + 1000
    obs = assemble_observation(tokenizer, proprio)
    assert obs.shape == (1, OBSERVATION_SIZE)
    np.testing.assert_array_equal(obs[0, :TOKENIZER_SIZE], tokenizer)
    np.testing.assert_array_equal(obs[0, TOKENIZER_SIZE:], proprio)
    assert ACTION_SIZE == 31


def test_assemble_rejects_wrong_dimensions():
    with pytest.raises(ValueError, match="tokenizer"):
        assemble_observation(np.zeros(679), np.zeros(PROPRIOCEPTION_SIZE))


def test_tokenizer_is_interleaved_per_future_frame():
    trajectory = np.stack([np.arange(31) + frame * 100 for frame in range(20)])
    tokenizer = build_recorded_tokenizer_observation(trajectory, 0.1, 0.0)
    first = tokenizer[:68]
    # First future frame is trajectory[1], in IsaacLab gather order.
    assert first[0] == trajectory[1, 0]
    assert first[31] == 1000.0  # velocity of MJ joint 0: (100 - 0) / 0.1
    np.testing.assert_array_equal(first[62:], [1, 0, 0, 0, 1, 0])
