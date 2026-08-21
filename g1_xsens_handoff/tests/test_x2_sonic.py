import numpy as np
import pytest

from xsens_bridge.x2_sonic import (
    ACTION_SIZE,
    HISTORY_LENGTH,
    IL_TO_MJ_DOF,
    OBSERVATION_SIZE,
    PROPRIOCEPTION_SIZE,
    TOKENIZER_SIZE,
    X2SonicProprioceptionBuffer,
    X2SonicDelayedReferenceBuffer,
    X2SonicReferenceFrame,
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


def test_tokenizer_matches_x2_training_group_then_reshape_layout():
    trajectory = np.stack([np.arange(31) + frame * 100 for frame in range(20)])
    tokenizer = build_recorded_tokenizer_observation(trajectory, 0.1, 0.0)
    rows = tokenizer.reshape(10, 68)
    positions = np.stack([trajectory[index] for index in range(1, 11)])[:, IL_TO_MJ_DOF]
    velocities = np.full((10, 31), 1000.0)[:, IL_TO_MJ_DOF]
    expected_command = np.concatenate((positions.reshape(-1), velocities.reshape(-1)))
    np.testing.assert_array_equal(rows[:, :62].reshape(-1), expected_command)
    np.testing.assert_array_equal(rows[:, 62:], np.tile([1, 0, 0, 1, 0, 0], (10, 1)))


def test_delayed_live_buffer_uses_received_frames_as_future():
    buffer = X2SonicDelayedReferenceBuffer(delay_s=1.0)
    for index in range(51):
        timestamp = index * 0.02
        values = np.full(31, timestamp, dtype=np.float32)
        buffer.push(X2SonicReferenceFrame(timestamp, values, np.ones(31)))
    assert buffer.ready(1.0)
    tokenizer = buffer.tokenizer(1.0).reshape(10, 68)
    # With a 1 s delay, the policy's +0.1 ... +1.0 s references are
    # received samples at 0.1 ... 1.0 s.
    command = tokenizer[:, :62].reshape(-1)
    expected_positions = np.repeat(np.arange(0.1, 1.01, 0.1), 31)
    np.testing.assert_allclose(command[:310], expected_positions, atol=1e-6)
    np.testing.assert_allclose(command[310:], 1.0)
