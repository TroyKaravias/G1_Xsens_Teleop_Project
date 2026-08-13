"""Map Xsens body segments onto IsaacTeleop's 24-joint body layout."""

from __future__ import annotations

from dataclasses import dataclass

from .xudp import PoseFrame, SegmentPose

# Ordered exactly like core.BodyJoint in IsaacTeleop's full_body.fbs schema.
ISAAC_BODY_JOINT_NAMES = (
    "pelvis",
    "left_hip",
    "right_hip",
    "spine1",
    "left_knee",
    "right_knee",
    "spine2",
    "left_ankle",
    "right_ankle",
    "spine3",
    "left_foot",
    "right_foot",
    "neck",
    "left_collar",
    "right_collar",
    "head",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hand",
    "right_hand",
)

# Xsens positions describe segment origins. For articulated limbs, the origin of
# the child segment is the closest available estimate of the connecting joint:
# upper leg -> hip, lower leg -> knee, foot -> ankle, and so on.
#
# Xsens has four intermediate torso segments while IsaacTeleop has three spine
# joints. L3 is intentionally omitted in this first mapping; L5, T12, and T8
# provide lower, middle, and upper torso samples without interpolation.
ISAAC_TO_XSENS_SEGMENT = {
    "pelvis": "pelvis",
    "left_hip": "left_upper_leg",
    "right_hip": "right_upper_leg",
    "spine1": "l5",
    "left_knee": "left_lower_leg",
    "right_knee": "right_lower_leg",
    "spine2": "t12",
    "left_ankle": "left_foot",
    "right_ankle": "right_foot",
    "spine3": "t8",
    "left_foot": "left_toe",
    "right_foot": "right_toe",
    "neck": "neck",
    "left_collar": "left_shoulder",
    "right_collar": "right_shoulder",
    "head": "head",
    "left_shoulder": "left_upper_arm",
    "right_shoulder": "right_upper_arm",
    "left_elbow": "left_forearm",
    "right_elbow": "right_forearm",
    "left_wrist": "left_hand",
    "right_wrist": "right_hand",
    # No finger tracking is present. The rigid hand pose is duplicated so the
    # schema remains complete; this must never be interpreted as finger state.
    "left_hand": "left_hand",
    "right_hand": "right_hand",
}


@dataclass(frozen=True)
class IsaacBodyJoint:
    """One joint in IsaacTeleop order, before coordinate-frame conversion."""

    name: str
    source_segment: str
    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    is_valid: bool = True


@dataclass(frozen=True)
class IsaacBodyFrame:
    """A vendor-neutral 24-joint frame in IsaacTeleop joint order."""

    recording_timestamp_ns: int
    sample_counter: int
    joints: tuple[IsaacBodyJoint, ...]


def map_xsens_to_isaac(frame: PoseFrame) -> IsaacBodyFrame:
    """Map a decoded Xsens pose frame to IsaacTeleop's joint ordering.

    This function only changes skeleton semantics and ordering. It intentionally
    leaves Xsens coordinates unchanged; robot/world frame calibration is a
    separate, explicit stage.
    """

    source_by_name: dict[str, SegmentPose] = {
        segment.name: segment for segment in frame.segments
    }
    joints = []
    for joint_name in ISAAC_BODY_JOINT_NAMES:
        source_name = ISAAC_TO_XSENS_SEGMENT[joint_name]
        try:
            source = source_by_name[source_name]
        except KeyError as error:
            raise ValueError(
                f"Xsens frame is missing required segment {source_name!r}"
            ) from error
        joints.append(
            IsaacBodyJoint(
                name=joint_name,
                source_segment=source_name,
                position_m=source.position_m,
                quaternion_wxyz=source.quaternion_wxyz,
            )
        )

    return IsaacBodyFrame(
        recording_timestamp_ns=frame.recording_timestamp_ns,
        sample_counter=frame.header.sample_counter,
        joints=tuple(joints),
    )
