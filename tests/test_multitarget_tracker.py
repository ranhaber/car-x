from cat_follow.multitarget.coordinator import MultiTargetCoordinator
from cat_follow.multitarget.predictive_tracker import PredictiveTracker
from cat_follow.multitarget.roles import PRIMARY_CAT, SECONDARY_CAT, RoleManager


def det(cx, cy, confidence=0.9, width=60, height=40):
    return (
        cx - width / 2,
        cy - height / 2,
        cx + width / 2,
        cy + height / 2,
        confidence,
        17,
    )


def test_predictive_tracker_seeds_velocity_and_coasts():
    tracker = PredictiveTracker(max_disappeared=2)
    tracker.update([det(100, 100)])
    state = next(iter(tracker.update([det(120, 100)]).values()))
    assert state.velocity == (20.0, 0.0)
    assert state.predicted_centroid == (140.0, 100.0)
    coasted = next(iter(tracker.update([]).values()))
    assert coasted.centroid == (140.0, 100.0)
    assert coasted.frames_since_update == 1


def test_low_confidence_rescues_but_does_not_create():
    tracker = PredictiveTracker(high_conf=0.3, low_conf=0.1)
    assert tracker.update([det(100, 100, 0.15)]) == {}
    tracker.update([det(100, 100, 0.9)])
    state = next(iter(tracker.update([det(102, 100, 0.15)]).values()))
    assert state.frames_since_update == 0


def test_roles_are_sticky_and_secondary_promotes():
    coordinator = MultiTargetCoordinator(max_disappeared=0)
    roles = coordinator.update([det(100, 100, 0.9), det(300, 100, 0.8)])
    assignments = {role: track_id for track_id, role in roles.items()}
    primary_id = assignments[PRIMARY_CAT]
    secondary_id = assignments[SECONDARY_CAT]

    # Confidence reversal must not flip established roles.
    roles = coordinator.update([det(100, 100, 0.4), det(300, 100, 0.95)])
    assert roles[primary_id] == PRIMARY_CAT
    assert roles[secondary_id] == SECONDARY_CAT

    # Move the primary beyond the gate so it is dropped; secondary is promoted.
    roles = coordinator.update([det(300, 100, 0.8)])
    assert roles == {secondary_id: PRIMARY_CAT}


def test_role_manager_caps_assignment_at_two_cats():
    tracker = PredictiveTracker()
    tracks = tracker.update(
        [det(100, 100, 0.7), det(300, 100, 0.9), det(500, 100, 0.5)]
    )
    roles = RoleManager().update(tracks)
    assert len(roles) == 2
