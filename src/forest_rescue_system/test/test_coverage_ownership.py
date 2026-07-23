import numpy as np

from forest_rescue_system.coverage_ownership import TriangleOwnership


def test_claim_assigns_owner_to_unclaimed_triangles():
    ownership = TriangleOwnership(3)
    ownership.claim([0, 2], drone_index=1)
    np.testing.assert_array_equal(ownership.owner_ids, [1, -1, 1])


def test_claim_does_not_overwrite_existing_owner():
    ownership = TriangleOwnership(2)
    ownership.claim([0], drone_index=0)
    ownership.claim([0, 1], drone_index=1)
    np.testing.assert_array_equal(ownership.owner_ids, [0, 1])


def test_unclaimed_mask_reflects_current_state():
    ownership = TriangleOwnership(3)
    ownership.claim([1], drone_index=0)
    np.testing.assert_array_equal(
        ownership.unclaimed_mask(), [True, False, True]
    )


def test_indices_for_drone_returns_only_owned_indices():
    ownership = TriangleOwnership(4)
    ownership.claim([0, 3], drone_index=2)
    np.testing.assert_array_equal(ownership.indices_for_drone(2), [0, 3])
    np.testing.assert_array_equal(ownership.indices_for_drone(0), [])
