import pytest

from eight.evaluation import percentile, summarize


def test_percentile_and_acceptance():
    assert percentile([1.0, 2.0, 3.0], 50.0) == pytest.approx(2.0)
    rows = [{"position_error_m": 0.1, "saturation_flags": 0} for _ in range(200)]
    result = summarize(rows, 0.05, 0.02, True, 0, 0)
    assert result["passed"]
    assert result["tracking_rmse_m"] == pytest.approx(0.1)
    assert result["tracking_p95_m"] == pytest.approx(0.1)


def test_safety_event_fails_acceptance():
    rows = [{"position_error_m": 0.1, "saturation_flags": 1} for _ in range(200)]
    result = summarize(rows, 0.05, 0.02, True, 1, 0)
    assert not result["passed"]
    assert result["saturation_fraction"] == pytest.approx(1.0)
