"""Statistics for a figure-eight flight, independent of ROS transport."""

import math


def percentile(values: list[float], percentage: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentage / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summarize(rows: list[dict], final_error: float, final_speed: float,
              settled: bool, offboard_losses: int, failsafe_events: int,
              abort_reason: str = "") -> dict:
    errors = [row["position_error_m"] for row in rows]
    rmse = math.sqrt(sum(error * error for error in errors) / len(errors)) if errors else math.nan
    p95 = percentile(errors, 95.0)
    maximum = max(errors) if errors else math.nan
    saturation_samples = sum(bool(row["saturation_flags"]) for row in rows)
    passed = (
        not abort_reason and len(errors) >= 100 and settled
        and rmse <= 0.25 and p95 <= 0.40 and maximum <= 0.60
        and final_error <= 0.25 and final_speed <= 0.20
        and offboard_losses == 0 and failsafe_events == 0
    )
    return {
        "passed": bool(passed),
        "reason": abort_reason,
        "tracking_samples": len(errors),
        "tracking_rmse_m": rmse,
        "tracking_p95_m": p95,
        "tracking_max_m": maximum,
        "final_error_m": final_error,
        "final_speed_mps": final_speed,
        "settled_for_2s": settled,
        "offboard_losses": offboard_losses,
        "failsafe_events": failsafe_events,
        "saturation_samples": saturation_samples,
        "saturation_fraction": saturation_samples / len(errors) if errors else math.nan,
    }
