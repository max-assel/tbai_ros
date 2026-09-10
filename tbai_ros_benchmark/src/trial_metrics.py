"""Skeleton: summarize finalized bags without adding load to the active trial."""


class TrialMetrics:
  """Collect trajectory, distance and per-component runtime summaries."""

  def summarize(self, bag_path, result, config):
    """Calculate selected metrics within motion_start through termination."""
    # TODO: Extract timestamped state and high/low-level actions from verified topics.
    # Suggestion: keep original rates/timestamps; optional CSV export should document alignment.
    # /cmd_vel is a velocity request, not a substitute for actual joint commands.
    # TODO: Calculate XY path length and final XY goal distance in one verified frame.
    # Suggestion: sum hypot(dx, dy) over ordered valid samples; exclude reset/cleanup.
    # Document filtering/sampling to control jitter bias; flag gaps/teleports rather than bridge them.
    # Missing trajectory produces null metrics with reasons, not misleading zero values.
    # TODO: Summarize MPC, WBC and policy durations separately: count, mean, median, p95, p99, max in ms.
    # Suggestion: define percentile method and warmup exclusion; use configured topic/field/unit.
    # Publication rate is NOT runtime. If unavailable, add small timing instrumentation around
    # existing calls using monotonic time; define GPU dispatch versus completed execution timing.
    # TODO: Preserve monitor time-to-goal and recovery events; represent unavailable data explicitly.
    # Suggestion: unsuccessful trials have null time-to-goal; retain raw runtime samples in the bag.
    raise NotImplementedError("Trajectory and runtime distributions")

  def save(self, attempt_dir, result, runtime_summary):
    """Persist selected metrics alongside raw data."""
    # TODO: Write result.json, events.jsonl, runtime_summary.json and batch summary.csv.
    # Suggestion: use dataclasses.asdict, atomic writes and unique attempt paths; retain metadata.yaml,
    # recording.bag and process logs. Record bag finalization/metric availability and failure reasons.
    # Analysis errors must not erase trial outcome or prevent cleanup.
    raise NotImplementedError("Result persistence")
