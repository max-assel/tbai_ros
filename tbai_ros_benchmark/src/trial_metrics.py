"""Summarize finalized bags, keeping unavailable measurements explicit."""

import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import statistics

import yaml


def write_json(path, value):
  path = Path(path)
  temporary = path.with_suffix(path.suffix + '.tmp')
  temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
  temporary.replace(path)


def field(message, path):
  for part in path.split('.'):
    message = getattr(message, part)
  return message


def percentile(values, fraction):
  position = (len(values) - 1) * fraction
  lower = int(position)
  upper = min(lower + 1, len(values) - 1)
  return values[lower] + (values[upper] - values[lower]) * (position - lower)


class TrialMetrics:
  def summarize(self, bag_path, result, config, world):
    """Use native state timestamps within the measured motion interval."""
    import rosbag
    starts = [e.sim_time_sec for e in result.events if e.kind == 'motion_start']
    ends = [e.sim_time_sec for e in result.events if e.kind == 'termination']
    start, end = (starts[0] if starts else None), (ends[-1] if ends else None)
    sources = config['runtime_sources']
    runtimes = {name: [] for name in sources}
    invalid_runtime = {name: 0 for name in sources}
    stream_topics = {name: config['data_sources'][name]['topic']
                     for name in ('state', 'action', 'velocity_request')}
    counts = {name: 0 for name in stream_topics}
    topics = set(stream_topics.values()) | {s['topic'] for s in sources.values() if s}
    points = []
    invalid_trajectory = None
    with rosbag.Bag(str(bag_path), 'r') as bag:
      for topic, message, receipt in bag.read_messages(topics=list(topics)):
        if start is None or end is None:
          continue
        receipt_time = receipt.to_sec()
        for name, stream_topic in stream_topics.items():
          if topic == stream_topic and start <= receipt_time <= end:
            counts[name] += 1
        if topic == stream_topics['state']:
          state_source = config['data_sources']['state']
          stamp = field(message, state_source['timestamp_field']).to_sec()
          if start <= stamp <= end:
            values = field(message, state_source['array_field'])
            if len(values) != state_source['expected_array_length'] or not all(math.isfinite(v) for v in values):
              invalid_trajectory = 'invalid_state'
            else:
              x, y = [values[i] for i in state_source['position_indices'][:2]]
              if not points or stamp != points[-1][0]:
                points.append((stamp, x, y))
        for name, source in sources.items():
          if not source or source['topic'] != topic:
            continue
          timestamp = field(message, source['timestamp_field']).to_sec()
          if start + config['runtime_summary']['warmup_exclusion_sim_sec'] <= timestamp <= end:
            value = float(field(message, source['field']))
            value *= {'s': 1000, 'ms': 1, 'us': 0.001, 'ns': 0.000001}[source['unit']]
            if math.isfinite(value) and value >= 0:
              runtimes[name].append(value)
            else:
              invalid_runtime[name] += 1
    distance = 0.0
    previous = None
    maximum_gap = config['trajectory']['max_sample_gap_sim_sec']
    for point in points:
      if previous is not None:
        dt = point[0] - previous[0]
        if dt == 0:
          continue
        segment = math.hypot(point[1] - previous[1], point[2] - previous[2])
        if dt < 0:
          invalid_trajectory = 'state_time_regression'
        elif dt > maximum_gap:
          invalid_trajectory = 'state_gap'
        elif segment / dt > config['trajectory']['max_segment_speed_mps']:
          invalid_trajectory = 'implausible_position_jump'
        distance += segment
      previous = point
    if start is None or end is None:
      invalid_trajectory = 'motion_interval_unavailable'
    elif len(points) < 2:
      invalid_trajectory = 'insufficient_state_samples'
    elif points[0][0] - start > maximum_gap or end - points[-1][0] > maximum_gap:
      invalid_trajectory = 'incomplete_state_coverage'
    result.distance_traveled_xy_m = None if invalid_trajectory else distance
    if points:
      gx, gy = config['world_settings'][world]['goal_position'][:2]
      if end is not None and 0 <= end - points[-1][0] <= maximum_gap:
        result.final_goal_distance_xy_m = math.hypot(points[-1][1] - gx, points[-1][2] - gy)
    summary = {'trajectory': {'available': invalid_trajectory is None, 'reason': invalid_trajectory,
                              'samples': len(points)},
               'streams': {name: {'topic': stream_topics[name], 'samples': count,
                                  'available': count > 0} for name, count in counts.items()},
               'recovery': {'available': False, 'reason': 'no_verified_event_source'},
               'components': {}}
    for name, values in runtimes.items():
      values.sort()
      summary['components'][name] = {
        'available': bool(values),
        'reason': None if values else ('source_not_configured' if not sources[name] else 'no_valid_samples'),
        'unit': 'ms', 'count': len(values), 'invalid_samples': invalid_runtime[name],
        'mean': statistics.mean(values) if values else None,
        'median': statistics.median(values) if values else None,
        'p95': percentile(values, 0.95) if values else None,
        'p99': percentile(values, 0.99) if values else None,
        'max': max(values) if values else None,
      }
    return summary

  def save(self, attempt_dir, result, runtime_summary):
    attempt_dir = Path(attempt_dir)
    write_json(attempt_dir / 'result.json', asdict(result))
    temporary = attempt_dir / 'events.jsonl.tmp'
    temporary.write_text(''.join(json.dumps(asdict(event), allow_nan=False) + '\n' for event in result.events))
    temporary.replace(attempt_dir / 'events.jsonl')
    write_json(attempt_dir / 'runtime_summary.json', runtime_summary)
    metadata_path = attempt_dir / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text()) if metadata_path.exists() else {}
    row = {key: metadata.get(key) for key in ('world', 'baseline', 'repetition')}
    row.update(attempt_dir=str(attempt_dir), status=result.status, reason=result.reason,
               stage=result.stage, sim_duration_sec=result.sim_duration_sec,
               wall_duration_sec=result.wall_duration_sec, time_to_goal_sim_sec=result.time_to_goal_sim_sec,
               distance_traveled_xy_m=result.distance_traveled_xy_m,
               final_goal_distance_xy_m=result.final_goal_distance_xy_m,
               cleanup_errors='; '.join(result.cleanup_errors))
    summary_path = attempt_dir.parents[2] / 'summary.csv'
    exists = summary_path.exists()
    with summary_path.open('a', newline='') as stream:
      writer = csv.DictWriter(stream, fieldnames=list(row))
      if not exists:
        writer.writeheader()
      writer.writerow(row)
