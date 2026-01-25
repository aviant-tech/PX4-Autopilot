#!/usr/bin/env python3
"""
Calculate bandwidth usage for logged topics in add_default_topics().
Excludes CONSTRAINED_MEMORY, HITL, and SITL sections.

Supports optional profiles:
  --ekf-replay: Enable estimator replay profile (full rate for EKF inputs)
  --high-rate: Enable high rate profile (full rate for control loop)
"""

import re
import csv
import os
import argparse

MAX_ESTIMATOR_INSTANCES = 6
DEFAULT_MULTI_MAX_INSTANCES = 10

# Paths are resolved relative to this file's location.
# This script lives at <PX4-Autopilot>/Tools/aviant/logger_budget.py
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PX4_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
MSG_DIR = os.path.join(PX4_ROOT, 'msg')
LOGGED_TOPICS_FILE = os.path.join(PX4_ROOT, 'src/modules/logger/logged_topics.cpp')
LOGGED_TOPICS_HEADER = os.path.join(PX4_ROOT, 'src/modules/logger/logged_topics.h')


def parse_interval_constants(header_path):
    """Parse interval constants from logged_topics.h."""
    interval_map = {}
    with open(header_path, 'r') as f:
        for line in f:
            # Match: static constexpr uint16_t NAME = VALUE;
            match = re.match(r'\s*static constexpr uint16_t (\w+)\s*=\s*(\d+)\s*;', line)
            if match:
                name, value = match.groups()
                if name in ('AUTO', 'VERY_FAST', 'FAST', 'MODERATE', 'SLOW'):
                    interval_map[name] = int(value)
    return interval_map


# Load interval values from header (can be overridden via CLI)
INTERVAL_MAP = parse_interval_constants(LOGGED_TOPICS_HEADER)

# Field type sizes in bytes
TYPE_SIZES = {
    'bool': 1,
    'char': 1,
    'int8': 1,
    'uint8': 1,
    'int16': 2,
    'uint16': 2,
    'int32': 4,
    'uint32': 4,
    'int64': 8,
    'uint64': 8,
    'float32': 4,
    'float64': 8,
}


def parse_msg_file(msg_path, visited=None):
    """
    Parse a .msg file and calculate its serialized size with XCDR alignment.
    Returns size in bytes or None if file not found.
    """
    if visited is None:
        visited = set()

    if msg_path in visited:
        return 0  # Avoid infinite recursion
    visited.add(msg_path)

    if not os.path.exists(msg_path):
        return None

    with open(msg_path, 'r') as f:
        content = f.read()

    offset = 0

    for line in content.splitlines():
        line = line.strip()

        # Skip empty lines and comments
        if not line or line.startswith('#'):
            continue

        # Remove inline comments before checking for constants
        code_part = line.split('#')[0].strip()
        if not code_part or '=' in code_part:
            continue

        # Parse field: "type name" or "type[N] name"
        match = re.match(r'^(\w+)(\[(\d+)\])?\s+(\w+)', line)
        if not match:
            continue

        field_type = match.group(1)
        array_size = int(match.group(3)) if match.group(3) else 1

        # Get base type size
        if field_type in TYPE_SIZES:
            type_size = TYPE_SIZES[field_type]
        else:
            # Nested message type - convert to CamelCase filename
            nested_msg = camel_to_snake(field_type) + '.msg'
            nested_path = os.path.join(MSG_DIR, nested_msg)
            nested_size = parse_msg_file(nested_path, visited.copy())
            if nested_size is None:
                # Try original name
                nested_path = os.path.join(MSG_DIR, field_type + '.msg')
                nested_size = parse_msg_file(nested_path, visited.copy())
            if nested_size is None:
                continue
            type_size = nested_size

        # Calculate alignment padding (XCDR alignment, max 8)
        align_size = min(type_size, 8)
        if align_size > 1 and offset % align_size != 0:
            padding = align_size - (offset % align_size)
            offset += padding

        # Add field size (with array multiplier)
        offset += type_size * array_size

    return offset


def camel_to_snake(name):
    """Convert CamelCase to snake_case."""
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


def snake_to_camel(name):
    """Convert snake_case to CamelCase."""
    return ''.join(word.capitalize() for word in name.split('_'))


def build_topic_to_msg_map():
    """
    Build a mapping from topic names to message files.
    Handles aliases defined with '# TOPICS' comments in .msg files.
    """
    topic_map = {}

    for filename in os.listdir(MSG_DIR):
        if not filename.endswith('.msg'):
            continue

        msg_path = os.path.join(MSG_DIR, filename)
        base_name = filename[:-4]  # Remove .msg
        # Default topic name is snake_case of filename
        default_topic = camel_to_snake(base_name)
        topic_map[default_topic] = msg_path

        # Check for TOPICS aliases
        with open(msg_path, 'r') as f:
            for line in f:
                if line.startswith('# TOPICS '):
                    aliases = line[9:].strip().split()
                    for alias in aliases:
                        topic_map[alias] = msg_path

    return topic_map


# Build the mapping once at module load
TOPIC_TO_MSG = build_topic_to_msg_map()


def get_msg_size(topic_name):
    """Get message size for a topic name."""
    msg_path = TOPIC_TO_MSG.get(topic_name)
    if msg_path:
        return parse_msg_file(msg_path)
    return None


def extract_function_body(filepath, func_name):
    """Extract function body by name."""
    with open(filepath, 'r') as f:
        content = f.read()

    pattern = rf'void LoggedTopics::{func_name}\(\)\s*\{{'
    match = re.search(pattern, content)
    if not match:
        raise ValueError(f"Could not find {func_name} function")

    start = match.end()
    brace_depth = 1
    pos = start
    while brace_depth > 0 and pos < len(content):
        if content[pos] == '{':
            brace_depth += 1
        elif content[pos] == '}':
            brace_depth -= 1
        pos += 1

    return content[start:pos-1]


def extract_add_default_topics(filepath):
    """Extract add_default_topics body, excluding conditional sections."""
    func_body = extract_function_body(filepath, 'add_default_topics')

    # Remove CONSTRAINED_MEMORY section
    func_body = re.sub(
        r'#if CONSTRAINED_MEMORY.*?#else',
        '#if 0\n#else',
        func_body,
        flags=re.DOTALL
    )

    # Remove SITL section
    func_body = re.sub(
        r'#ifdef CONFIG_ARCH_BOARD_PX4_SITL.*?#endif\s*/\*\s*CONFIG_ARCH_BOARD_PX4_SITL\s*\*/',
        '',
        func_body,
        flags=re.DOTALL
    )

    # Remove HITL section (sys_hitl >= 1)
    func_body = re.sub(
        r'if \(sys_hitl >= 1\)\s*\{[^}]*\}',
        '',
        func_body,
        flags=re.DOTALL
    )

    return func_body


def extract_profile_topics(filepath, func_name):
    """Extract topics from a profile function (e.g., add_high_rate_topics)."""
    func_body = extract_function_body(filepath, func_name)
    return func_body


def parse_line(line):
    """Parse an add_topic line."""
    stripped = line.strip()
    if not stripped or stripped.startswith('//'):
        return None

    pattern = r'add_(optional_)?(topic_multi|topic)\s*\(\s*"([^"]+)"(?:\s*,\s*([^,)]+))?(?:\s*,\s*([^,)]+))?\s*\)'
    match = re.search(pattern, stripped)
    if not match:
        return None

    is_optional = match.group(1) is not None
    is_multi = match.group(2) == 'topic_multi'
    topic_name = match.group(3)
    interval_raw = match.group(4)
    max_instances_raw = match.group(5)

    # Parse interval
    interval = 0
    interval_name = 'AUTO'  # default
    if interval_raw:
        interval_raw = interval_raw.strip()
        if interval_raw in INTERVAL_MAP:
            # Source uses constant name
            interval_name = interval_raw
            interval = INTERVAL_MAP[interval_raw]
        elif interval_raw.isdigit():
            # Source uses hardcoded number
            interval = int(interval_raw)
            interval_name = f'{interval}ms'

    # Parse instances
    num_instances = 1
    start_instance = 0
    if is_multi:
        if max_instances_raw:
            max_instances_raw = max_instances_raw.strip()
            if max_instances_raw == 'MAX_ESTIMATOR_INSTANCES':
                num_instances = MAX_ESTIMATOR_INSTANCES
            elif max_instances_raw == 'CONFIG_BOARD_UAVCAN_INTERFACES':
                num_instances = 2
            elif max_instances_raw.isdigit():
                num_instances = int(max_instances_raw)
            else:
                num_instances = DEFAULT_MULTI_MAX_INSTANCES
        else:
            num_instances = DEFAULT_MULTI_MAX_INSTANCES
    else:
        # For add_topic, second arg after interval is explicit instance
        if max_instances_raw:
            max_instances_raw = max_instances_raw.strip()
            if max_instances_raw.isdigit():
                start_instance = int(max_instances_raw)

    # Parse comment for expected Hz rate (e.g., "// 200 Hz" or "// ~50 Hz")
    expected_hz = None
    comment_match = re.search(r'//.*?(\d+)\s*Hz', stripped, re.IGNORECASE)
    if comment_match:
        expected_hz = int(comment_match.group(1))

    return {
        'topic': topic_name,
        'optional': is_optional,
        'interval': interval,
        'interval_name': interval_name,
        'start_instance': start_instance,
        'num_instances': num_instances,
        'expected_hz': expected_hz,
    }


def process_lines_into_subscriptions(func_body, subscriptions):
    """Parse lines and update subscriptions dict."""
    for line in func_body.splitlines():
        parsed = parse_line(line)
        if parsed:
            topic = parsed['topic']
            start = parsed['start_instance']
            end = start + parsed['num_instances']
            for i in range(start, end):
                key = (topic, i)
                if key not in subscriptions:
                    subscriptions[key] = {
                        'optional': parsed['optional'],
                        'interval': parsed['interval'],
                        'interval_name': parsed['interval_name'],
                        'expected_hz': parsed['expected_hz'],
                    }
                else:
                    # Non-optional overrides optional
                    if not parsed['optional']:
                        subscriptions[key]['optional'] = False
                    # Profile overrides interval (use 0 for AUTO with expected_hz)
                    subscriptions[key]['interval'] = parsed['interval']
                    subscriptions[key]['interval_name'] = parsed['interval_name']
                    # Keep expected_hz if we have one (including explicit 0 Hz)
                    if parsed['expected_hz'] is not None:
                        subscriptions[key]['expected_hz'] = parsed['expected_hz']


def main():
    parser = argparse.ArgumentParser(
        description='Calculate bandwidth usage for logged topics.'
    )
    parser.add_argument(
        '--ekf-replay',
        action='store_true',
        help='Enable estimator replay profile (full rate for EKF inputs)'
    )
    parser.add_argument(
        '--high-rate',
        action='store_true',
        help='Enable high rate profile (full rate for control loop)'
    )
    parser.add_argument(
        '-o', '--output',
        default=os.path.join(SCRIPT_DIR, 'logger_budget.csv'),
        help='Output CSV file path (default: Tools/aviant/logger_budget.csv)'
    )
    parser.add_argument(
        '--very-fast', type=int, metavar='MS',
        help=f'Override VERY_FAST interval in ms (default: {INTERVAL_MAP["VERY_FAST"]})'
    )
    parser.add_argument(
        '--fast', type=int, metavar='MS',
        help=f'Override FAST interval in ms (default: {INTERVAL_MAP["FAST"]})'
    )
    parser.add_argument(
        '--moderate', type=int, metavar='MS',
        help=f'Override MODERATE interval in ms (default: {INTERVAL_MAP["MODERATE"]})'
    )
    parser.add_argument(
        '--slow', type=int, metavar='MS',
        help=f'Override SLOW interval in ms (default: {INTERVAL_MAP["SLOW"]})'
    )
    args = parser.parse_args()

    # Apply interval overrides
    if args.very_fast is not None:
        INTERVAL_MAP['VERY_FAST'] = args.very_fast
    if args.fast is not None:
        INTERVAL_MAP['FAST'] = args.fast
    if args.moderate is not None:
        INTERVAL_MAP['MODERATE'] = args.moderate
    if args.slow is not None:
        INTERVAL_MAP['SLOW'] = args.slow

    # Start with default topics
    func_body = extract_add_default_topics(LOGGED_TOPICS_FILE)
    subscriptions = {}
    process_lines_into_subscriptions(func_body, subscriptions)

    # Apply profiles (they override intervals)
    if args.ekf_replay:
        profile_body = extract_profile_topics(LOGGED_TOPICS_FILE, 'add_estimator_replay_topics')
        process_lines_into_subscriptions(profile_body, subscriptions)
        print("Profile: EKF Replay enabled")

    if args.high_rate:
        profile_body = extract_profile_topics(LOGGED_TOPICS_FILE, 'add_high_rate_topics')
        process_lines_into_subscriptions(profile_body, subscriptions)
        print("Profile: High Rate enabled")

    # Build rows
    rows = []
    for (topic, instance), info in sorted(subscriptions.items()):
        # Format name (always include instance suffix)
        name = f"{topic}_{instance}"

        # Get message size
        msg_bytes = get_msg_size(topic)

        # Calculate kbps
        kbps = None
        expected_hz = info.get('expected_hz')
        if msg_bytes is not None:
            if info['interval'] > 0:
                # Use interval to calculate rate
                kbps = (msg_bytes / 1000) * (1000 / info['interval'])
            elif expected_hz is not None:
                # Use expected Hz from comment for AUTO interval (0 Hz = 0 kbps)
                kbps = (msg_bytes / 1000) * expected_hz

        rows.append({
            'name': name,
            'optional': info['optional'],
            'interval': info['interval'] if info['interval'] > 0 else None,
            'interval_name': info['interval_name'],
            'expected_hz': expected_hz,
            'bytes': msg_bytes,
            'kbps': round(kbps, 3) if kbps is not None else None,
        })

    # Check for AUTO topics with unknown rates
    unknown_rate = [r['name'] for r in rows if r['interval'] is None and r['expected_hz'] is None]
    if unknown_rate:
        raise ValueError(
            f"AUTO topics with unknown rate (add '// <N> Hz' comment to source):\n  " +
            "\n  ".join(unknown_rate)
        )

    # Calculate totals
    total_kbps = sum(r['kbps'] for r in rows if r['kbps'] is not None and not r['optional'])
    total_kbps_all = sum(r['kbps'] for r in rows if r['kbps'] is not None)

    # Sort by kbps descending (None values at end)
    rows.sort(key=lambda r: (r['kbps'] is None, -(r['kbps'] or 0)))

    # Calculate marginal% and cumulative% (non-optional only)
    cumulative = 0.0
    for row in rows:
        if not row['optional'] and row['kbps'] is not None and total_kbps > 0:
            row['marginal_pct'] = round(100.0 * row['kbps'] / total_kbps, 2)
            cumulative += row['marginal_pct']
            row['cumulative_pct'] = round(cumulative, 2)
        else:
            row['marginal_pct'] = None
            row['cumulative_pct'] = None

    # Write CSV
    output_file = args.output
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['name', 'optional', 'interval', 'expected_hz', 'bytes', 'kbps', 'marginal%', 'cumulative%'])
        for row in rows:
            writer.writerow([
                row['name'],
                row['optional'],
                row['interval'],
                row['expected_hz'],
                row['bytes'],
                row['kbps'],
                row['marginal_pct'],
                row['cumulative_pct'],
            ])
        # Total rows
        writer.writerow([])
        writer.writerow(['TOTAL (non-optional)', '', '', '', '', round(total_kbps, 3), '100.0', ''])
        writer.writerow(['TOTAL (all)', '', '', '', '', round(total_kbps_all, 3), '', ''])

    non_optional_count = sum(1 for r in rows if not r['optional'])
    optional_count = sum(1 for r in rows if r['optional'])
    max_topics = 255

    print(f"Written {len(rows)} topics to {output_file}")
    print(f"Subscriptions: {non_optional_count} non-optional, {optional_count} optional, {len(rows)} total (max {max_topics})")
    print(f"Bandwidth: {total_kbps:.3f} kbps (non-optional), {total_kbps_all:.3f} kbps (all)")

    # Calculate bandwidth by interval name (non-optional only)
    category_kbps = {}
    for row in rows:
        if not row['optional'] and row['kbps'] is not None:
            cat = row['interval_name']
            category_kbps[cat] = category_kbps.get(cat, 0) + row['kbps']

    # Print breakdown in order: AUTO, VERY_FAST, FAST, MODERATE, SLOW, then any custom intervals
    standard_categories = ['AUTO', 'VERY_FAST', 'FAST', 'MODERATE', 'SLOW']
    custom_categories = sorted([c for c in category_kbps.keys() if c not in standard_categories])
    parts = []
    for cat in standard_categories + custom_categories:
        if cat in category_kbps:
            pct = 100.0 * category_kbps[cat] / total_kbps if total_kbps > 0 else 0
            parts.append(f"{cat}: {pct:.1f}%")
    print(f"By interval: {', '.join(parts)}")

    # Print top 20 non-optional topics
    non_optional_rows = [r for r in rows if not r['optional'] and r['kbps'] is not None]
    non_optional_rows.sort(key=lambda r: -r['kbps'])
    print("Top 20 non-optional:")
    for r in non_optional_rows[:20]:
        pct = 100.0 * r['kbps'] / total_kbps if total_kbps > 0 else 0
        print(f"  {r['name']}: {r['kbps']:.3f} kbps ({pct:.1f}%)")

    if non_optional_count > max_topics:
        raise ValueError(
            f"Non-optional subscription count ({non_optional_count}) exceeds limit ({max_topics})"
        )


if __name__ == '__main__':
    main()
