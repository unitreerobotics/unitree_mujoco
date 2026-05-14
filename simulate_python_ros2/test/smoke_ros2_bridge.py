import argparse
import subprocess
import sys
import time


TOPICS = (
    "/lowstate",
    "/secondary_imu",
    "/wirelesscontroller",
    "/lf/bmsstate",
    "/sportmodestate",
)


def main():
    parser = argparse.ArgumentParser(description="Smoke-check expected ROS2 topics from simulate_python_ros2.")
    parser.add_argument("--timeout", type=float, default=5.0, help="Timeout per topic echo in seconds.")
    args = parser.parse_args()

    failures = []
    for topic in TOPICS:
        cmd = ["ros2", "topic", "echo", "--once", topic]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=args.timeout)
        except subprocess.TimeoutExpired:
            failures.append(f"{topic}: timeout after {args.timeout}s")
            continue
        if result.returncode != 0:
            failures.append(f"{topic}: exit {result.returncode}: {result.stderr.strip()}")
            continue
        sys.stdout.write(f"[OK] {topic}\n")
        sys.stdout.flush()
        time.sleep(0.1)

    if failures:
        for failure in failures:
            sys.stderr.write(f"[FAIL] {failure}\n")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
