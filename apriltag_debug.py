#!/usr/bin/env python3

import subprocess
import yaml
import sys
import time

TOPIC = "/apriltag/positions"
SETUP = "install/setup.bash"


def get_message():
    command = f"source {SETUP} && ros2 topic echo {TOPIC} --once"

    result = subprocess.run(
        ["bash", "-c", command],
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        print("ros2 topic echo failed:")
        print(result.stderr)
        return None

    output = result.stdout.strip()

    if not output:
        print("ros2 topic echo returned no output")
        return None

    try:
        # Remove YAML document separators.
        output = output.replace("---", "").strip()

        message = yaml.safe_load(output)

        if not isinstance(message, dict):
            print("Unexpected YAML result:")
            print(repr(message))
            return None

        return message

    except yaml.YAMLError as e:
        print("YAML parsing failed:")
        print(e)
        print("Raw output:")
        print(result.stdout)
        return None


def clear_screen():
    print("\033[2J\033[H", end="")


def print_table(tags, detected):
    clear_screen()

    print("AprilTag Position Monitor")
    print("=" * 125)

    if detected:
        print("Detected tags: " + ", ".join(sorted(detected)))
    else:
        print("Detected tags: none")

    print()

    headers = [
        "Tag",
        "Status",
        "X",
        "Y",
        "Z",
        "QX",
        "QY",
        "QZ",
        "QW",
    ]

    widths = [
        18,
        12,
        13,
        13,
        13,
        13,
        13,
        13,
        13,
    ]

    print(
        " | ".join(
            f"{h:<{w}}"
            for h, w in zip(headers, widths)
        )
    )

    print("-" * 125)

    for name in sorted(tags):
        tag = tags[name]

        status = (
            "DETECTED"
            if name in detected
            else "LAST SEEN"
        )

        values = [
            name,
            status,
            f"{tag['x']:.5f}",
            f"{tag['y']:.5f}",
            f"{tag['z']:.5f}",
            f"{tag['qx']:.5f}",
            f"{tag['qy']:.5f}",
            f"{tag['qz']:.5f}",
            f"{tag['qw']:.5f}",
        ]

        print(
            " | ".join(
                f"{str(v):<{w}}"
                for v, w in zip(values, widths)
            )
        )

    print()
    print("Waiting for next update...  Ctrl+C to exit.")


def main():
    tags = {}

    while True:
        try:
            message = get_message()

            if message is None:
                time.sleep(0.03)
                continue

            positions = message.get("positions", [])

            detected = set()

            for tag in positions:
                name = tag["tag_name"]

                tags[name] = tag
                detected.add(name)

            print_table(tags, detected)

        except KeyboardInterrupt:
            print("\nExiting.")
            sys.exit(0)


if __name__ == "__main__":
    main()