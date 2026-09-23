#!/usr/bin/env python3
"""Read-only ROS graph check for the robot bringup ownership contract.

The checker intentionally uses the public ``ros2`` CLI so it can run in a
normal ROS 2 Humble shell without adding a runtime node or changing the graph.
It reports graph evidence; it does not claim that a node name proves an
executable identity when a deployment has renamed the node.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_OUTPUT = Path("experiments/results/bunker_startup_audit/hardware_ownership_report.md")
TOPICS = ("/agt/chassis/odometry", "/wheel/odom", "/tf", "/tf_static")

# The current launch sources use ``agt_bunker_base`` or ``bunker`` as node
# names.  Do not match status bridges or arbitrary nodes containing "bunker".
BUNKER_NODE_RE = re.compile(r"(?:^|/)(?:bunker|bunker_base|agt_bunker_base)$")


def run_ros2(args: list[str]) -> tuple[str, int]:
    """Run one read-only ros2 CLI query and return combined output/status."""

    try:
        result = subprocess.run(
            ["ros2", *args],
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return f"{type(exc).__name__}: {exc}", 127
    output = "\n".join(part for part in (result.stdout, result.stderr) if part)
    return output.strip(), result.returncode


def parse_nodes(output: str) -> list[str]:
    nodes = []
    for line in output.splitlines():
        line = line.strip()
        if line.startswith("/"):
            nodes.append(line)
    return sorted(set(nodes))


def parse_topic_info(output: str) -> dict[str, object]:
    if "Unknown topic" in output or "Unknown topic name" in output:
        return {"exists": False, "publisher_count": 0, "publisher_nodes": []}

    count_match = re.search(r"Publisher count:\s*(\d+)", output)
    publisher_nodes: list[str] = []
    # ``ros2 topic info -v`` prints one endpoint block per publisher/subscriber.
    # Keep only blocks explicitly marked as PUBLISHER.
    for block in re.split(r"\n\s*\n", output):
        if re.search(r"Endpoint type:\s*PUBLISHER", block):
            node_match = re.search(r"Node name:\s*(\S+)", block)
            namespace_match = re.search(r"Node namespace:\s*(\S+)", block)
            if node_match:
                name = node_match.group(1)
                namespace = namespace_match.group(1) if namespace_match else "/"
                if namespace == "/":
                    publisher_nodes.append(f"/{name.lstrip('/')}")
                else:
                    publisher_nodes.append(
                        f"{namespace.rstrip('/')}/{name.lstrip('/')}"
                    )

    return {
        "exists": bool(count_match) or bool(output),
        "publisher_count": int(count_match.group(1)) if count_match else 0,
        "publisher_nodes": sorted(set(publisher_nodes)),
    }


def query_graph() -> dict[str, object]:
    node_output, node_status = run_ros2(["node", "list"])
    nodes = parse_nodes(node_output) if node_status == 0 else []
    topic_info = {}
    for topic in TOPICS:
        output, status = run_ros2(["topic", "info", "-v", topic])
        topic_info[topic] = parse_topic_info(output) if status == 0 else {
            "exists": False,
            "publisher_count": 0,
            "publisher_nodes": [],
            "error": output,
        }
    bunker_nodes = [node for node in nodes if BUNKER_NODE_RE.search(node)]
    return {
        "nodes": nodes,
        "bunker_nodes": bunker_nodes,
        "topics": topic_info,
        "node_query_status": node_status,
    }


def topic_row(graph: dict[str, object], topic: str) -> tuple[int, list[str]]:
    info = graph["topics"][topic]
    return int(info["publisher_count"]), list(info["publisher_nodes"])


def render_report(graph: dict[str, object]) -> str:
    bunker_nodes = graph["bunker_nodes"]
    bunker_count = len(bunker_nodes)
    ownership_status = "PASS" if bunker_count <= 1 else "FAIL"
    domain_id = os.environ.get("ROS_DOMAIN_ID", "0 (default/unset)")
    generated = datetime.now(timezone.utc).isoformat(timespec="seconds")

    lines = [
        "# Hardware Ownership Report",
        "",
        "只读 ROS graph 检查；本工具没有创建节点、发布 topic 或启动硬件。",
        "",
        f"Generated: `{generated}`",
        f"ROS_DOMAIN_ID: `{domain_id}`",
        "",
        "## Ownership",
        "",
        f"Status: **{ownership_status}**",
        "",
        f"Bunker-like node count: `{bunker_count}`",
        "",
        "Node names observed:",
        "",
    ]
    if bunker_nodes:
        lines.extend(f"- `{node}`" for node in bunker_nodes)
    else:
        lines.append("- none")

    lines.extend(
        [
            "",
            "Rule: `PASS` means the graph exposes at most one node using the "
            "current canonical Bunker node names (`/bunker`, `/bunker_base`, "
            "`/agt_bunker_base`). A renamed executable cannot be identified from "
            "`ros2 node list` alone.",
            "",
            "## Odometry Publishers",
            "",
            "| topic | publisher count | publisher nodes |",
            "| --- | ---: | --- |",
        ]
    )
    for topic in ("/agt/chassis/odometry", "/wheel/odom"):
        count, nodes = topic_row(graph, topic)
        lines.append(
            f"| `{topic}` | {count} | "
            f"{', '.join(f'`{node}`' for node in nodes) if nodes else 'none'} |"
        )

    lines.extend(["", "## TF Sources", ""])
    for topic in ("/tf", "/tf_static"):
        count, nodes = topic_row(graph, topic)
        lines.extend(
            [
                f"### `{topic}`",
                "",
                f"Publisher count: `{count}`",
                "",
                f"Sources: {', '.join(f'`{node}`' for node in nodes) if nodes else 'none'}",
                "",
            ]
        )
    tf_counts = sum(topic_row(graph, topic)[0] for topic in ("/tf", "/tf_static"))
    if tf_counts > 1:
        lines.append(
            "Potential TF source multiplicity detected. Topic graph data does not "
            "identify whether the same `odom -> base_link` pair is duplicated."
        )
    else:
        lines.append(
            "No multiple TF publisher is visible in the current graph. Exact "
            "frame-pair ownership still requires TF message/frame inspection."
        )

    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "- This report checks current graph visibility only; it does not inspect launch history.",
            "- A missing `/wheel/odom` or `/agt/chassis/odometry` topic is reported as zero publishers.",
            "- A `PASS` with zero Bunker nodes means no duplicate is visible; it does not mean the hardware owner is running.",
            "- The Bunker executable must not be started twice even when its remapped odometry topics differ.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Markdown output path (default: {DEFAULT_OUTPUT})",
    )
    args = parser.parse_args()

    report = render_report(query_graph())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(report, encoding="utf-8")
    print(report, end="")
    print(f"Wrote {args.output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
