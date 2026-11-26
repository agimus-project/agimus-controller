#!/usr/bin/env python3
"""Check that every real-time function has a real-time test.

Every header tag `/// @rt Type::method` needs a doctest test case whose name
starts with `RtType.method`; a free function `/// @rt name` needs one starting
with `RtFree.name`. See doc/rt_safety.md.
"""

import argparse
import pathlib
import re
import sys

TAG = re.compile(r"///\s*@rt\s+(\S+)")
TEST = re.compile(r'TEST_CASE(?:_FIXTURE)?\s*\(\s*(?:[^,"]*,\s*)?"([^"]+)"')


def expected_prefix(tag: str) -> str:
    if "::" in tag:
        owner, method = tag.rsplit("::", 1)
        return f"Rt{owner.split('::')[-1]}.{method}"
    return f"RtFree.{tag}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--include", type=pathlib.Path, required=True)
    parser.add_argument("--tests", type=pathlib.Path, required=True)
    args = parser.parse_args()

    tags = {}
    for header in sorted(args.include.rglob("*.hpp")):
        for line_no, line in enumerate(header.read_text().splitlines(), 1):
            match = TAG.search(line)
            if match:
                tags.setdefault(match.group(1), f"{header}:{line_no}")

    test_names = []
    for source in sorted(args.tests.rglob("*.cpp")):
        test_names += TEST.findall(source.read_text())

    missing = [
        (tag, where)
        for tag, where in sorted(tags.items())
        if not any(name.startswith(expected_prefix(tag)) for name in test_names)
    ]
    for tag, where in missing:
        print(f"{where}: @rt {tag} has no test named '{expected_prefix(tag)}...'")
    print(f"{len(tags) - len(missing)}/{len(tags)} @rt functions covered.")
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
