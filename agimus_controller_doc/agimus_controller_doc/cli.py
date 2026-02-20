import sys
from pathlib import Path
from sphinx.cmd.build import build_main
<<<<<<< HEAD
from typing import List, Optional


def build(argv: Optional[List[str]] = None) -> int:
=======


def build(argv: list | None = None) -> int:
>>>>>>> cf4d68b (Add agimus-controller-doc Nix derivation and expose package in flake)
    """Build the documentation.

    Usage (after `poetry install`):
      poetry run agimus-docs-build

    This will run Sphinx to build HTML in `docs/_build/html`.
    """
    project_root = Path(__file__).resolve().parents[1]
    docs_src = project_root / "docs"
    build_dir = docs_src / "_build" / "html"

    if argv is None:
        argv = ["-b", "html", str(docs_src), str(build_dir)]

    # Ensure docs source exists
    if not docs_src.exists():
        print(f"Docs source not found at {docs_src}", file=sys.stderr)
        return 2

    print(f"Building docs from {docs_src} into {build_dir}")
    # Call sphinx build
    return build_main(argv)


if __name__ == "__main__":
    sys.exit(build())
