"""Clone the robot description packages that build_glb.py reads meshes from into ../deps,
at the commits this build was made with.

Run:  python fetch_deps.py
"""

import subprocess
from pathlib import Path

DEPS = Path(__file__).resolve().parent.parent / "deps"

# name: (url, commit)
PACKAGES = {
    "eddie_description": ("https://github.com/secorolab/eddie_description.git",
                          "b447e245ae47b60e1db318bd226a8ce0beb6340b"),
    "ros2_kortex": ("https://github.com/secorolab/ros2_kortex.git",
                    "b86f223249da004f6ef863c788ed4fce7157be24"),
    "ros2_robotiq_gripper": ("https://github.com/PickNikRobotics/ros2_robotiq_gripper.git",
                             "a5e1405098166dd9bd63fd24b3c03173fc1cb984"),
}


def git(*args: str, cwd: Path | None = None) -> str:
    return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()


def main() -> None:
    DEPS.mkdir(exist_ok=True)
    for name, (url, commit) in PACKAGES.items():
        repo = DEPS / name
        if not (repo / ".git").exists():
            print(f"cloning {name}")
            git("clone", "--quiet", url, str(repo))
        if git("rev-parse", "HEAD", cwd=repo) != commit:
            git("fetch", "--quiet", "origin", cwd=repo)
            git("checkout", "--quiet", commit, cwd=repo)
        print(f"{name} at {commit[:10]}")


if __name__ == "__main__":
    main()
