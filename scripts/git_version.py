Import("env")

import subprocess

try:
    git_sha = subprocess.check_output(
        ["git", "describe", "--always", "--dirty"],
        text=True,
    ).strip()
except (subprocess.CalledProcessError, FileNotFoundError):
    git_sha = "unknown"

env.Append(
    BUILD_FLAGS=[
        f'-DGIT_SHA=\\"{git_sha}\\"',
    ]
)
