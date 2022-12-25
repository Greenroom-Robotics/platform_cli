from typing import TypedDict, cast, Optional, Dict
import click
import os
from pathlib import Path
import subprocess


class RosEnv(TypedDict):
    PLATFORM_MODULE: str
    ROS_OVERLAY: str


class PkgEnv(TypedDict):
    GHCR_PAT: str


def check_directory_ownership(path: Path) -> bool:
    stat = path.stat()
    return stat.st_uid == os.getuid() and stat.st_gid == os.getgid()


def get_ros_env() -> RosEnv:
    for env in RosEnv.__required_keys__:  # type: ignore
        if env not in os.environ:
            raise click.ClickException(f"{env} environment variable must be set.")

    return cast(RosEnv, os.environ)


def get_pkg_env() -> PkgEnv:
    for env in PkgEnv.__required_keys__:  # type: ignore
        if env not in os.environ:
            raise click.ClickException(f"{env} environment variable must be set.")

    return cast(PkgEnv, os.environ)


def echo(msg: str, color: str):
    click.echo(click.style(msg, fg=color))  # type: ignore


def log_group_start(group_name: str):
    """Log a group start to the github actions log"""
    if (os.environ.get("CI") or "").lower() == "true":
        print(f"::group::{group_name}")


def log_group_end():
    """Log a group end to the github actions log"""
    if (os.environ.get("CI") or "").lower() == "true":
        print("::endgroup::")


def get_project_root() -> Optional[Path]:
    # not sure of a great way to find the project root, find the first .git directory?

    p = Path.cwd()

    while len(p.parts) > 1:
        if (p / ".git").exists() and (p / "package.json").exists():
            return p
        else:
            p = p.parent

    return None


def stdout_call(
    command: str,
    cwd: Optional[Path] = None,
    project_root_cwd: bool = False,
    abort: bool = True,
) -> str:
    if project_root_cwd and cwd:
        raise RuntimeError("Both 'cwd' and 'project_root_cwd' are set")

    if project_root_cwd:
        cwd = get_project_root()
        if cwd is None:
            raise RuntimeError("Could not find project root.")

    click.echo(
        click.style(
            f"Running: {click.style(command, bold=True)} in {click.style(str(cwd if cwd else Path.cwd()), bold=True)}",
            fg="blue",
        )
    )
    try:
        proc = subprocess.run(
            command,
            shell=True,
            executable="/bin/bash",
            capture_output=True,
            cwd=cwd,
            check=abort,
        )
    except subprocess.CalledProcessError as e:
        print(e)
        raise click.ClickException("Run failed")

    return proc.stdout.decode("ascii")


def call(
    command: str,
    cwd: Optional[Path] = None,
    project_root_cwd: bool = False,
    abort: bool = True,
    sudo: bool = False,
    env: Dict[str, str] = {},
):
    if project_root_cwd and cwd:
        raise RuntimeError("Both 'cwd' and 'project_root_cwd' are set")

    if project_root_cwd:
        cwd = get_project_root()
        if cwd is None:
            raise RuntimeError("Could not find project root.")

    env = {**os.environ, **env}

    if sudo:
        command = "sudo " + command

    click.echo(
        click.style(
            f"Running: {click.style(command, bold=True)} in {click.style(str(cwd if cwd else Path.cwd()), bold=True)}",
            fg="blue",
        )
    )
    try:
        proc = subprocess.run(
            command, shell=True, executable="/bin/bash", cwd=cwd, check=abort, env=env
        )
    except subprocess.CalledProcessError as e:
        print(e)
        raise click.ClickException("Run failed")

    return proc
