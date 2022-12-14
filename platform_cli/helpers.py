from typing import TypedDict, cast, Optional
import click
import os
from pathlib import Path
import subprocess


class RosEnv(TypedDict):
    PLATFORM_MODULE: str
    ROS_OVERLAY: str

class PkgEnv(TypedDict):
    GHCR_PAT: str


def get_ros_env() -> RosEnv:
    if "PLATFORM_MODULE" not in os.environ:
        raise click.ClickException("PLATFORM_MODULE environment variable must be set. eg) platform_perception")
    if "ROS_OVERLAY" not in os.environ:
        raise click.ClickException("ROS_OVERLAY environment variable must be set. eg) /opt/ros/humble")
    
    return cast(RosEnv, os.environ)


def get_pkg_env() -> PkgEnv:
    if "GHCR_PAT" not in os.environ:
        raise click.ClickException("Personal access token not in environment variables! Aborting.")

    return cast(PkgEnv, os.environ)


def echo(msg: str, color: str):
    click.echo(click.style(msg, fg=color)) # type: ignore


def get_project_root() -> Optional[Path]:
    # not sure of a great way to find the project root, find the first .git directory?

    p = Path.cwd()

    while len(p.parts) > 1:
        if (p / ".git").exists() and (p / "package.json").exists():
            return p
        else:
            p = p.parent

    return None


def stdout_call(command: str, cwd: Optional[Path]=None, project_root_cwd: bool=False, abort: bool=True) -> str:
    if project_root_cwd and cwd:
        raise RuntimeError("Both 'cwd' and 'project_root_cwd' are set")

    if project_root_cwd:
        cwd = get_project_root()
        if cwd is None:
            raise RuntimeError("Could not find project root.")

    click.echo(click.style(f"Running: {click.style(command, bold=True)} in {click.style(str(cwd if cwd else Path.cwd()), bold=True)}", fg="blue"))
    try:
        proc = subprocess.run(command, shell=True, executable="/bin/bash", capture_output=True, cwd=cwd, check=abort)
    except subprocess.CalledProcessError as e:
        print(e)
        raise click.ClickException("Run failed")

    return proc.stdout.decode('ascii')


def call(command: str, cwd: Optional[Path]=None, project_root_cwd: bool=False, abort: bool=True, sudo: bool=False, env=None):
    if project_root_cwd and cwd:
        raise RuntimeError("Both 'cwd' and 'project_root_cwd' are set")

    if project_root_cwd:
        cwd = get_project_root()
        if cwd is None:
            raise RuntimeError("Could not find project root.")

    if env:
        env = {**os.environ, **env}

    if sudo:
        command = "sudo " + command
        
    click.echo(click.style(f"Running: {click.style(command, bold=True)} in {click.style(str(cwd if cwd else Path.cwd()), bold=True)}", fg="blue"))
    try:
        proc = subprocess.run(command, shell=True, executable="/bin/bash", cwd=cwd, check=abort, env=env)
    except subprocess.CalledProcessError as e:
        print(e)
        raise click.ClickException("Run failed")

    return proc
