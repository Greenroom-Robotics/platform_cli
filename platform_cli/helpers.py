from typing import TypedDict, cast
import click
import os
from pathlib import Path
import subprocess

class Env(TypedDict):
    PLATFORM_MODULE: str
    ROS_OVERLAY: str

def get_env() -> Env:
    if "PLATFORM_MODULE" not in os.environ:
        raise click.ClickException("PLATFORM_MODULE environment variable must be set. eg) platform_perception")
    if "ROS_OVERLAY" not in os.environ:
        raise click.ClickException("ROS_OVERLAY environment variable must be set. eg) /opt/ros/galactic")
    check_gr_auth()

def check_gr_auth():
    if "GHCR_PAT" not in os.environ:
        raise click.ClickException("Personal access token not in environment variables! Aborting.")

    return cast(Env, os.environ)

def echo(msg: str, color: str):
    click.echo(click.style(msg, fg=color)) # type: ignore

class Env(TypedDict):
    PLATFORM_MODULE: str
    ROS_OVERLAY: str


def get_project_root() -> Path:
    # not sure of a great way to find the project root, find the first .git directory?

    p = Path.cwd()

    while len(p.parts) > 1:
        if (p / ".git").exists() and (p / "package.json").exists():
            return p
        else:
            p = p.parent

    return None


def stdout_call(command: str, cwd=None, project_root_cwd=False, abort=True) -> str:
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


def call(command: str, cwd=None, project_root_cwd=False, abort=True) -> subprocess.CompletedProcess:
    if project_root_cwd and cwd:
        raise RuntimeError("Both 'cwd' and 'project_root_cwd' are set")

    if project_root_cwd:
        cwd = get_project_root()
        if cwd is None:
            raise RuntimeError("Could not find project root.")

    click.echo(click.style(f"Running: {click.style(command, bold=True)} in {click.style(str(cwd if cwd else Path.cwd()), bold=True)}", fg="blue"))
    try:
        proc = subprocess.run(command, shell=True, executable="/bin/bash", cwd=cwd, check=abort)
    except subprocess.CalledProcessError as e:
        print(e)
        raise click.ClickException("Run failed")

    return proc
