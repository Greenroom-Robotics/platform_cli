from typing import TypedDict, cast
import click
import os

class Env(TypedDict):
    PLATFORM_MODULE: str
    ROS_OVERLAY: str

def get_env() -> Env:
    if "PLATFORM_MODULE" not in os.environ:
        raise click.ClickException("PLATFORM_MODULE environment variable must be set. eg) platform_perception")
    if "ROS_OVERLAY" not in os.environ:
        raise click.ClickException("ROS_OVERLAY environment variable must be set. eg) /opt/ros/galactic")

    return cast(Env, os.environ)

def echo(msg: str, color: str):
    click.echo(click.style(msg, fg=color)) # type: ignore