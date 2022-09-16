from typing import TypedDict
import click
import os

# Check the environment for every command and throw errors
def check_env():
    if "PLATFORM_MODULE" not in os.environ:
        raise click.ClickException("PLATFORM_MODULE environment variable must be set. eg) platform_perception")
    if "ROS_OVERLAY" not in os.environ:
        raise click.ClickException("ROS_OVERLAY environment variable must be set. eg) /opt/ros/galactic")

class Env(TypedDict):
    PLATFORM_MODULE: str
    ROS_OVERLAY: str