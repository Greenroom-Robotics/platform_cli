from typing import Any, cast
import click
import subprocess
import os

# Check the environment for every command and throw errors
def check_env():
    if "PACKAGE_MODULE" not in os.environ:
        raise click.ClickException("PACKAGE_MODULE environment variable must be set. eg) platform_perception")
    if "ROS_OVERLAY" not in os.environ:
        raise click.ClickException("ROS_OVERLAY environment variable must be set. eg) /opt/ros/galactic")
env = cast(Any, os.environ)

@click.group(help=click.style("A CLI for common scripts shared between Greenroom platform modules and platform CI.", fg="green"))
def cli():
    # check_env()
    pass

@cli.command()
def build():
    """Runs colcon build on all packages in"""
    click.echo(click.style("Building all packages...", fg='green'))
    subprocess.run(
        ["colcon build --install-base" f"/opt/greenroom/{env.PACKAGE_MODULE}"],
        check=True,
    )

@cli.command()
def test():
    """Runs colcon test on all packages"""
    click.echo(click.style("Testing all packages...", fg='green'))
    subprocess.run(
        ["colcon test --install-base", f"/opt/greenroom/{env.PACKAGE_MODULE}"],
        check=True,
    )

@cli.command()
def package():
    """Runs poetry install on all packages"""
    click.echo(click.style("Deploying package...", fg='green'))
    subprocess.run(
        ["ls"],
        check=True,
    )

if __name__ == '__main__':
    cli()