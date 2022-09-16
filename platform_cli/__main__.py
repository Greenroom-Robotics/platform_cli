from pathlib import Path
from typing import cast
import click
import os

from platform_cli.helpers import check_env, Env
from platform_cli.cli.ros_packages import RosPackages
from platform_cli.cli.poetry_packages import PoetryPackages


@click.group(help=click.style("A CLI for common scripts shared between Greenroom platform modules and platform CI.", fg="green"))
def cli():
    pass

env = cast(Env, os.environ)

# Colcon packages
ros_packages = RosPackages(env)

@cli.group(help="Commands for ROS packages")
def ros():
    check_env()
    pass

@ros.command(name="build")
def ros_build():
    """Runs colcon build on all ros packages"""
    ros_packages.build()

@ros.command(name="test")
def ros_test():
    """Runs colcon test on all ros packages"""
    ros_packages.test()

@ros.command(name="install_poetry_deps")
@click.option('--base-path', type=str, help="The path to where the packages are installed")
def ros_install_poetry_deps(base_path: str):
    """Installs the poetry deps for any python packages"""
    base_path_defaulted = Path(base_path) if base_path else Path.cwd() / "packages"
    ros_packages.install_poetry_deps(base_path=base_path_defaulted)

# Poetry packages
poetry_packages = PoetryPackages(env)

@cli.group(help="Commands for pure poetry packages")
def poetry():
    # check_env()
    pass

@poetry.command(name="install")
def poetry_install():
    """Runs poetry install on all poetry packages"""
    poetry_packages.install()

@poetry.command(name="test")
def poetry_test():
    """Runs pytest on all poetry packages"""
    poetry_packages.test()

if __name__ == '__main__':
    cli()