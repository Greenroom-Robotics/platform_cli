import subprocess
from glob import glob
from pathlib import Path
import click

from platform_cli.helpers import Env

default_path = Path.cwd() / "packages"

class PoetryPackages():
    """
    CLI handlers associated with pure poetry packages
    """
    def __init__(self, env: Env):
        self.env = env

    def _get_non_ros_poetry_packages(self, path=default_path):
        package_xmls = glob(str(path / "**/package.xml"), recursive=True)
        package_xml_dirs = [Path(item).parent for item in package_xmls]
        pyproject_tomls = glob(str(path / "**/pyproject.toml"), recursive=True)
        pyproject_toml_dirs = [Path(item).parent for item in pyproject_tomls]

        non_ros_poetry_packages = []
        for dir in pyproject_toml_dirs:
            if dir not in package_xml_dirs:
                non_ros_poetry_packages.append(dir)

        click.echo(click.style(f"{len(non_ros_poetry_packages)} package(s) found", fg='green'))
        return non_ros_poetry_packages

    def install(self):
        click.echo(click.style("Installing non-ros poetry packages...", fg='blue'))
        non_ros_poetry_packages = self._get_non_ros_poetry_packages()
        for dir in non_ros_poetry_packages:
            click.echo(click.style(f"Installing {str(dir)}...", fg="blue"))
            error = subprocess.call(f"cd {dir} && poetry install", shell=True, executable='/bin/bash')
            if (error):
                raise click.ClickException("Install failed")
        click.echo(click.style("Complete", fg="green"))

    def test(self):
        click.echo(click.style("Testing non-ros poetry packages...", fg="blue"))
        # Find the packages which are not ros packages...
        non_ros_poetry_packages = self._get_non_ros_poetry_packages()

        for dir in non_ros_poetry_packages:
            click.echo(click.style(f"Running tests for {str(dir)}...", fg="blue"))
            error = subprocess.call(f"cd {dir} && python3 -m pytest .", shell=True, executable='/bin/bash')
            if (error):
                raise click.ClickException("Pytest failed")

        click.echo(click.style("Complete", fg="green"))