from glob import glob
from pathlib import Path
import click
import subprocess

from platform_cli.helpers import Env

class RosPackages():
    """
    CLI handlers associated with ROS packages
    """
    def __init__(self, env: Env):
        self.env = env

    def _get_ros_poetry_packages(self, path:Path):
        package_xmls = glob(str(path / "**/package.xml"))
        package_xml_dirs = [Path(item).parent for item in package_xmls]
        pyproject_tomls = glob(str(path / "**/pyproject.toml"))
        pyproject_toml_dirs = [Path(item).parent for item in pyproject_tomls]

        ros_poetry_packages = []
        for dir in pyproject_toml_dirs:
            if dir in package_xml_dirs:
                ros_poetry_packages.append(dir)

        return ros_poetry_packages

    def build(self):
        click.echo(click.style("Building all packages...", fg='green'))
        subprocess.call(
            f"colcon build --install-base /opt/greenroom/{self.env['PACKAGE_MODULE']}",
            shell=True,
        )

    def test(self):
        click.echo(click.style("Testing all packages...", fg='green'))
        subprocess.call(
            f"colcon test --install-base /opt/greenroom/{self.env['PACKAGE_MODULE']}",
            shell=True,
        )

    def install_poetry_deps(self, base_path: Path):
        click.echo(click.style(f"Installing all poetry deps in {base_path}", fg='green'))
        ros_poetry_packages = self._get_ros_poetry_packages(base_path)
        for dir in ros_poetry_packages:
            click.echo(click.style(f"Installing {str(dir)}...", fg="blue"))
            error = subprocess.call(f"cd {dir} && poetry install", shell=True)
            if (error):
                raise click.ClickException("Install failed")
