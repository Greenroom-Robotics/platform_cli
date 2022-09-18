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
        package_xmls = glob(str(path / "**/package.xml"), recursive=True)
        package_xml_dirs = [Path(item).parent for item in package_xmls]
        pyproject_tomls = glob(str(path / "**/pyproject.toml"), recursive=True)
        pyproject_toml_dirs = [Path(item).parent for item in pyproject_tomls]

        ros_poetry_packages = []
        for dir in pyproject_toml_dirs:
            if dir in package_xml_dirs:
                ros_poetry_packages.append(dir)

        click.echo(click.style(f"{len(ros_poetry_packages)} package(s) found", fg='green'))
        return ros_poetry_packages

    def build(self, args_str: str):
        click.echo(click.style("Building packages...", fg='green'))
        subprocess.call(
            f"colcon build --merge-install --install-base /opt/greenroom/{self.env['PLATFORM_MODULE']} {args_str}",
            shell=True,
            executable='/bin/bash'
        )

    def test(self, args_str: str):
        click.echo(click.style("Testing packages...", fg='green'))
        subprocess.call(
            f"colcon test --merge-install --install-base /opt/greenroom/{self.env['PLATFORM_MODULE']} {args_str}; colcon test-result --verbose",
            shell=True,
            executable='/bin/bash'
        )

    def install_poetry_deps(self, base_path: Path):
        click.echo(click.style(f"Installing all poetry deps in {base_path} using pip", fg='green'))
        ros_poetry_packages = self._get_ros_poetry_packages(base_path)
        # Disable venv
        subprocess.call("config virtualenvs.create false", shell=True, executable='/bin/bash')
        
        for dir in ros_poetry_packages:
            click.echo(click.style(f"Installing {str(dir)}...", fg="blue"))
            # export dependencies to a requirements.txt file without hashes to decrease time to resolve dependencies.
            error = subprocess.call(f"cd {dir} && poetry export --without-hashes --format=requirements.txt > requirements.txt && pip3 install -r requirements.txt", shell=True, executable='/bin/bash')
            if (error):
                raise click.ClickException("Install failed")
