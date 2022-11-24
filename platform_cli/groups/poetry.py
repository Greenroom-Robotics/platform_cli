import subprocess
from glob import glob
from pathlib import Path
import click

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import echo

default_path = Path.cwd() / "packages"

class Poetry(PlatformCliGroup):
    def _get_non_ros_poetry_packages(self, path: Path=default_path):
        package_xmls = glob(str(path / "**/package.xml"), recursive=True)
        package_xml_dirs = [Path(item).parent for item in package_xmls]
        pyproject_tomls = glob(str(path / "**/pyproject.toml"), recursive=True)
        pyproject_toml_dirs = [Path(item).parent for item in pyproject_tomls]

        non_ros_poetry_packages: list[Path] = []
        for dir in pyproject_toml_dirs:
            if dir not in package_xml_dirs:
                non_ros_poetry_packages.append(dir)

        echo(f"{len(non_ros_poetry_packages)} package(s) found", 'green')
        return non_ros_poetry_packages

    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated with pure poetry packages")
        def poetry():
            pass

        @poetry.command(name="install")
        def install(): # type: ignore
            """Runs poetry install on all poetry packages"""

            echo("Installing non-ros poetry packages...", 'blue')
            non_ros_poetry_packages = self._get_non_ros_poetry_packages()
            for dir in non_ros_poetry_packages:
                echo(f"Installing {str(dir)}...", "blue")
                error = subprocess.call(f"cd {dir} && poetry install", shell=True, executable='/bin/bash')
                if (error):
                    raise click.ClickException("Install failed")
            echo("Complete", "green")

        @poetry.command(name="test")
        def test(): # type: ignore
            """Runs pytest on all poetry packages"""

            echo("Testing non-ros poetry packages...", "blue")
            # Find the packages which are not ros packages...
            non_ros_poetry_packages = self._get_non_ros_poetry_packages()

            for dir in non_ros_poetry_packages:
                echo(f"Running tests for {str(dir)}...", "blue")
                error = subprocess.call(f"cd {dir} && python3 -m pytest .", shell=True, executable='/bin/bash')
                if (error):
                    raise click.ClickException("Pytest failed")

            echo("Complete", "green")