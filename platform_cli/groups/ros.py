from glob import glob
from typing import List
from pathlib import Path
import click
import subprocess

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import get_ros_env, echo

class Ros(PlatformCliGroup):
    def _get_ros_poetry_packages(self, path:Path) -> List[Path]:
        package_xmls = glob(str(path / "**/package.xml"), recursive=True)
        package_xml_dirs = [Path(item).parent for item in package_xmls]
        pyproject_tomls = glob(str(path / "**/pyproject.toml"), recursive=True)
        pyproject_toml_dirs = [Path(item).parent for item in pyproject_tomls]

        ros_poetry_packages: List[Path] = []
        for dir in pyproject_toml_dirs:
            if dir in package_xml_dirs:
                ros_poetry_packages.append(dir)

        echo(f"{len(ros_poetry_packages)} package(s) found", 'green')
        return ros_poetry_packages

    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated with ROS packages")
        def ros():
            pass
            
        @ros.command(name="build")
        @click.argument("args", nargs=-1)
        def build(args: List[str]): # type: ignore
            """Runs colcon build on all ros package"""

            env = get_ros_env()
            args_str = " ".join(args)

            echo("Building packages...", 'green')
            error = subprocess.call(
                f"colcon build --merge-install --install-base /opt/greenroom/{env['PLATFORM_MODULE']} {args_str}",
                shell=True,
                executable='/bin/bash'
            )
            if (error):
                raise click.ClickException("Build failed")
                
        @ros.command(name="test")
        @click.argument("args", nargs=-1)
        def test(args: List[str]): # type: ignore
            """Runs colcon test on all ros packages"""

            env = get_ros_env()
            args_str = " ".join(args)

            echo("Testing packages...", 'green')
            error = subprocess.call(
                f"colcon test --merge-install --install-base /opt/greenroom/{env['PLATFORM_MODULE']} {args_str}; colcon test-result --all --verbose",
                shell=True,
                executable='/bin/bash'
            )
            if (error):
                raise click.ClickException("Test failed")

        @ros.command(name="install_poetry_deps")
        @click.option('--base-path', type=str, help="The path to where the packages are installed")
        def install_poetry_deps(base_path: Path): # type: ignore
            """Installs the poetry deps for any python packages"""

            env = get_ros_env()
            base_path = Path(base_path) if base_path else Path(f"/opt/greenroom/{env['PLATFORM_MODULE']}")


            echo(f"Installing all poetry deps in {base_path} using pip", 'green')
            ros_poetry_packages = self._get_ros_poetry_packages(base_path)
            # Disable venv
            subprocess.call("poetry config virtualenvs.create false", shell=True, executable='/bin/bash')

            for dir in ros_poetry_packages:
                echo(f"Installing {str(dir)}...", "blue")
                # export dependencies to a requirements.txt file without hashes to decrease time to resolve dependencies.
                # https://github.com/python-poetry/poetry-plugin-export/issues/78
                error = subprocess.call(f"cd {dir} && poetry export -f requirements.txt --without-hashes --output requirements.txt && pip3 install -r requirements.txt && rm requirements.txt", shell=True, executable='/bin/bash')
                if (error):
                    raise click.ClickException("Install failed")
