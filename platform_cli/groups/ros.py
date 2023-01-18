from glob import glob
from typing import List
from pathlib import Path
import click

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import get_ros_env, echo, call


def get_ros_poetry_packages(path: Path) -> List[Path]:
    package_xmls = glob(str(path / "**/package.xml"), recursive=True)
    package_xml_dirs = [Path(item).parent for item in package_xmls]
    pyproject_tomls = glob(str(path / "**/pyproject.toml"), recursive=True)
    pyproject_toml_dirs = [Path(item).parent for item in pyproject_tomls]

    ros_poetry_packages: List[Path] = []
    for dir in pyproject_toml_dirs:
        if dir in package_xml_dirs:
            ros_poetry_packages.append(dir)

    echo(f"{len(ros_poetry_packages)} package(s) found", "green")
    return ros_poetry_packages


class Ros(PlatformCliGroup):
    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated with ROS packages")
        def ros():
            pass

        @ros.command(name="build")
        @click.option("--package", type=str, default=None, help="The package to build")
        @click.argument("args", nargs=-1)
        def build(package: str, args: List[str]):  # type: ignore
            """Runs colcon build on all ROS package"""

            env = get_ros_env()
            args_str = " ".join(args)

            if package:
                # use --packages-up-to if the dependencies weren't installed
                # use --packages-select if all the dependencies were rosdepped
                args_str += f" --packages-select {package}"

            echo("Building packages...", "green")
            call(
                f"colcon build --merge-install --install-base /opt/greenroom/{env['PLATFORM_MODULE']} {args_str}"
            )

        @ros.command(name="test")
        @click.option("--results-dir", type=str, default=None)
        @click.option("--package", type=str, default=None, help="The package to test")
        @click.argument("args", nargs=-1)
        def test(results_dir: str, package: str, args: List[str]):  # type: ignore
            """Runs colcon test on all ROS packages"""

            env = get_ros_env()
            args_str = " ".join(args)

            if results_dir:
                args_str += f" --test-result-base {results_dir}"

            # Some args only apply to colcon test
            args_str_test = args_str
            if package:
                args_str_test += f" --packages-select {package}"

            echo("Testing packages...", "green")
            p = call(
                f"colcon test --merge-install --install-base /opt/greenroom/{env['PLATFORM_MODULE']} {args_str_test}",
                abort=False,
            )
            p2 = call(f"colcon test-result --all --verbose {args_str}", abort=False)

            exit(max([p.returncode, p2.returncode]))

        @ros.command(name="install_poetry_deps")
        @click.option("--base-path", type=str, help="The path to where the packages are installed")
        def install_poetry_deps(base_path: Path):  # type: ignore
            """Installs the poetry deps for any python packages"""

            env = get_ros_env()
            base_path = (
                Path(base_path) if base_path else Path(f"/opt/greenroom/{env['PLATFORM_MODULE']}")
            )

            echo(f"Installing all poetry deps in {base_path} using pip", "green")
            ros_poetry_packages = get_ros_poetry_packages(base_path)
            # Disable venv
            call("poetry config virtualenvs.create false")

            for dir in ros_poetry_packages:
                echo(f"Installing {str(dir)}...", "blue")
                # export dependencies to a requirements.txt file without hashes to decrease time to resolve dependencies.
                # https://github.com/python-poetry/poetry-plugin-export/issues/78
                call(
                    "poetry export -f requirements.txt --without-hashes --output requirements.txt",
                    cwd=dir,
                )
                call("pip3 install -r requirements.txt", cwd=dir)
                (dir / "requirements.txt").unlink()
