from glob import glob
from typing import List
from pathlib import Path
import click


from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import get_ros_env, echo, call, start_watcher


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


def collect_xunit_xmls(destination: Path, package: str):  # type: ignore
    """Collects all pytest, gtest XML files from the test results"""

    package = "*" if not package else package

    # pytest puts their results into build/pkg_name/pytest.xml
    xmls = list((Path.cwd() / "build").glob(f"{package}/pytest.xml"))
    # pytest based tests put their results into build/pkg_name/test_results/pkg_name/*.xunit.xml
    xmls += list((Path.cwd() / "build").glob(f"{package}/test_results/{package}/*.xunit.xml"))
    # gtest based tests put their results into build/pkg_name/test_results/pkg_name/*.gtest.xml
    xmls += list((Path.cwd() / "build").glob(f"{package}/test_results/{package}/*.gtest.xml"))

    for f in xmls:
        pkg_name = f.parent.name
        dest = destination / pkg_name
        dest.mkdir(exist_ok=True)
        echo(f"Copying {f.relative_to(Path.cwd())} to {dest}", "green")
        call(f"cp --no-clobber {f} {dest}")


class Ros(PlatformCliGroup):
    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated with ROS packages")
        def ros():
            pass

        @ros.command(name="build")
        @click.option("--package", type=str, default=None, help="The package to build")
        @click.option("--debug-symbols", is_flag=True, show_default=True, default=False)
        @click.option("--no-base", is_flag=True, default=False)
        @click.option(
            "--watch", type=bool, is_flag=True, default=False, help="Should we watch for changes?"
        )
        @click.argument("args", nargs=-1)
        def build(package: str, debug_symbols: bool, no_base: bool, watch: bool, args: List[str]):
            """Runs colcon build on all ROS package"""

            def command():
                args_str = " ".join(args)

                if package:
                    # use --packages-up-to if the dependencies weren't installed
                    # use --packages-select if all the dependencies were rosdepped
                    args_str += f" --packages-select {package}"

                if not no_base:
                    env = get_ros_env()
                    args_str += f" --merge-install --symlink-install --install-base /opt/greenroom/{env['PLATFORM_MODULE']}"

                if debug_symbols:
                    args_str += " --cmake-args -D CMAKE_BUILD_TYPE=RelWithDebInfo"

                return call(f"colcon build {args_str}")

            if watch:
                return start_watcher(command)

            command()

        @ros.command(name="test")
        @click.option("--package", type=str, default=None, help="The package to test")
        @click.option("--results-dir", type=Path, default=None)
        @click.option(
            "--watch", type=bool, is_flag=True, default=False, help="Should we watch for changes?"
        )
        @click.option(
            "--build",
            type=bool,
            is_flag=True,
            default=False,
            help="Should we build before testing?",
        )
        @click.option(
            "--retest-until-pass",
            type=int,
            default=2,
            help="Number of times to retest until pass",
        )
        @click.argument("args", nargs=-1)
        def test(
            package: str,
            results_dir: Path,
            watch: bool,
            build: bool,
            retest_until_pass: int,
            args: List[str],
        ):
            """Runs colcon test on all ROS packages"""

            env = get_ros_env()
            args_str = " ".join(args)

            # Some args only apply to colcon test
            args_str_test = args_str
            if package:
                args_str_test += f" --packages-select {package}"

            def command():
                if build:
                    call("platform ros build")
                p1 = call(
                    " ".join(
                        [
                            "colcon test",
                            "--merge-install",
                            f"--install-base /opt/greenroom/{env['PLATFORM_MODULE']}",
                            "--event-handlers console_direct+",
                            f"--retest-until-pass {retest_until_pass}",
                            '--pytest-args "--verbose"',
                            args_str_test,
                        ]
                    ),
                    abort=False,
                )
                p2 = call(f"colcon test-result --all --verbose {args_str}", abort=False)
                return p1, p2

            if watch:
                return start_watcher(command)

            p1, p2 = command()

            if results_dir and results_dir.exists():
                collect_xunit_xmls(results_dir, package)

            exit(max([p1.returncode, p2.returncode]))

        @ros.command(name="launch")
        @click.argument("package_name", type=str)
        @click.argument("launch_file_name", type=str)
        @click.option(
            "--watch", type=bool, is_flag=True, default=False, help="Should we watch for changes?"
        )
        @click.option(
            "--build",
            type=bool,
            is_flag=True,
            default=False,
            help="Should we build before launching?",
        )
        def launch(package_name: str, launch_file_name: str, watch: bool, build: bool):  # type: ignore
            """Launches a ROS Package"""

            def command():
                if build:
                    call("platform ros build")
                return call(f"ros2 launch {package_name} {launch_file_name}", process=watch)

            if watch:
                return start_watcher(command)

            command()

        @ros.command(name="run")
        @click.option(
            "--watch", type=bool, is_flag=True, default=False, help="Should we watch for changes?"
        )
        @click.option(
            "--build",
            type=bool,
            is_flag=True,
            default=False,
            help="Should we build before launching?",
        )
        @click.argument("args", nargs=-1)
        def run(watch: bool, build: bool, args: List[str]):  # type: ignore
            """Runs a script after building the ROS packages"""

            def command():
                if build:
                    call("platform ros build")
                return call(" ".join(args), process=watch)

            if watch:
                return start_watcher(command)

            command()

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
