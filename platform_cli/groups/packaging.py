from pathlib import Path
from typing import List, Dict
import click
from psutil import cpu_count
import shutil
import os

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import call, stdout_call, get_pkg_env, echo


GR_APT_REPO_PRIVATE = "Greenroom-Robotics/packages"
GR_APT_REPO_PUBLIC = "Greenroom-Robotics/public_packages"
GR_APT_REPO_PATH = Path.home() / ".gr/gr-packages"


def get_ros_distro():
    return os.environ["ROS_DISTRO"]


def get_debs(p: Path) -> List[Path]:
    return list(p.glob("*.deb")) + list(p.glob("*.ddeb"))


def find_packages(p: Path) -> Dict[str, str]:
    # hack city.. hack hack city bitch
    pkgs = {
        k: v
        for k, v in zip(
            stdout_call("colcon list -n", cwd=p).split("\n"),
            stdout_call("colcon list -p", cwd=p).split("\n"),
        )
        if k and v
    }
    return pkgs


def parse_version(version: str):
    # version should be of the form "1.2.3" or "1.2.3-alpha.1"
    version_split = version.split("-")
    version_semver = version_split[0]
    version_prerelease = version_split[1] if len(version_split) == 2 else ""
    if len(version_semver.split(".")) != 3:
        raise ValueError("Version should be of the form 1.2.3 or 1.2.3-alpha1")

    return version_semver, version_prerelease


def get_apt_repo_url(public: bool = False) -> str:
    """If we have API_TOKEN_GITHUB, use https, otherwise use ssh"""
    packages_repo = GR_APT_REPO_PUBLIC if public else GR_APT_REPO_PRIVATE

    if "API_TOKEN_GITHUB" in os.environ:
        return f"https://x-access-token:{os.environ['API_TOKEN_GITHUB']}@github.com/{packages_repo}.git"

    return f"git@github.com:{packages_repo}.git"


class Packaging(PlatformCliGroup):
    def create(self, cli: click.Group):
        @cli.group(help="Packaging commands")
        def pkg():
            pass

        @pkg.command(name="setup")
        def setup():  # type: ignore reportUnusedFunction
            """Sets up the greenroom apt and rosdep lists"""
            call(
                f"curl -s https://{os.environ['GHCR_PAT']}@raw.githubusercontent.com/Greenroom-Robotics/rosdistro/main/scripts/setup-rosdep.sh | bash -s"
            )
            call(
                f"curl -s https://{os.environ['GHCR_PAT']}@raw.githubusercontent.com/Greenroom-Robotics/packages/main/scripts/setup-apt.sh | bash -s"
            )
            call(
                "curl -s https://raw.githubusercontent.com/Greenroom-Robotics/public_packages/main/scripts/setup-apt.sh | bash -s"
            )
            call("rosdep init", sudo=True, abort=False)

        @pkg.command(name="clean")
        def clean():  # type: ignore reportUnusedFunction
            """Removes debians and log directories"""
            dirs = [".obj-x86_64-linux-gnu", ".obj-aarch64-linux-gnu", "debian", "log"]

            for d in dirs:
                p = Path(d)
                if p.is_dir():
                    shutil.rmtree(p)

        @pkg.command(name="refresh-deps")
        def refresh_deps():  # type: ignore reportUnusedFunction
            """Refresh rosdeps"""
            call("sudo apt-get update")
            call("rosdep update")

        @pkg.command(name="install-deps")
        def install_deps():  # type: ignore reportUnusedFunction
            """Installs rosdeps"""
            get_pkg_env()
            pkg_dir = Path.cwd()
            refresh_deps.callback()  # type: ignore
            call(f"rosdep install -y --rosdistro {get_ros_distro()} --from-paths {pkg_dir} -i")

        @pkg.command(name="get-sources")
        def get_sources():  # type: ignore reportUnusedFunction
            """Imports items from the .repo file"""
            if Path(".repos").is_file():
                call("vcs import --recursive < .repos")
            else:
                raise click.ClickException("No '.repos' file found. Unsure how to obtain sources.")

        @pkg.command(name="build")
        @click.option("--version", type=str, help="The version to call the debian", default=None)
        @click.option(
            "--output",
            type=str,
            default="debs",
            help="The output directory for the debs",
        )
        @click.option("--no-tests", type=bool, default=True)
        def build(version: str, output: str, no_tests: bool):  # type: ignore reportUnusedFunction
            """Builds the package using bloom"""

            pkg_name = Path.cwd().name
            pkg_type = "rosdebian"
            src_dir = Path("src")
            bloom_args = ""

            if version is not None:
                version_semver, version_prerelease = parse_version(version)
                echo(f"Updating package.xml version to {version_semver}", "blue")
                # This will replace anything between the <version></version> tags in the package.xml
                call(
                    f'sed -i ":a;N;\\$!ba; s|<version>.*<\\/version>|<version>{version_semver}<\\/version>|g" package.xml'
                )

                if version_prerelease:
                    bloom_args += f'-i "{version_prerelease}"'

            # need to make this more generic
            if src_dir.is_dir():
                pkgs = find_packages(src_dir)
                if pkgs and pkg_name in pkgs:
                    bloom_args += f" --src-dir={src_dir / pkgs[pkg_name]}"

            if no_tests:
                bloom_args += " --no-tests"

            # this is the equivalent of turning off type safety for compiled libraries.
            # TODO rework the build process to build deps debs first if required, then install debs then build pkgs
            bloom_args += " --ignore-shlibs-missing-info"

            call(f"bloom-generate {pkg_type} --ros-distro {get_ros_distro()} {bloom_args}")

            cpus = cpu_count() if cpu_count() else 1
            call(f"fakeroot debian/rules binary -j{cpus}")

            # the .deb and .ddeb files are in the parent directory
            # move .deb/.ddeb files into the output folder
            Path(output).mkdir(parents=True, exist_ok=True)
            debs = get_debs(Path.cwd().parent)
            echo(f"Moving {len(debs)} .deb / .ddeb files to {output}", "blue")
            if debs:
                for d in debs:
                    try:
                        shutil.move(str(d), output)
                    except Exception as e:
                        raise click.ClickException(
                            f"Error moving .deb. You may need to chown the output folder: {e}"
                        )

            else:
                raise click.ClickException("No debs found.")

            echo("Build complete", "green")

        @pkg.command(name="apt-clone")
        @click.option(
            "--public",
            type=bool,
            help="Should this package be published to the public PPA",
            default=False,
        )
        def apt_clone(public: bool):  # type: ignore reportUnusedFunction
            """Checks out the GR apt repo"""
            github_repo_url = get_apt_repo_url(public)
            try:
                call(f"git clone --filter=blob:none {github_repo_url} {GR_APT_REPO_PATH}")
            except Exception as e:
                raise click.ClickException(f"Error cloning apt repo: {e}")

        @pkg.command(name="apt-push")
        def apt_push():  # type: ignore reportUnusedFunction
            """Pushes to the GR apt repo"""
            call("git pull --rebase", cwd=GR_APT_REPO_PATH)
            call("git push", cwd=GR_APT_REPO_PATH)

        @pkg.command(name="apt-update")
        def apt_update():  # type: ignore reportUnusedFunction
            """Update the GR apt repo"""
            if not GR_APT_REPO_PATH:
                raise click.ClickException("GR apt repo has not been cloned.")
            call("git pull --rebase", cwd=GR_APT_REPO_PATH)

        @pkg.command(name="apt-add")
        @click.argument("deb", type=click.Path(exists=True), required=False)
        def apt_add(deb: str):  # type: ignore reportUnusedFunction
            """Adds a .deb to the GR apt repo"""

            if not GR_APT_REPO_PATH:
                raise click.ClickException("GR apt repo has not been cloned.")

            if deb:
                debs = [Path(deb)]
            else:
                debs = get_debs(Path.cwd())

            if not debs:
                raise click.ClickException("No debs found.")
            for d in debs:
                shutil.copy(d, GR_APT_REPO_PATH / "debian")
                call(f"git add debian/{d.name}", cwd=GR_APT_REPO_PATH)

            call(
                f"git commit -a -m 'feat: add debian package: {' '.join(d.name for d in debs)}'",
                cwd=GR_APT_REPO_PATH,
            )
