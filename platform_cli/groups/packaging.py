from pathlib import Path
from typing import List, Dict
import click
from psutil import cpu_count
import shutil
import os

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import call, stdout_call, get_pkg_env, echo

GR_APT_REPO_URL = "git@github.com:Greenroom-Robotics/packages.git"
GR_APT_REPO_PATH = Path.home() / ".gr/gr-packages"

def get_ros_distro():
    return os.environ['ROS_DISTRO']

def get_debs(p: Path) -> List[Path]:
    return list(p.glob("*.deb")) + list(p.glob("*.ddeb"))

def find_packages(p: Path) -> Dict[str, str]:
    # hack city.. hack hack city bitch
    pkgs = {k: v for k, v in zip(stdout_call(f"colcon list -n", cwd=p).split("\n"), stdout_call(f"colcon list -p", cwd=p).split("\n")) if k and v}
    return pkgs

def parse_version(version: str):
    # version should be of the form "1.2.3" or "1.2.3-alpha.1"
    version_split = version.split("-")
    version_semver = version_split[0]
    version_prerelease = version_split[1] if len(version_split) == 2 else ""
    if (len(version_semver.split(".")) is not 3):
        raise ValueError("Version should be of the form 1.2.3 or 1.2.3-alpha1")
        
    return version_semver, version_prerelease


class Packaging(PlatformCliGroup):
    def create(self, cli: click.Group):
        @cli.group(help="Packaging commands")
        def pkg():
            pass

        @pkg.command(name="setup")
        def setup(): # type: ignore reportUnusedFunction
            """Sets up the greenroom apt and rosdep lists"""
            call("curl -s https://$GHCR_PAT@raw.githubusercontent.com/Greenroom-Robotics/rosdistro/main/scripts/setup-rosdep.sh | bash -s")
            call("curl -s https://$GHCR_PAT@raw.githubusercontent.com/Greenroom-Robotics/packages/main/scripts/setup-apt.sh | bash -s")
            call("apt-get update", sudo=True)
            call("rosdep init", sudo=True, abort=False)
            call("rosdep update")

        @pkg.command(name="clean")
        def clean(): # type: ignore reportUnusedFunction
            """Removes debians and log directories"""
            dirs = ['.obj-x86_64-linux-gnu', 'debian', 'log']

            for d in dirs:
                p = Path(d)
                if p.is_dir():
                    shutil.rmtree(p)

            files = get_debs(Path.cwd())
            for f in files:
                f.unlink()

        @pkg.command(name="install-deps")
        def install_deps(): # type: ignore reportUnusedFunction
            """Installs rosdeps"""
            get_pkg_env()
            pkg_dir = Path.cwd()
            call("sudo apt-get update")
            call("rosdep update")
            call(f"rosdep install -y --rosdistro {get_ros_distro()} --from-paths {pkg_dir} -i")

        @pkg.command(name="get-sources")
        def get_sources(): # type: ignore reportUnusedFunction
            """Imports items from the .repo dir"""
            if Path(".repos").is_file():
                call("vcs import --recursive < .repos")
            else:
                raise click.ClickException("No '.repos' file found. Unsure how to obtain sources.")

        @pkg.command(name="build")
        @click.option('--version', type=str, help="The version to call the debian", required=True)
        @click.option('--no-tests', type=bool, default=True)
        def build(version: str, no_tests: bool): # type: ignore reportUnusedFunction
            """Builds the package using bloom"""


            version_semver, version_prerelease = parse_version(version)
            echo(f"Updating package.xml version to {version_semver}", "blue")
            # This will replace anything between the <version></version> tags in the package.xml
            call(f'sed -i \":a;N;\\$!ba; s|<version>.*<\\/version>|<version>{version_semver}<\\/version>|g" package.xml')

            pkg_name = Path.cwd().name
            pkg_type = "rosdebian"
            src_dir = Path("src")
            bloom_args = ''

            # need to make this more generic
            if src_dir.is_dir():
                pkgs = find_packages(src_dir)
                if pkgs:
                    bloom_args += f' --src-dir={src_dir / pkgs[pkg_name]}'

            if version_prerelease:
                bloom_args += f'-i "{version_prerelease}"'

            if no_tests:
                bloom_args += "--no-tests"

            call(f"bloom-generate {pkg_type} --ros-distro {get_ros_distro()} {bloom_args}")

            cpus = cpu_count() if cpu_count() else 1
            call(f"fakeroot debian/rules binary -j{cpus}")

            # move deb files
            debs = get_debs(Path.cwd().parent)
            if debs:
                for d in debs:
                    shutil.move(str(d), str(Path.cwd()))
            else:
                raise click.ClickException("No debs found.")

            echo("Build complete", "green")

        @pkg.command(name="apt-clone")
        def apt_clone(): # type: ignore reportUnusedFunction
            """Checks out the GR apt repo"""
            call(f"git clone --filter=blob:none {GR_APT_REPO_URL} {GR_APT_REPO_PATH}")

        @pkg.command(name="apt-push")
        def apt_push(): # type: ignore reportUnusedFunction
            """Pushes to the GR apt repo"""
            call(f"git pull --rebase", cwd=GR_APT_REPO_PATH)
            call(f"git push", cwd=GR_APT_REPO_PATH)

        @pkg.command(name="apt-update")
        def apt_update(): # type: ignore reportUnusedFunction
            """Update the GR apt repo"""
            if not GR_APT_REPO_PATH:
                raise click.ClickException("GR apt repo has not been cloned.")
            call(f"git pull --rebase", cwd=GR_APT_REPO_PATH)

        @pkg.command(name="apt-add")
        @click.argument('deb', type=click.Path(exists=True), required=False)
        def apt_add(deb: str): # type: ignore reportUnusedFunction 
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

            call(f"git commit -a -m 'feat: add debian package: {' '.join(d.name for d in debs)}'", cwd=GR_APT_REPO_PATH)
