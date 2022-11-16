from pathlib import Path
from typing import List, Dict
import click
from psutil import cpu_count
import shutil
import os

from platform_cli.helpers import call, stdout_call, get_project_root, check_gr_auth

GR_APT_REPO_URL = "git@github.com:Greenroom-Robotics/packages.git"
GR_APT_REPO_PATH = Path(".git/gr-packages")

GR_APT_REPO_PATH_USER = Path.home() / ".gr/gr-packages"

def get_ros_distro():
    return os.environ['ROS_DISTRO']


def get_debs(p: Path) -> List[Path]:
    return list(p.glob("*.deb")) + list(p.glob("*.ddeb"))


def find_packages(p: Path) -> Dict[str, str]:
    # hack city.. hack hack city bitch
    pkgs = {k: v for k, v in zip(stdout_call(f"colcon list -n", cwd=p).split("\n"), stdout_call(f"colcon list -p", cwd=p).split("\n")) if k and v}
    return pkgs


class Packaging:
    def __init__(self, cli: click.Group):
        @cli.group(help="Packaging commands")
        def pkg():
            self.apt_repo_path = None
            prj_root = get_project_root()

            if prj_root:
                self.apt_repo_path = get_project_root() / GR_APT_REPO_PATH
                if not self.apt_repo_path.is_dir():
                    self.apt_repo_path = None

            if self.apt_repo_path is None:
                if GR_APT_REPO_PATH_USER.is_dir():
                    self.apt_repo_path = GR_APT_REPO_PATH_USER

        @pkg.command(name="setup")
        def setup():
#     curl -s https://$GHCR_PAT@raw.githubusercontent.com/Greenroom-Robotics/rosdistro/main/scripts/setup-rosdep.sh | bash -s
#     curl -s https://$GHCR_PAT@raw.githubusercontent.com/Greenroom-Robotics/packages/main/scripts/setup-apt.sh | bash -s
            call("apt-get update", sudo=True)
            call("rosdep init", sudo=True, abort=False)
            call("rosdep update")

        @pkg.command(name="clean")
        def clean():
            dirs = ['.obj-x86_64-linux-gnu', 'debian', 'log']

            for d in dirs:
                p = Path(d)
                if p.is_dir():
                    shutil.rmtree(p)

            files = get_debs(Path.cwd())
            for f in files:
                f.unlink()

        @pkg.command(name="install-deps")
        def install_deps():
            check_gr_auth()
            pkg_dir = Path.cwd()
            call("sudo apt-get update")
            call("rosdep update")
            call(f"rosdep install -y --rosdistro {get_ros_distro()} --from-paths {pkg_dir} -i")

        @pkg.command(name="get-sources")
        def get_sources():
            if Path(".repos").is_file():
                call("vcs import --recursive < .repos")
            else:
                raise click.ClickException("No '.repos' file found. Unsure how to obtain sources.")

        @pkg.command(name="build")
        def build():
            """Builds the package using bloom"""

            pkg_name = Path.cwd().name
            pkg_type = "rosdebian"
            src_dir = Path("src")
            bloom_args = ''

            # need to make this more generic
            if src_dir.is_dir():
                pkgs = find_packages(src_dir)
                if pkgs:
                    bloom_args += f' --src-dir={src_dir / pkgs[pkg_name]}'

            # if prerelease:
            #     bloom-generate rosdebian --ros-distro "$ROS2_DISTRO" -i "$VERSION_PRERELEASE" $BLOOM_GENERATE_EXTRA_ARGS || exit $?

            call(f"bloom-generate {pkg_type} --ros-distro {get_ros_distro()} --no-tests{bloom_args}")

            cpus = cpu_count() if cpu_count() else 1
            call(f"fakeroot debian/rules binary -j{cpus}")

            # move deb files
            debs = get_debs(Path.cwd().parent)
            if debs:
                for d in debs:
                    shutil.move(str(d), str(Path.cwd()))
            else:
                raise click.ClickException("No debs found.")

        @pkg.command(name="apt-clone")
        def clone_apt():
            """Checks out the GR apt repo"""
            call(f"git clone --filter=blob:none {GR_APT_REPO_URL} {GR_APT_REPO_PATH}", project_root_cwd=True)

        @pkg.command(name="apt-push")
        def push_apt():
            """Pushes to the GR apt repo"""
            call(f"git pull --rebase", cwd=self.apt_repo_path)
            call(f"git push", cwd=self.apt_repo_path)

        @pkg.command(name="apt-update")
        def update_apt():
            """Update the GR apt repo"""
            if not self.apt_repo_path:
                raise click.ClickException("GR apt repo has not been cloned.")
            call(f"git pull --rebase", cwd=self.apt_repo_path)

        @click.argument('deb', type=click.Path(exists=True), required=False)
        @pkg.command(name="apt-add")
        def add_apt(deb):
            if not self.apt_repo_path:
                raise click.ClickException("GR apt repo has not been cloned.")

            if deb:
                debs = [Path(deb)]
            else:
                debs = get_debs(Path.cwd())

            if not debs:
                raise click.ClickException("No debs found.")
            for d in debs:
                shutil.copy(d, self.apt_repo_path / "debian")
                call(f"git add debian/{d.name}", cwd=self.apt_repo_path)

            call(f"git commit -a -m 'feat: add debian package: {' '.join(d.name for d in debs)}'", cwd=self.apt_repo_path)
