from pathlib import Path
from typing import List
import click
from psutil import cpu_count
import shutil
import os

from platform_cli.helpers import call, get_project_root

GR_APT_REPO_URL = "git@github.com:Greenroom-Robotics/packages.git"
GR_APT_REPO_PATH = Path(".git/gr-packages")

def get_ros_distro():
    return os.environ['ROS_DISTRO']


def get_debs(p: Path) -> List[Path]:
    return list(p.glob("*.deb")) + list(p.glob("*.ddeb"))


class Packaging:
    def __init__(self, cli: click.Group):
        @cli.group(help="Packaging commands")
        def pkg():
            self.apt_repo_path = get_project_root() / GR_APT_REPO_PATH
            if not self.apt_repo_path.is_dir():
                self.apt_repo_path = None

        @pkg.command(name="clean")
        def clean():
            dirs = ['.obj-x86_64-linux-gnu', 'debian']

            for d in dirs:
                p = Path(d)
                if p.is_dir():
                    shutil.rmtree(p)

        @pkg.command(name="install-deps")
        def install_deps():
            call("rosdep update")
            call(f"rosdep install -y --rosdistro {get_ros_distro()} --from-paths $PACKAGES_DIRECTORY -i")

        @pkg.command(name="get-sources")
        def get_sources():
            if Path(".repos").is_file():
                call("vcs import --recursive < .repos")
            else:
                raise click.ClickException("No '.repos' file found. Unsure how to obtain sources.")

        @pkg.command(name="build")
        def build():
            """Builds the package using bloom"""

            pkg_type = "debian"

            bloom_args = ''

            # need to make this more generic
            if Path("src").is_dir():
                src_dir = "src/"
                bloom_args += f' --src-dir={src_dir}'

            # if prerelease:
            #     bloom-generate rosdebian --ros-distro "$ROS2_DISTRO" -i "$VERSION_PRERELEASE" $BLOOM_GENERATE_EXTRA_ARGS || exit $?

            call(f"bloom-generate {pkg_type} --ros-distro {get_ros_distro()} --no-tests{bloom_args}")

            cpus = cpu_count() if cpu_count() else 1
            call(f"fakeroot debian/rules binary -j{cpus}")

            # move deb files
            debs = get_debs(Path.cwd().parent)
            if debs:
                for d in debs:
                    shutil.move(d, Path.cwd())
            else:
                raise click.ClickException("No debs found.")

        @pkg.command(name="apt-clone")
        def clone_apt():
            """Checks out the GR apt repo"""
            call(f"git clone --filter=blob:none {GR_APT_REPO_URL} {GR_APT_REPO_PATH}", project_root_cwd=True)

        @pkg.command(name="apt-push")
        def clone_apt():
            """Pushes to the GR apt repo"""
            call(f"git push", cwd=self.apt_repo_path)

        @pkg.command(name="apt-update")
        def update_apt():
            """Update the GR apt repo"""
            if not self.apt_repo_path:
                raise click.ClickException("GR apt repo has not been cloned.")
            call(f"git pull --rebase", cwd=self.apt_repo_path)

        @pkg.command(name="apt-add")
        def add_apt():
            if not self.apt_repo_path:
                raise click.ClickException("GR apt repo has not been cloned.")

            debs = get_debs(Path.cwd())
            if not debs:
                raise click.ClickException("No debs found.")
            for d in debs:
                shutil.copy(d, self.apt_repo_path / "debian")
                call(f"git add debian/{d.name}", cwd=self.apt_repo_path)

            call(f"git commit -a -m 'feat: add debian package: {' '.join(d.name for d in debs)}'", cwd=self.apt_repo_path)
