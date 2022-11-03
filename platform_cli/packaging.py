from pathlib import Path
from typing import List
import click
from psutil import cpu_count
import shutil
import os


from platform_cli.helpers import call


def get_debs(p: Path) -> List[Path]:
    return list(p.glob("*.deb")) + list(p.glob("*.ddeb"))


class Packaging:
    def __init__(self, cli: click.Group):
        @cli.group(help="Packaging commands")
        def pkg():
            pass

        @pkg.command(name="clean")
        def clean():
            dirs = ['.obj-x86_64-linux-gnu', 'debian']

            for d in dirs:
                p = Path(d)
                if p.is_dir():
                    shutil.rmtree(p)

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

            call(f"bloom-generate {pkg_type} --ros-distro {os.environ['ROS_DISTRO']} --no-tests{bloom_args}")

            cpus = cpu_count() if cpu_count() else 1
            call(f"fakeroot debian/rules binary -j{cpus}")

            # move deb files
            debs = get_debs(Path.cwd().parent)
            if debs:
                for d in debs:
                    shutil.move(d, Path.cwd())
            else:
                raise click.ClickException("No debs found found.")

        @pkg.command(name="add")
        def add_to_repo():
            # call("git clone --filter=blob:none git@github.com:Greenroom-Robotics/packages.git .git/gr-packages", project_root_cwd=True)
            call(f"git add debian/{pkg}")
            call(f"git commit -a -m 'feat: add debian package: {pkg}'")
