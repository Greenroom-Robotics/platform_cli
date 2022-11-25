from glob import glob
from typing import List
from pathlib import Path
import click
import tempfile

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import get_env, echo, call

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

            env = get_env()
            args_str = " ".join(args)

            echo("Building packages...", 'green')
            call(
                f"colcon build --merge-install --install-base /opt/greenroom/{env['PLATFORM_MODULE']} {args_str}",
            )

                
        @ros.command(name="test")
        @click.argument("args", nargs=-1)
        def test(args: List[str]): # type: ignore
            """Runs colcon test on all ros packages"""

            env = get_env()
            args_str = " ".join(args)

            echo("Testing packages...", 'green')
            call(
                f"colcon test --merge-install --install-base /opt/greenroom/{env['PLATFORM_MODULE']} {args_str}; colcon test-result --all --verbose",
            )

        @ros.command(name="install_poetry_deps")
        @click.option('--base-path', type=str, help="The path to where the packages are installed")
        def install_poetry_deps(base_path: Path): # type: ignore
            """Installs the poetry deps for any python packages"""

            env = get_env()
            base_path = Path(base_path) if base_path else Path(f"/opt/greenroom/{env['PLATFORM_MODULE']}")


            echo(f"Installing all poetry deps in {base_path} using pip", 'green')
            ros_poetry_packages = self._get_ros_poetry_packages(base_path)
            # Disable venv
            call("poetry config virtualenvs.create false")

            for dir in ros_poetry_packages:
                echo(f"Installing {str(dir)}...", "blue")
                # export dependencies to a requirements.txt file without hashes to decrease time to resolve dependencies.
                # https://github.com/python-poetry/poetry-plugin-export/issues/78
                call(f"cd {dir} && poetry export -f requirements.txt --without-hashes --output requirements.txt && pip3 install -r requirements.txt && rm requirements.txt")
       

        @ros.command(name="package_deb")
        @click.option('--version', type=str, help="The version to call the debian", required=True)
        @click.option('--ros-distro', type=str, help="The ros distro", required=True)
        @click.option('--output', type=str, help="The deb output directory", default="output")
        def package_deb(version: str, ros_distro: str, output=str): # type: ignore
            """Packages a debian for the package in the current working directory"""

            # version should be of the form "1.2.3" or "1.2.3-alpha.1"
            version_split = version.split("-")
            version_semver = version_split[0]
            version_prerelease = version_split[1] if len(version_split) == 2 else ""

            echo(f"Building deb for version: {version_semver} {version_prerelease}", 'blue')

            echo(f"Updating package.xml version to {version_semver}", "blue")
            # This will replace anything between the <version></version> tags in the package.xml
            call(f'sed -i \":a;N;\\$!ba; s|<version>.*<\\/version>|<version>{version_semver}<\\/version>|g" package.xml')

            echo(f"Blooming the package...", "blue")
            call(f'bloom-generate rosdebian --ros-distro "{ros_distro}" -i "{version_prerelease}"')

            echo(f"Running fakeroot...", "blue")
            call("fakeroot debian/rules binary -j8")

            echo("Moving deb into the output directory", "blue")
            call(f"mkdir -p {output}")
            call(f"mv ../*.deb {output}")

    
        @ros.command(name="publish_deb")
        @click.option('--input', type=str, help="The deb output directory", default="output/*.deb")
        @click.option('--package-repo', type=str, help="The repo to publish the package to", default="git@github.com:Greenroom-Robotics/packages.git")
        @click.option('--package-repo-dest-folder', type=str, help="The destination folder in the package repo", default="debian")
        @click.option('--package-repo-branch', type=str, help="The branch to publish to", default="main")
        def publish_deb(input: str, package_repo: str, package_repo_dest_folder: str, package_repo_branch: str): # type: ignore
            """Publishes a debian for the package in the current working directory"""

            package_repo_dir = tempfile.mkdtemp()

            echo("Cloning the package repo...", "green")
            call(f'git clone --single-branch --branch {package_repo_branch} "{package_repo}" {package_repo_dir}')

            echo("Copying the .deb into the package repo", "green")
            desination_path_full = f"{package_repo_dir}/{package_repo_dest_folder}"
            call(f'cp -R {input} {desination_path_full}')

            echo("Push the .deb to the package repo", "green")
            # https://$PUBLISH_GIT_SERVER/${GITHUB_REPOSITORY}/commit/${GITHUB_SHA}
            commit_message = f"feat: add debian package"
            call(f'cd {package_repo_dir} && git add . && git commit -m "{commit_message}" && git pull -r && git push -u origin HEAD:"{package_repo_branch}" --force')



