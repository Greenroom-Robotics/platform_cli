from glob import glob
from typing import List, Optional
from pathlib import Path
from tempfile import NamedTemporaryFile
import click

from git import Repo
from platform_cli.groups.release import find_packages
from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import get_env, echo, call

from python_on_whales import docker

def get_system_platform_path() -> Path:
    return Path.home() / "work/platform"


class Workspace(PlatformCliGroup):
    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated with colcon workspaces")
        def ws():
            pass

        @ws.command(name="setup-proxy")
        @click.argument("container_name", type=str)
        def base_image():

            setup_proxy().callback()
            
        
        @ws.command(name="setup-proxy")
        @click.argument("container_name", type=str)
        def setup_proxy(container_name: str):  # type: ignore
            """Sets up the proxy for the workspace container"""

            if docker.container.exists(container_name):
                container = docker.container.inspect(container_name)
            else:
                echo(f"Container '{container_name}' not found", "red")
                return

            proxy_ip = "172.17.0.1"
            proxy_port = "3142"

            proxy_conf = f"""Acquire {{
    HTTP::proxy "http://{proxy_ip}:{proxy_port}";
    HTTPS::proxy "http://{proxy_ip}:{proxy_port}";
}}"""

            with NamedTemporaryFile() as f:
                f.write(proxy_conf.encode())
                f.flush()
                container.copy_to(f.name, "/etc/apt/apt.conf.d/01proxy")

        @ws.command(name="container")
        @click.option("--base-image", type=str, default=None, help="The base image to use")
        @click.option("--path", type=str, multiple=True, help="Paths to compile in the workspace")
        # @click.option("--debug-symbols", is_flag=True, show_default=True, default=False)
        # @click.option("--no-base", is_flag=True, default=False)
        # @click.argument("args", nargs=-1)
        def container(base_image: Optional[str], path: List[str]):  # type: ignore
            """Creates a container for the workspace"""

            system_platform_path = get_system_platform_path()
            container_platform_path = Path("/platform")
            container_home_path = Path("/home/ros")
            workspace_path = Path.cwd()
            container_name = f"platform_ws_{workspace_path.name}"

            base_image = base_image if base_image else "ghcr.io/greenroom-robotics/ros_builder:humble-latest"

            if docker.container.exists(container_name):
                container = docker.container.inspect(container_name)
                echo(f"Container '{container_name}' already exists - removing", "red")
                container.remove(force=True)

            echo(f"Creating container '{container_name}' for workspace {workspace_path}...", "green")

            # workspace_volume = docker.volume.create(f"{container_name}_src", "local",
            #                                         options={
            #                                             "type": "overlay",
            #                                             "o": f"""lowerdir={workspace_path / "src"},upperdir={workspace_path / "workdir"},workdir={workspace_path / "tmp"}""",
            #                                             "device": "overlay"
            #                                         })

            container = docker.run(base_image,
                                   ["tail", "-f", "/dev/null"],
                name=container_name,
                volumes=[
                # (workspace_path / "src", "/home/ros/ws/src",  "ro"),
                         (system_platform_path, container_platform_path, "ro"),
                        #   (workspace_volume, "/ws_src")
                          ],
                workdir=container_home_path,
                # envs=env,
                detach=True, remove=False
            )

            echo(f"Container '{container_name}' created", "green")
            container.execute(["mkdir", "-p", "ws/src"], tty=True)  # , "chown", "ros:ros", "/home/ros/ws"

            for p in [Path(x) for x in path]:
                p_rel = container_platform_path / p.relative_to(system_platform_path)
                container.execute(["ln", "-s", str(p_rel), "ws/src"], tty=True)

            container.execute(["pip", "install", str(container_platform_path / "tools/platform_cli")], tty=True)
            container.execute(["platform", "pkg", "setup"], tty=True)
            # container.execute(["platform", "pkg", "refresh-deps"], tty=True)
            
            # how do we source the setup.bash files? needs to be ran as bash -l -c COMMAND
            # container.execute(["colcon", "build"], tty=True)

            # container.stop()

        @ws.command(name="symlink")
        @click.option("--package", type=str, multiple=True)
        def symlink(package: List[str]):  # type: ignore
            workspace_path = Path.cwd()
            platform_packages = find_packages(get_system_platform_path() / "packages")

            for p in package:
                if p not in platform_packages:
                    echo(f"Package '{p}' not found", "red")
                    return

                if (workspace_path / "src" / p).exists():
                    echo(f"Package '{p}' already exists in workspace", "yellow")
                    continue

                echo(f"Adding {p} to workspace as a symlink", "green")
                (workspace_path / "src" / p).symlink_to(platform_packages[p].package_path)

        @ws.command(name="versions")
        # @click.argument("package", type=str)
        def versions():  # type: ignore
            """Gets the versions of the packages in the workspace"""

            workspace_path = Path.cwd()
            packages = find_packages(workspace_path / "src")

            print(packages)
            for name, info in packages.items():
                if info.module_info is None:
                    continue

                repo = Repo(info.module_info.platform_module_path)
                for tag in repo.tags:
                    if tag.name.startswith(name):
                        print(tag)

        @ws.command(name="build-pkg")
        @click.option(
            "--package",
            type=str,
            help="Which package should we build.",
        )
        @click.option(
            "--version",
            type=str,
            help="The version number to assign to the deb",
            required=False,
            default="",
        )
        # add a flag to skip rosdep
        @click.option("--no-rosdep", type=bool, is_flag=True, default=False)
        def build_pkg(package: str, version: str, no_rosdep: bool):  # type: ignore
            """Builds a package in the workspace"""

            workspace_path = Path.cwd()
            container_ws = Path("/home/ros/ws")
            packages = find_packages(workspace_path / "src")
            container_name = f"platform_ws_{workspace_path.name}"
            container = docker.container.inspect(container_name)

            if package not in packages:
                echo(f"Package '{package}' not found", "red")
                return
            
            rel_path = packages[package].package_path.relative_to(workspace_path)

            container.execute(["platform", "pkg", "clean"], tty=True, workdir=container_ws / rel_path)
            if not no_rosdep:
                container.execute(["platform", "pkg", "install-deps"], tty=True, workdir=container_ws / rel_path)
            container.execute(["platform", "pkg", "build", "--version", version], tty=True, workdir=container_ws / rel_path)
