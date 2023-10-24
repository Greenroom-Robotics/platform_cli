from glob import glob
from typing import List
from pathlib import Path
import click

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import get_env, PkgEnv, echo, call

from python_on_whales import docker


class Workspace(PlatformCliGroup):
    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated with colcon workspaces")
        def ws():
            pass

        @ws.command(name="container")
        # @click.option("--package", type=str, default=None, help="The package to build")
        # @click.option("--debug-symbols", is_flag=True, show_default=True, default=False)
        # @click.option("--no-base", is_flag=True, default=False)
        # @click.argument("args", nargs=-1)
        def container():  # type: ignore
            """Creates a container for the workspace"""

            env = get_env(PkgEnv)
            workspace_path = Path.cwd()
            container_name = f"platform_ws_{workspace_path.name}"

            if docker.container.exists(container_name):
                container = docker.container.inspect(container_name)
                echo(f"Container '{container_name}' already exists - removing", "red")
                container.remove(force=True)

            echo(f"Creating container '{container_name}' for workspace {workspace_path}...", "green")
            container = docker.run("ghcr.io/greenroom-robotics/ros_builder:humble-latest", ["tail", "-f", "/dev/null"],
                name=container_name,
                volumes=[(workspace_path / "src", "/home/ros/ws/src",  "ro"),
                         (Path.home() / "work/platform", "/platform", "rw")],
                workdir="/home/ros/ws",
                envs=env,
                detach=True, remove=False
            )
            echo(f"Container '{container_name}' created", "green")

            container.execute(["sudo", "chown", "ros:ros", "/home/ros/ws"], tty=True)
            container.execute(["pip", "install", "/platform/tools/platform_cli"], tty=True)
            container.execute(["platform", "pkg", "setup"], tty=True)
            container.execute(["platform", "pkg", "refresh-deps"], tty=True)
            
            # how do we source the setup.bash files?
            # container.execute(["colcon", "build"], tty=True)
            # can_adapter@  
            #    platform_can@   platform_control@ 





            # container.stop()
