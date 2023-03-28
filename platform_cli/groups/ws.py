from glob import glob
from typing import List
from pathlib import Path
from tempfile import NamedTemporaryFile
import click

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import get_env, PkgEnv, echo, call

from python_on_whales import docker


class Workspace(PlatformCliGroup):
    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated with colcon workspaces")
        def ws():
            pass

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

            workspace_volume = docker.volume.create(f"{container_name}_src", "local",
                                                    options={
                                                        "type": "overlay",
                                                        "o": f"""lowerdir={workspace_path / "src"},upperdir={workspace_path / "workdir"},workdir={workspace_path / "tmp"}""",
                                                        "device": "overlay"
                                                    })

            container = docker.run("ghcr.io/greenroom-robotics/ros_builder:humble-latest", ["tail", "-f", "/dev/null"],
                name=container_name,
                volumes=[
                # (workspace_path / "src", "/home/ros/ws/src",  "ro"),
                         (Path.home() / "work/platform", "/platform", "rw"),
                          (workspace_volume, "/ws_src")],
                workdir="/home/ros",
                envs=env,
                detach=True, remove=False
            )
            echo(f"Container '{container_name}' created", "green")
            container.execute(["mkdir", "ws"], tty=True)  # , "chown", "ros:ros", "/home/ros/ws"
            container.execute(["ln", "-s", "/ws_src", "ws/src"], tty=True)
            # container.execute(["sudo", "chown", "ros:ros", "/home/ros/ws"], tty=True)
            container.execute(["pip", "install", "/platform/tools/platform_cli"], tty=True)
            container.execute(["platform", "pkg", "setup"], tty=True)
            # container.execute(["platform", "pkg", "refresh-deps"], tty=True)
            
            # how do we source the setup.bash files?
            # container.execute(["colcon", "build"], tty=True)
            # can_adapter@  
            #    platform_can@   platform_control@ 





            # container.stop()
