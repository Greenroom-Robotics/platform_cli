import click
import shutil

import os
from typing import List
from glob import glob
from pathlib import Path
import xml.etree.ElementTree as ET
import json
from dataclasses import dataclass
from python_on_whales import docker
from python_on_whales.components.buildx.imagetools.models import Manifest

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import echo, call

PACKAGES_DIRECTORY = "packages"
DEBS_DIRECTORY = "debs"
DOCKER_REGISTRY = "localhost:5000"


@dataclass
class PackageInfo():
    package_path: Path
    package_name: str
    platform_module_path: Path
    platform_module_name: str


class Release(PlatformCliGroup):
    def _generate_package_jsons_for_each_package(self):
        """
        This will generate a fake package.json next to any package.xml.
        This is done as a hack so semantic-release can be used to release the package.
        """
        echo(
            f"Looking in ./{PACKAGES_DIRECTORY} for package.xml files", "blue")
        package_xmls = glob(
            str(Path.cwd() / f"{PACKAGES_DIRECTORY}/**/package.xml"))

        echo(f"Total found: {len(package_xmls)}", "blue")
        for package_xml_path in package_xmls:
            package_xml = ET.parse(package_xml_path)
            root = package_xml.getroot()
            package_name = root.find("name").text  # type: ignore
            package_json = {"name": package_name,
                            "version": "0.0.0", "license": "UNLICENSED"}
            package_json_path = str(
                Path(package_xml_path).parent / "package.json")

            echo(f"Writing {package_json_path}", "blue")
            f = open(package_json_path, "w")
            f.write(json.dumps(package_json, indent="    "))
            f.close()

    def _get_package_info(self) -> PackageInfo:
        """
        Returns the package info for the current working directory.
        This assumes the cwd is a package. It will find out the name of the platform module.

        File structure should be:
        platform_module/packages/package_name

        eg)
        platform_notifications/packages/notification_msgs
        """
        package_path = Path.cwd()
        package_name = package_path.name
        platform_module_path = (Path.cwd() / "../..").resolve()
        platform_module_name = platform_module_path.name

        return PackageInfo(
            package_path=package_path,
            package_name=package_name,
            platform_module_path=platform_module_path,
            platform_module_name=platform_module_name,
        )

    def _build_deb_in_docker(
        self,
        version: str,
        platform_module_name: str,
        platform_module_path: Path,
        package_name: str,
        docker_image_name: str,
        architecture: str,
        image_manifests: Manifest
    ):
        """
        Runs the build command in a docker container
        The volume is mounted so we have access to the created deb file
        """

        manifests = image_manifests.manifests or []

        # Find the image for the platform and archicture
        image_for_docker_platform = next(manifest for manifest in manifests if manifest.platform and manifest.platform.architecture == architecture)
        echo(f"Docker image manifest {image_for_docker_platform}", "blue")
        docker_plaform = f"linux/{architecture}"

        host_debs_path = platform_module_path / \
            PACKAGES_DIRECTORY / package_name / DEBS_DIRECTORY
        docker_debs_path = f"/home/ros/{platform_module_name}/{PACKAGES_DIRECTORY}/{package_name}/{DEBS_DIRECTORY}"

        # Make the .debs directory on the host, otherwise docker will make it with root permissions!
        host_debs_path.mkdir(exist_ok=True)
        # Make the .debs directory writable by all users
        os.chmod(host_debs_path, 0o777)

        return docker.run(
            f"{docker_image_name}@{image_for_docker_platform.digest}",
            ["/bin/bash", "-c", f"source /home/ros/.profile && platform pkg build --version {version} --output {DEBS_DIRECTORY} && platform pkg clean"],
            interactive=True,
            workdir=f'/home/ros/{platform_module_name}/{PACKAGES_DIRECTORY}/{package_name}',
            volumes=[
                # We only mount the /debs directory for each package
                (host_debs_path, docker_debs_path)
            ],
            platform=docker_plaform,
        )

    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated releasing a platform module")
        def release():
            pass

        @release.command(name="setup")
        def setup():  # type: ignore
            """Copies the package.json and yarn.lock into the root of the project and installs the deps"""

            echo(
                "Copying package.json and yarn.lock to root and installing deps...", 'blue')
            asset_dir = Path(__file__).parent.parent / "assets"

            dest_path_package_json = Path.cwd() / "package.json"
            dest_path_yarn_lock = Path.cwd() / "yarn.lock"
            dest_path_release_config = Path.cwd() / "release.config.js"

            shutil.copyfile(asset_dir / "package.json", dest_path_package_json)
            shutil.copyfile(asset_dir / "yarn.lock", dest_path_yarn_lock)
            shutil.copyfile(asset_dir / "release.config.js", dest_path_release_config)

            self._generate_package_jsons_for_each_package()

            call("yarn install --frozen-lockfile")

        @release.command(name="create")
        @click.argument("args", nargs=-1,)
        def create(args: List[str]):  # type: ignore
            """Creates a release of the platform module package. See release.config.js for more info"""
            args_str = " ".join(args)
            call(f"yarn multi-semantic-release {args_str}")

        @release.command(name="deb-prepare")
        @click.option('--version', type=str, help="The version to call the debian", required=True)
        @click.option('--arch', type=str, help="The archictecture to build for. OS will be linux. eg) linux/{architecture}", default=["amd64", "arm64"], multiple=True)
        def deb_prepare(version: str, arch: List[str]):  # type: ignore
            """Prepares the release by building the debian package inside a docker container"""
            docker_platforms = [
                f"linux/{architecture}" for architecture in arch]
            echo(f"Preparing .deb for {arch}", "blue")

            if ("API_TOKEN_GITHUB" not in os.environ):
                raise Exception(f"API_TOKEN_GITHUB must be set")

            package_info = self._get_package_info()
            docker_image_name = f"{DOCKER_REGISTRY}/{package_info.platform_module_name}:latest"

            # Install qemu binfmt support for other architectures
            docker.run("multiarch/qemu-user-static", ["--reset", "-p", "yes", "--credential", "yes"], privileged=True, remove=True)

            # Start a local registry on port 5000
            try:
                docker.run("registry:2", publish=[(5000, 5000)], detach=True, name="registry", remove=True)
            except Exception as e:
                echo(f"Local registry already running: {e}", "yellow")

            try:
                # Configure docker to use the platform buildx builder
                # Network host is required for the local registry to work
                docker.buildx.create(
                    name="platform", driver="docker-container", use=True, driver_options={"network": "host"})
            except:
                echo("docker buildx environment already exists", "yellow")
                echo("Consider running `docker buildx rm platform` if you want to reset the build environment", "yellow")

            docker.buildx.use("platform")

            docker_file_exists = os.path.isfile("../../Dockerfile")
            if not docker_file_exists:
                echo(f"Dockerfile does not exist in {package_info.platform_module_path}. This must be run from a package directory.", "red")
                exit(1)

            # Build the images for arm and amd using buildx
            docker.buildx.build(
                "../../",
                platforms=docker_platforms,
                tags=[docker_image_name],
                build_args={
                    "API_TOKEN_GITHUB": os.environ["API_TOKEN_GITHUB"],
                },
                output={
                    "type": "registry"
                },
            )
     
            # Inspect the image to get the manifest
            image_manifests = docker.buildx.imagetools.inspect(
                docker_image_name)

            for architecture in arch:
                echo(f"Building .deb for {architecture}", "blue")
                try:
                    self._build_deb_in_docker(
                        version=version,
                        platform_module_name=package_info.platform_module_name,
                        platform_module_path=package_info.platform_module_path,
                        package_name=package_info.package_name,
                        docker_image_name=docker_image_name,
                        architecture=architecture,
                        image_manifests=image_manifests,
                    )
                except Exception as e:
                    echo(f"Failed to build .deb for {architecture}", "red")
                    raise e

        @release.command(name="deb-publish")
        def deb_publish():  # type: ignore
            """Publishes the deb to the apt repo"""
            try:
                call("platform pkg apt-clone")
            except Exception:
                echo("Apt repo already exists", "yellow")

            debs_folder = Path.cwd() / DEBS_DIRECTORY

            call("platform pkg apt-add", cwd=Path(debs_folder))
            call("platform pkg apt-push")
