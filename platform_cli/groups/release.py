import click
import shutil

from typing import List
from glob import glob
from pathlib import Path
import xml.etree.ElementTree as ET
import json
from dataclasses import dataclass
from python_on_whales import docker
from python_on_whales.components.buildx.imagetools.models import ImageVariantManifest

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import echo, call

PACKAGES_DIRECTORY = "packages"
DEBS_DIRECTORY = "debs"

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
        echo(f"Looking in ./{PACKAGES_DIRECTORY} for package.xml files", "blue")
        package_xmls = glob(str(Path.cwd() / f"{PACKAGES_DIRECTORY}/**/package.xml"))

        echo(f"Total found: {len(package_xmls)}", "blue")
        for package_xml_path in package_xmls:
            package_xml = ET.parse(package_xml_path)
            root = package_xml.getroot()
            package_name = root.find("name").text  # type: ignore
            package_json = {"name": package_name, "version": "0.0.0", "license": "UNLICENSED"}
            package_json_path = str(Path(package_xml_path).parent / "package.json")

            echo(f"Writing {package_json_path}", "blue")
            f = open(package_json_path, "w")
            f.write(json.dumps(package_json, indent="    "))
            f.close()

    def _get_package_info(self) -> PackageInfo:
        """Returns the package info for the current working directory"""
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

    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated releasing a platform module")
        def release():
            pass
            
        @release.command(name="setup")
        def setup(): # type: ignore
            """Copies the package.json and yarn.lock into the root of the project and installs the deps"""

            echo("Copying package.json and yarn.lock to root and installing deps...", 'blue')
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
        def create(args: List[str]): # type: ignore
            """Creates a release of the platform module package. See release.config.js for more info"""
            args_str = " ".join(args)
            call(f"yarn multi-semantic-release {args_str}")

        @release.command(name="deb-prepare")
        @click.option('--version', type=str, help="The version to call the debian", required=True)
        def deb_prepare(version: str): # type: ignore
            """Prepares the release by building the debian package inside a docker container"""

            package_info = self._get_package_info()

            # Install binfmt support for arm64
            call("docker run --privileged --rm tonistiigi/binfmt --install all")

            # Configure docker to use the platform buildx builder
            try:
                call("docker buildx create --name platform --driver docker-container --bootstrap")
            except: 
                echo("docker buildx already exists", "blue")
            call("docker buildx use platform")

            # Build the images for arm and amd using buildx
            docker_image_name = f"ghcr.io/greenroom-robotics/{package_info.platform_module_name}:latest"
            docker_build_command = [
                "docker buildx build", 
                "--platform linux/amd64,linux/arm64",
                "--build-arg API_TOKEN_GITHUB",
                f"-t {docker_image_name}", 
                "--push", # it seems like we need to push this to a registry :(
                "../../",
            ]
            call(" ".join(docker_build_command))
            
            # Inspect the image to get the manifest
            image_manifests = docker.buildx.imagetools.inspect(docker_image_name)
            # Find the arm64 and amd64 images as we need their digest
            def is_arm64_image(item: ImageVariantManifest):
                return item.platform and item.platform.architecture == "arm64"
            def is_amd64_image(item: ImageVariantManifest):
                return item.platform and item.platform.architecture == "amd64"
            manifests = image_manifests.manifests or []
            image_arm64 = next(manifest for manifest in manifests if is_arm64_image(manifest))
            image_amd64 = next(manifest for manifest in manifests if is_amd64_image(manifest))
    
            # We run the build command in a docker container
            # The volume is mounted so we have access to the created deb file
            def build_deb(platform: str, digest: str):
                return docker.run(
                    f"{docker_image_name}@{digest}", 
                    [f"/bin/bash -c 'source /home/ros/.profile && platform pkg build --version {version} --output {DEBS_DIRECTORY}'"],
                    workdir=f'/home/ros/{package_info.platform_module_name}/{PACKAGES_DIRECTORY}/{package_info.package_name}',
                    volumes=[
                        (package_info.platform_module_path, f"/home/ros/{package_info.platform_module_name}",)
                    ],
                    platform=platform,
                )

            build_deb("linux/amd64", image_amd64.digest)
            build_deb("linux/arm64", image_arm64.digest)

        @release.command(name="deb-publish")
        def deb_publish(): # type: ignore
            """Publishes the deb to the apt repo"""
            try:
                call("platform pkg apt-clone")
            except Exception:
                echo("Apt repo already exists", "yellow")

            
            debs_folder = Path.cwd() / DEBS_DIRECTORY

            call("platform pkg apt-add", cwd=Path(debs_folder))
            call("platform pkg apt-push") 

