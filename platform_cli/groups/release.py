import click
import shutil

import os
from typing import List
from enum import Enum
from glob import glob
from pathlib import Path
import xml.etree.ElementTree as ET
import json
from dataclasses import dataclass
from python_on_whales import docker
from python_on_whales.components.buildx.imagetools.models import Manifest

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import echo, call, LogLevels

DEBS_DIRECTORY = "debs"
DOCKER_REGISTRY = "localhost:5000"


class ReleaseMode(Enum):
    SINGLE = "SINGLE"
    MULTI = "MULTI"


@dataclass
class PackageInfo:
    package_path: Path
    package_name: str
    platform_module_path: Path
    platform_module_name: str


@dataclass
class ModuleInfo:
    platform_module_path: Path
    platform_module_name: str


def get_releaserc(changelog: bool, public: bool = False):
    RELEASERC = {
        "branches": ["main", {"name": "alpha", "prerelease": True}],
        "plugins": [
            ["@semantic-release/commit-analyzer", {"preset": "conventionalcommits"}],
            [
                "@semantic-release/release-notes-generator",
                {"preset": "conventionalcommits"},
            ],
            "@semantic-release/changelog",
            [
                "@semantic-release/exec",
                {
                    "prepareCmd": "platform release deb-prepare --version ${nextRelease.version}",
                    "publishCmd": f"platform release deb-publish --public {public}",
                },
            ],
            [
                "@semantic-release/github",
                {"assets": [{"path": "**/*.deb"}], "successComment": False},
            ],
        ],
    }
    if changelog:
        return {
            **RELEASERC,
            "plugins": [
                *RELEASERC["plugins"],
                ["@semantic-release/git", {"assets": ["CHANGELOG.md"]}],
            ],
        }

    return RELEASERC


class Release(PlatformCliGroup):
    def get_package_xmls(self):
        """Returns all package.xmls ignoring those inside node_modules"""
        return [Path(p) for p in Path.cwd().glob("**/package.xml") if "node_modules" not in str(p)]

    def _write_root_package_json(self, src: Path, package_jsons: List[Path]):
        """Writes the root package.json file"""
        dest = Path.cwd() / "package.json"
        with open(src) as f:
            # Set the package_name to be the package.xml package name or the name of the cwd
            package_json = json.load(f)
            package_xml_path = Path.cwd() / "package.xml"
            package_name = (
                self._get_package_name_from_package_xml(package_xml_path)
                if package_xml_path.exists()
                else Path.cwd().name
            )
            # The package name is the name of the package.xml
            package_json["name"] = package_name
            # The workspaces are the parent directories of the package.jsons
            package_json["workspaces"] = [str(p.parent) for p in package_jsons]
            with open(dest, "w") as f:
                json.dump(package_json, f, indent=4)

    def _write_docker_file(self, asset_dir: Path, dest_dir: Path, release_mode: ReleaseMode):
        """
        Writes the dockerfile to the destination directory
        Select the "single" or "multi" dockerfile based on the release mode
        """
        dockerfile = (
            asset_dir / "Dockerfile.single"
            if release_mode == ReleaseMode.SINGLE
            else asset_dir / "Dockerfile.multi"
        )
        dest = dest_dir / "Dockerfile"
        shutil.copyfile(dockerfile, dest)

    def _write_package_json(self, dest: Path, package_name: str):
        package_json = {
            "name": package_name,
            "version": "0.0.0",
            "license": "UNLICENSED",
        }
        echo(f"Writing {dest}", "blue")
        with open(dest, "w") as f:
            json.dump(package_json, f, indent=4)

    def _write_docker_ignore(self):
        """Write a .dockerignore which ignores node_modules"""
        dest = Path.cwd() / ".dockerignore"
        # If the .dockerignore already exists, check to see if it already ignores node_modules
        if dest.exists():
            with open(dest) as f:
                if "node_modules" in f.read():
                    # It already ignores node_modules, so return
                    return
                # If it doesn't ignore node_modules, append it
                with open(dest, "a") as f:
                    f.write("node_modules")
                return
        # If there is no .dockerignore, create one
        with open(dest, "w") as f:
            f.write("node_modules")
            return

    def _get_release_mode(self) -> ReleaseMode:
        """Returns the release mode for the current working directory"""
        package_xml_path = Path.cwd() / "package.xml"
        if package_xml_path.exists():
            return ReleaseMode.SINGLE
        return ReleaseMode.MULTI

    def _get_docker_image_name(self, platform_module_name: str) -> str:
        """Returns the docker image name for a package"""
        # Note, uppercase is not allowed in docker image names
        return f"{DOCKER_REGISTRY}/{platform_module_name.lower()}:latest"

    def _write_root_yarn_lock(self, src: Path):
        dest = Path.cwd() / "yarn.lock"
        shutil.copyfile(src, dest)

    def _get_package_name_from_package_xml(self, package_xml_path: Path) -> str:
        """Reads the name from a package.xml"""
        package_xml = ET.parse(package_xml_path)
        root = package_xml.getroot()
        package_name: str = root.find("name").text  # type: ignore
        return package_name

    def _generate_package_jsons_for_each_package(self):
        """
        This will generate a fake package.json next to any package.xml.
        This is done as a hack so semantic-release can be used to release the package.
        """
        package_xmls = self.get_package_xmls()

        echo(f"Total found: {len(package_xmls)}", "blue")

        package_jsons: List[Path] = []
        for package_xml_path in package_xmls:
            package_name = self._get_package_name_from_package_xml(package_xml_path)
            package_json_path = Path(package_xml_path).parent / "package.json"
            package_jsons.append(package_json_path)
            if not package_json_path.exists():
                self._write_package_json(package_json_path, package_name)

        return package_jsons

    def _check_parents_for_file(self, filename: str) -> Path:
        """Checks each parent directory for a file"""
        current_path = Path.cwd()
        while current_path.exists():
            file_path = current_path / filename
            if file_path.exists():
                return file_path.parent
            current_path = current_path.parent

        raise Exception(f"Could not find {filename} in any parent directory")

    def _get_package_info(self) -> PackageInfo:
        """
        Returns the package info for the current working directory.
        This assumes the cwd is a package. It will find out the name of the platform module.

        File structure should be something like:
        platform_module/packages/package_name

        eg)
        platform_notifications/packages/notification_msgs
        """
        package_path = Path.cwd()
        package_name = package_path.name
        module_info = self._get_module_info()

        return PackageInfo(
            package_path=package_path,
            package_name=package_name,
            platform_module_path=module_info.platform_module_path,
            platform_module_name=module_info.platform_module_name,
        )

    def _get_module_info(self) -> ModuleInfo:
        """
        Returns the module info for the current working directory.
        """
        platform_module_path = self._check_parents_for_file(".git")
        platform_module_name = platform_module_path.name

        return ModuleInfo(
            platform_module_path=platform_module_path,
            platform_module_name=platform_module_name,
        )

    def _build_deb_in_docker(
        self,
        version: str,
        package_info: PackageInfo,
        docker_image_name: str,
        architecture: str,
        image_manifests: Manifest,
    ):
        """
        Runs the build command in a docker container
        The volume is mounted so we have access to the created deb file
        """
        echo(
            f"Building {package_info.package_name} .deb for {architecture}",
            group_start=True,
        )
        manifests = image_manifests.manifests or []

        # Find the image for the platform and archicture
        image_for_docker_platform = next(
            manifest
            for manifest in manifests
            if manifest.platform and manifest.platform.architecture == architecture
        )
        echo(f"Docker image manifest {image_for_docker_platform}", "blue")
        docker_plaform = f"linux/{architecture}"

        package_relative_to_platform_module = package_info.package_path.relative_to(
            package_info.platform_module_path
        )
        host_debs_path = package_info.package_path / DEBS_DIRECTORY
        docker_working_dir = (
            Path("/home/ros/")
            / package_info.platform_module_name
            / package_relative_to_platform_module
        )
        docker_debs_path = docker_working_dir / DEBS_DIRECTORY

        # Make the .debs directory on the host, otherwise docker will make it with root permissions!
        host_debs_path.mkdir(exist_ok=True)
        # Make the .debs directory writable by all users
        os.chmod(host_debs_path, 0o777)

        docker.run(
            f"{docker_image_name}@{image_for_docker_platform.digest}",
            [
                "/bin/bash",
                "-c",
                f"source /home/ros/.profile && platform pkg build --version {version} --output {DEBS_DIRECTORY} && platform pkg clean",
            ],
            interactive=True,
            workdir=docker_working_dir,
            volumes=[
                # We only mount the /debs directory for each package
                (host_debs_path, docker_debs_path)
            ],
            platform=docker_plaform,
        )
        echo(group_end=True)

    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated releasing a platform module")
        def release():
            pass

        @release.command(name="setup")
        def setup():  # type: ignore
            """Copies the package.json and yarn.lock into the root of the project and installs the deps"""
            echo("Setting up release...", "blue", group_start=True)
            echo(
                "Copying package.json and yarn.lock to root and installing deps...",
                "blue",
            )
            asset_dir = Path(__file__).parent.parent / "assets"

            self._write_docker_ignore()
            self._write_root_yarn_lock(asset_dir / "yarn.lock")
            package_jsons = self._generate_package_jsons_for_each_package()
            self._write_root_package_json(asset_dir / "package.json", package_jsons)

            release_mode = self._get_release_mode()
            module_info = self._get_module_info()

            # If a Dockerfile does not exist in the module root, create it
            docker_file_exists = (module_info.platform_module_path / "Dockerfile").exists()
            echo(f"Dockerfile exists: {docker_file_exists}", "blue")
            if not docker_file_exists:
                echo("Creating Dockerfile...", "blue")
                self._write_docker_file(asset_dir, module_info.platform_module_path, release_mode)

            call("yarn install --frozen-lockfile")
            echo(group_end=True)

        @release.command(name="create")
        @click.option(
            "--changelog",
            type=bool,
            help="Should we publish a CHANGELOG.md back to git",
            default=True,
        )
        @click.option(
            "--public",
            type=bool,
            help="Should this package be published to the public PPA",
            default=False,
        )
        @click.argument(
            "args",
            nargs=-1,
        )
        def create(changelog: bool, public: bool, args: List[str]):  # type: ignore
            """Creates a release of the platform module package. See .releaserc for more info"""
            args_str = " ".join(args)

            # Create the releaserc file
            releaserc = get_releaserc(changelog, public)
            dest_path_releaserc = Path.cwd() / ".releaserc"
            with open(dest_path_releaserc, "w+") as f:
                f.write(json.dumps(releaserc, indent=4))

            # Run the correct release script in the package.json based off the release mode
            release_mode = self._get_release_mode()

            if release_mode == ReleaseMode.SINGLE:
                echo(
                    "Release mode: SINGLE, running semantic-release for root package",
                    "blue",
                )
                call(f"yarn semantic-release {args_str}")
            else:
                echo(
                    "Release mode: MULTI, running multi-semantic-release for root package",
                    "blue",
                )
                call(f"yarn multi-semantic-release {args_str}")

        @release.command(name="deb-prepare")
        @click.option("--version", type=str, help="The version to call the debian", required=True)
        @click.option(
            "--arch",
            type=str,
            help="The archictecture to build for. OS will be linux. eg) linux/{architecture}",
            default=["amd64", "arm64"],
            multiple=True,
        )
        def deb_prepare(version: str, arch: List[str]):  # type: ignore
            """Prepares the release by building the debian package inside a docker container"""
            docker_platforms = [f"linux/{architecture}" for architecture in arch]
            echo(f"Preparing to build .deb for {arch}", "blue")

            if "API_TOKEN_GITHUB" not in os.environ:
                raise Exception("API_TOKEN_GITHUB must be set")

            package_info = self._get_package_info()
            docker_image_name = self._get_docker_image_name(package_info.platform_module_name)

            # Install qemu binfmt support for other architectures
            echo("Setting up QEMU...", group_start=True)
            docker.run(
                "multiarch/qemu-user-static",
                ["--reset", "-p", "yes", "--credential", "yes"],
                privileged=True,
                remove=True,
            )

            # Start a local registry on port 5000
            echo("Setting up local docker registry...", group_start=True, group_end=True)
            try:
                docker.run(
                    "registry:2",
                    publish=[(5000, 5000)],
                    detach=True,
                    name="registry",
                    remove=True,
                )
            except Exception as e:
                echo(f"Local registry already running: {e}", "yellow")

            echo(
                "Building docker container with buildx...",
                group_start=True,
                group_end=True,
            )
            try:
                # Configure docker to use the platform buildx builder
                # Network host is required for the local registry to work
                docker.buildx.create(
                    name="platform",
                    driver="docker-container",
                    use=True,
                    driver_options={"network": "host"},
                )
            except Exception:
                echo("docker buildx environment already exists", "yellow")
                echo(
                    "Consider running `docker buildx rm platform` if you want to reset the build environment",
                    "yellow",
                )

            docker.buildx.use("platform")

            # Build the images for arm and amd using buildx
            docker.buildx.build(
                package_info.platform_module_path,
                platforms=docker_platforms,
                tags=[docker_image_name],
                build_args={
                    "API_TOKEN_GITHUB": os.environ["API_TOKEN_GITHUB"],
                    "PLATFORM_MODULE": package_info.platform_module_name,
                },
                output={"type": "registry"},
            )

            # Inspect the image to get the manifest
            image_manifests = docker.buildx.imagetools.inspect(docker_image_name)
            echo(group_end=True)

            for architecture in arch:
                try:
                    self._build_deb_in_docker(
                        version=version,
                        package_info=package_info,
                        docker_image_name=docker_image_name,
                        architecture=architecture,
                        image_manifests=image_manifests,
                    )
                except Exception as e:
                    echo(
                        f"Failed to build .deb for {architecture}",
                        "red",
                        level=LogLevels.ERROR,
                    )
                    raise e

        @release.command(name="deb-publish")
        @click.option(
            "--public",
            type=bool,
            help="Should this package be published to the public PPA",
            default=False,
        )
        def deb_publish(public: bool):  # type: ignore
            """Publishes the deb to the apt repo"""
            try:
                echo("Publishing .deb to apt repo...", group_start=True)
                call(f"platform pkg apt-clone --public {public}")

                debs_folder = Path.cwd() / DEBS_DIRECTORY

                call("platform pkg apt-add", cwd=Path(debs_folder))
                call("platform pkg apt-push")
                echo(group_end=True)
            except Exception as e:
                echo("Failed to publish .deb", "red", level=LogLevels.ERROR)
                raise e
