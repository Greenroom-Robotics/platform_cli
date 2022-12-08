import click
import shutil

from glob import glob
from pathlib import Path
import xml.etree.ElementTree as ET
import json

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import echo, call

PACKAGES_DIRECTORY = "packages"

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
            dest_path_yarn_lock = Path.cwd() / "release.config.js"

            shutil.copyfile(asset_dir / "package.json", dest_path_package_json)
            shutil.copyfile(asset_dir / "yarn.lock", dest_path_yarn_lock)
            shutil.copyfile(asset_dir / "release.config.js", dest_path_yarn_lock)

            call("yarn install --frozen-lockfile")
            
        @release.command(name="create")
        def create(): # type: ignore
            """Creates a release of the platform module"""
            self._generate_package_jsons_for_each_package()
            call("yarn release")
            


