from typing import List
from pathlib import Path
import click

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.helpers import echo, call


class Py(PlatformCliGroup):
    def create(self, cli: click.Group):
        @cli.group(help="CLI handlers associated with Python packages")
        def py():
            pass

        @py.command(name="test")
        @click.argument("path", type=Path, default=Path("."))
        @click.argument("args", nargs=-1)
        def test(path: Path, args: List[str]):  # type: ignore
            """Runs pytest on the selected package/path"""
            args_str = " ".join(args)
            echo("Testing packages...", "green")
            call(f"python3 -m pytest {path} -s {args_str}", abort=False)
