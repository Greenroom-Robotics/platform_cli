import click

from platform_cli.helpers import check_env, Env
from platform_cli.ros_packages import RosPackages
from platform_cli.poetry_packages import PoetryPackages
from platform_cli.packaging import Packaging



base_groups: list[PlatformCliGroup] = [
    Ros(),
    Poetry(),
    Packaging(),
]

help = f"""
{click.style('Greenroom Platform CLI', bg='green', fg='black', bold=True)}

{click.style('A CLI for common scripts shared between Greenroom platform modules and platform CI.', fg='green', bold=True)}"""

def init_platform_cli(help: str=help, extra_groups: list[PlatformCliGroup] = []):
    """
    This will initialise the platform_cli.
    A list of PlatformCliGroups can be passed in, these will also be initialised as cli_groups
   
    Example:

    class SomeOtherGroup(PlatformCliGroup):
        def create(cli: click.group):
            @cli.group(help="Help for some other CLI group")
            def some_other_group():
                pass

            @poetry.command(name="example")
            def example():
                pass

    init_platform_cli(extra_groups=[SomeOtherGroup()])
    """
    groups = [*base_groups, *extra_groups]

    @click.group(help=help)
    def cli(): # type: ignore
        pass

    # Create all the groups
    for group in groups:
        group.create(cli)

    cli()
    
if __name__ == '__main__':
    init_platform_cli()
