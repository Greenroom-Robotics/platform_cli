import click

from platform_cli.groups.base import PlatformCliGroup
from platform_cli.groups.ros import Ros
from platform_cli.groups.poetry import Poetry



# This maintains a list of registerd plugin groups
cli_groups: list[PlatformCliGroup] = [
    Ros(),
    Poetry()
]

def register_cli_group(group: PlatformCliGroup):
    """Call this to register a cli_group"""
    cli_groups.append(group)


def init_platform_cli():
    """
    This will initialise the platform_cli.
    A list of PlatformCliGroups can be passed in, these will also be initialised as cli_groups
   
    Example:

    class ExampleGroup(PlatformCliGroup):
        def create(cli: click.group):
            @cli.group(help="Help for some other CLI group")
            def some_example_group():
                pass

            @poetry.command(name="some_example_command")
            def some_example_command():
                pass

    register_cli_group(ExampleGroup())
    init_platform_cli()
    ```

    """
    help_text = f"""
    {click.style('Greenroom Platform CLI', bg='green', bold=True)}

    {click.style('A CLI for common scripts shared between Greenroom platform modules and platform CI.', fg='green', bold=True)}
    """

    @click.group(help=help_text)
    def cli(): # type: ignore
        pass

    # Create all the groups
    for group in cli_groups:
        group.create(cli)

    cli()
    
if __name__ == '__main__':
    init_platform_cli()
