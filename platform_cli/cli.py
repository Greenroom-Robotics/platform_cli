"""Console script for platform_cli."""

import click


@click.command()
def main():
    """Main entrypoint."""
    click.echo("platform_cli")
    click.echo("=" * len("platform_cli"))
    click.echo("Skeleton project created by Cookiecutter PyPackage")


if __name__ == "__main__":
    main()  # pragma: no cover
