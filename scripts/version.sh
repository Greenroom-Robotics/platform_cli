# Get the version from the args
version=$1

echo "# This is generated by scripts/version.sh as part of the release process
__version__ = \"${version}\""> ./platform_cli/__init__.py