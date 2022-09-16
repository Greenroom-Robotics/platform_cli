# Platform CLI

A CLI for common commands shared between Greenroom Platform Modules and Platform CI

## Development

* `pip install -e .` to install in edit mode
* `platform` to test it

## Usage

* `pip install git+https://github.com/Greenroom-Robotics/platform_cli.git@main` to install from a tag/branch
* `platform` to use.

### Requirements

Please export the following environment variables:
* `PLATFORM_MODULE` eg) `platform_perception`
* `ROS_OVERLAY` eg) `/opt/ros/galactic`

### Commands

#### platform ros
```
Usage: platform ros [OPTIONS] COMMAND [ARGS]...

  Commands for ROS packages

Options:
  --help  Show this message and exit.

Commands:
  build                Runs colcon build on all ros packages
  install_poetry_deps  Installs the poetry deps for any python packages
  test                 Runs colcon test on all ros packages
```

#### platform poetry
```
Usage: platform poetry [OPTIONS] COMMAND [ARGS]...

  Commands for pure poetry packages

Options:
  --help  Show this message and exit.

Commands:
  install  Runs poetry install on all poetry packages
  test     Runs pytest on all poetry packages
```