# Platform CLI

A CLI for common commands shared between Greenroom Platform Modules and Platform CI

## Development

* `pip install -e .` to install in edit mode
* `python -m platform_cli` to test it

## Usage

* `pip install git+https://github.com/Greenroom-Robotics/platform_cli.git@main` to install from a tag/branch
* `python -m platform_cli` to use.

### Requirements

Please export the following environment variables:
* `PLATFORM_MODULE` eg) `platform_perception`
* `ROS_OVERLAY` eg) `/opt/ros/galactic`
