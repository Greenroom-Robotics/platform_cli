# Platform CLI

![Platform_CLI](docs/assets/platform_cli.png)

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

#### `platform ros`
```
build                Runs colcon build on all ROS package
install_poetry_deps  Installs the poetry deps for any python packages
test                 Runs colcon test on all ROS packages
```

#### `platform poetry`
```
install  Runs poetry install on all poetry packages
test     Runs pytest on all poetry packages
```

#### `platform pkg`

```bash
apt-add       Adds a .deb to the GR apt repo
apt-clone     Checks out the GR apt repo
apt-push      Pushes to the GR apt repo
apt-update    Update the GR apt repo
build         Builds the package using bloom
clean         Removes debians and log directories
get-sources   Imports items from the .repo file
install-deps  Installs rosdeps
refresh-deps  Installs rosdeps
setup         Sets up the greenroom apt and rosdep lists
```

#### `platform release`

```bash
create       Creates a release of the platform module package.
deb-prepare  Prepares the release by building the debian package inside a docker container
deb-publish  Publishes the deb to the apt repo
setup        Copies the package.json and yarn.lock into the root of the project and installs the deps
```

See [releases](./docs/releases.md) for more into on how `platform release create` works.

### double-dash support

Many commands support `--` in order to pipe args to the internal tools. For example,

```bash
platform ros build -- --help # will list colcon --help rather than the plaform cli's
# or
platform ros build -- --packages-select some_package # to pass directly to colcon
```
