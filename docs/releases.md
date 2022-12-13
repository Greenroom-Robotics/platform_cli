# Releases

## How to create a release

Simply run the release workflow from any platform module repo.


## How releases work

Releases are done using [semantic-release](https://github.com/semantic-release/semantic-release). There are 2 tricky things here... 
1. We want to build and version each package separately which is why we use [multi-semantic-release](https://github.com/qiwi/multi-semantic-release). 
2. We want to build `.deb`s for both **AMD64** and **ARM64** and possibly multiple versions of ROS which is why we use docker to build the `.deb` files. We then need to mount these back to the host machine so they can be linked into a single release.


### Setup:
`platform release setup` is run from the root of a platform module repo. This will:

1. Copy a `package.json`, `yarn.lock` and `release.config.js` into the root of the project.
2. `yarn install` all the nodejs deps (you need to have nodejs installed!)
3. Find all the `package.xml` files in the repo and generate a `packages.json` file next to it (this is because `semantic-release` doesn't support `package.xml` files)
4. The [release.config.js](../platform_cli/assets/release.config.js) config will be used for all of the packages (even though it is used in the root of the project)
5. We **DO NOT** want to commit back the `package.json`, `yarn.lock` or `release.config.js`.

### Creating the release:
`platform release create` is run from the root of a platform module repo. This will:

1. Run `yarn multi-semantic-release` which triggers [multi-semantic-release](https://github.com/qiwi/multi-semantic-release) to run `semantic-release` on each package in the repo. It determines packages by looking for `package.json` files in the repo.
2. From the root of each package it will follow the instructions in [release.config.js](../platform_cli//assets/release.config.js)
3. This includes:
   1. Analysing the commits using `conventionalcommits` to figure out what the next version should be
   2. Generating releases notes for each package based on the commits
   3. Generating a changelog for each package based in the commits
   4. Running [`platform release deb-prepare`](../platform_cli/groups/release.py#:~:text=deb_prepare) which builds the `.deb` in a docker container. 
      1. Sets up `tonistiigi/binfmt` which allows docker to run `arm64` containers on `amd64` machines
      2. Create a local docker registry on [localhost:5000](http://localhost:5000) to store the built images
      3. Uses `buildx` to build for both `amd64` and `arm64` and push to the local registry
      4. Executes [`platform pkg build`](../platform_cli/groups/packaging.py#:~:text=build) inside each docker container to build the `.deb` with a docker volume to mount the resultant `.deb` back to the host machine.
   5. Running [`platform release deb-publish`](../platform_cli/groups/release.py#:~:text=deb_publish) to publish the `.deb` to the apt repo
   6. Uploading the `.deb` to the github release
   7. Commiting back the changes to the `CHANGELOG.md` files
