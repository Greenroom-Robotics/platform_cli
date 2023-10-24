## [1.2.0](https://github.com/Greenroom-Robotics/platform_cli/compare/v1.1.0...v1.2.0) (2023-10-24)


### Features

* add --build and --watch flags to ros test and launch ([ce05eb6](https://github.com/Greenroom-Robotics/platform_cli/commit/ce05eb620d5fd931f140169994c3523fa71ea759))
* add --sparse option to apt clone and add ([71e3def](https://github.com/Greenroom-Robotics/platform_cli/commit/71e3def7b9f1dfea266c7536d3912ef077e59443))
* add --watch option to `platform ros build` ([14992dd](https://github.com/Greenroom-Robotics/platform_cli/commit/14992dd43b39c35beba1d23c0f2c3a90a44907fc))
* add package-dir argument to release ([63fc640](https://github.com/Greenroom-Robotics/platform_cli/commit/63fc64022615a84d1e3182c7a754848e79e0cdad))
* update Dockerfile to iron ([613a028](https://github.com/Greenroom-Robotics/platform_cli/commit/613a028e77f949d886c906c30f011ed0fb81ac46))


### Bug Fixes

* add exception if API_TOKEN_GITHUB not set ([e4db930](https://github.com/Greenroom-Robotics/platform_cli/commit/e4db9304552011f3bf2a579a005ba82d3d47b15e))
* also upload the debian debugsym ddeb to gh release ([aa89f48](https://github.com/Greenroom-Robotics/platform_cli/commit/aa89f48127fb5ced703408caaffeb585889aa63a))
* do not force push ([880e819](https://github.com/Greenroom-Robotics/platform_cli/commit/880e81975a037c7d6737e2d1a678f0ff20098252))
* don't have colcon create log when finding packages ([2b3f026](https://github.com/Greenroom-Robotics/platform_cli/commit/2b3f026c75ddaddfe960897b92c0a155c43f6998))
* ensure git lfs hooks have been installed ([a7e8278](https://github.com/Greenroom-Robotics/platform_cli/commit/a7e8278d5e34ddc885d629d788d66374227ecf66))
* finding debs when debug_files is false ([42ee412](https://github.com/Greenroom-Robotics/platform_cli/commit/42ee4120acf10523228643c3ceb93578b7ca0b4e))
* use ProcessWatcher to handle `--watch` ([d81b99d](https://github.com/Greenroom-Robotics/platform_cli/commit/d81b99dbdf0fd592d6cdad7e139fb24c1da7a611))

## [1.0.1](https://github.com/Greenroom-Robotics/platform_cli/compare/v1.0.0...v1.0.1) (2022-11-20)


### Bug Fixes

* add __init__.py to git asset list ([fde2453](https://github.com/Greenroom-Robotics/platform_cli/commit/fde24539e8e44e6d497306864f327b9b98949173))
* add --without-hashes to fix https://github.com/python-poetry/poetry/issues/3472 ([93ca1ec](https://github.com/Greenroom-Robotics/platform_cli/commit/93ca1ecd0bc495ef4dfeb62b73e410763c8ae222))
* raise errors on build and test ([05d148c](https://github.com/Greenroom-Robotics/platform_cli/commit/05d148c0f6837cf1c9b5183ffe233ffada5c23c8))

## 1.0.0 (2022-09-18)


### Features

* add entrypoint ([9ce834c](https://github.com/Greenroom-Robotics/platform_cli/commit/9ce834cad55d039ac25dee1daa48b369baff5169))
* add more stuff ([34137b6](https://github.com/Greenroom-Robotics/platform_cli/commit/34137b6fdc8abca034fa5735fba97669f0d79dba))
* add release ci ([8c20153](https://github.com/Greenroom-Robotics/platform_cli/commit/8c201532cbbb950480af4134869bbf058995be6f))
* improve cli with double-dash ([dd62f5a](https://github.com/Greenroom-Robotics/platform_cli/commit/dd62f5a65d89e0f5773cdc1215395f242f2ef0d4))
* initial commit ([7758798](https://github.com/Greenroom-Robotics/platform_cli/commit/775879853f8a38465bcaaf978fad28051d297001))
* make glbober recursive ([ea95821](https://github.com/Greenroom-Robotics/platform_cli/commit/ea9582146ce9dd5b0dffe25c5ef591e8217c50de))
* remove docker build ([5033325](https://github.com/Greenroom-Robotics/platform_cli/commit/50333250dbe30ac215d7543608939890f54c318c))
* simplify ([2a921c7](https://github.com/Greenroom-Robotics/platform_cli/commit/2a921c7928ade04a8c394f51f7db2c1f99d1a9e7))


### Bug Fixes

* add click ([8e2bbcb](https://github.com/Greenroom-Robotics/platform_cli/commit/8e2bbcb014fc32bfd90c60c1cb7fc3a8a212561d))
* always run opt/greenroom/package ([a4a292d](https://github.com/Greenroom-Robotics/platform_cli/commit/a4a292db45b80891754d2febb64bc0c5e96b22bb))
* change to PLATFORM_MODULE ([3c28227](https://github.com/Greenroom-Robotics/platform_cli/commit/3c28227d05a9272e23e64cea654b139a6e80eab0))
* flatten ([3b8906b](https://github.com/Greenroom-Robotics/platform_cli/commit/3b8906bd2214d020fddca929c6447eb7e99eee68))
* maybe ([0184fd8](https://github.com/Greenroom-Robotics/platform_cli/commit/0184fd8e64db2ea78d17f98ce62c5c6339e066b6))
* rm requirements ([4cfba7f](https://github.com/Greenroom-Robotics/platform_cli/commit/4cfba7f1d82f1578a002321d9eb92b698e31b861))
* test only ubuntu ([47dba6b](https://github.com/Greenroom-Robotics/platform_cli/commit/47dba6baa3e2f19887bc5966e9d00268405bb8dc))
* use /bin/bash ([3aed4e0](https://github.com/Greenroom-Robotics/platform_cli/commit/3aed4e0bc75862eac4253faf9ef760a958457e40))
* use pip3 ([f03af69](https://github.com/Greenroom-Robotics/platform_cli/commit/f03af69493ef4e3a5e016aee9f7f8aa9750e3411))
* venv warning in stdout ([a0a8414](https://github.com/Greenroom-Robotics/platform_cli/commit/a0a84140a8117b29a07d46e9c2cc6607d9ea66a0))
