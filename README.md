# seymour-dev-base

base container tooling for ros development

the `seymour-dev-base` image built by this repository in intended to be
used as a base image for developing specific ros packages

for example to create a new container based on this image:

```Containerfile
FROM ghcr.io/freshrobotics/seymour-dev-base

# your custom setup here ...

```

in memory of [seymour papert](https://en.wikipedia.org/wiki/Seymour_Papert)


## dependencies

* ubuntu linux (22.04+)
* [podman](https://podman.io/)
    - `$ sudo apt install podman`
* [just](https://github.com/casey/just)
    - `$ sudo apt install just` (on ubuntu 24.04+)


## developing

at a linux bash terminal run:

* `$ just build-image` to build the base container image
* `$ just run` to run the container and get a shell


## multiarch support

docker can be configured to use [qemu](https://www.qemu.org/) to emulate system
architectures that differ from the current system architecture

to install qemu multiarch support on an ubuntu system run:

* `$ just setup-multiarch`

the `build-image` task will by default build linux/amd64 and linux/arm64 images
and add them to the current image manifest
