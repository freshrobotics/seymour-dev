# seymour-dev

container tooling for ros development

in memory of [seymour papert](https://en.wikipedia.org/wiki/Seymour_Papert)


## dependencies

* linux
* a [recent version of docker](https://docs.docker.com/engine/install/ubuntu/)
* make (`$ sudo apt install build-essential`)

## layout

the `./src/` folder will be mounted into a ros workspace inside the container

place the ros packages you are developing in the `./src/` folder


## developing

at a linux bash terminal run:

* `$ make build` to build the current source packages inside the container
* `$ make run` to run the container and get a shell
* `$ make shell` to get another shell into the running container
* `$ make clean` to remove build artifacts
