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


## multiarch support

docker can be configured to use [qemu](https://www.qemu.org/) to emulate system
architectures that differ from the current system architecture

to install qemu multiarch support on an ubuntu system run:

* `$ make install-multiarch`

then set the "PLATFORM" in the makefile to the architecture to target

for example to target "linux/arm64" (raspberry pi) from a "linux/amd64" (intel)
system uncomment this line in the makefile:

```
#PLATFORM=linux/arm64
```

and comment out this line:

```
PLATFORM=linux/amd64
```

to build and run the docker image in emulation:

* `$ make build`
* `$ make run`


## selecting a dds implementation

both cyclonedds and fastrtps are installed in the container and
either can be selected at runtime

by default the development container is configured to use cyclonedds

to select fastrtps instead uncomment the following line in the makefile:

```
#RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
```

and comment out this line:

```
RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
```

now when you start a new container with `$ make run` it will select fastrtps


## ros demo

to run the ros talker - listener demo use two terminal windows

in terminal a (`a$`):

* `a$ make build` to build the current source packages inside the container
* `a$ make talker-demo` to run the talker demo in a new container

in terminal b (`b$`):

* `b$ make listener-demo` to run the listener demo in a new container

you should see output in terminal a similar to:

```
[INFO] [1722471491.552201505] [talker]: Publishing: 'Hello World: 1'
[INFO] [1722471492.552257864] [talker]: Publishing: 'Hello World: 2'
[INFO] [1722471493.552201928] [talker]: Publishing: 'Hello World: 3'
[INFO] [1722471493.552201928] [talker]: Publishing: 'Hello World: 4'
```

and output in terminal b similar to:

```
[INFO] [1722471494.552915449] [listener]: I heard: [Hello World: 3]
[INFO] [1722471495.552809036] [listener]: I heard: [Hello World: 4]
```
