SHELL := /bin/bash

PACKAGE=seymour-dev
VERSION:=0.1.0
PLATFORM=linux/amd64
#PLATFORM=linux/arm64
CONTAINER:=ghcr.io/freshrobotics/$(PACKAGE)-$(PLATFORM):$(VERSION)
USERNAME=seymour
WORKSPACE=/home/$(USERNAME)/workspace
RUN_AS_UID=$(shell id -u)
RUN_AS_GID=$(shell id -g)
RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
# RMW_IMPLEMENTATION="rmw_fastrtps_cpp"

DOCKER_RUN_ARGS=--rm -it \
		--platform $(PLATFORM) \
		--network host \
		--privileged \
		--env DISPLAY \
		--env RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} \
		--volume $(PWD):$(WORKSPACE)

PHONY: help
help: ## show help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.PHONY: version
version: ## print the package version
	@echo $(VERSION)

.PHONY: run
run: ## start container with shell
	docker run $(DOCKER_RUN_ARGS) \
		--name $(PACKAGE) \
		$(CONTAINER) \
		/bin/bash -i

.PHONY: stop
stop: ## stops running container
	docker stop $(PACKAGE)

.PHONY: shell
shell: ## get (another) shell to running container
	docker exec -it $(PACKAGE) /bin/bash

.PHONY: image
image: ## builds the docker image
	docker build \
		--platform $(PLATFORM) \
		--build-arg USERNAME=$(USERNAME) \
		--build-arg RUN_AS_UID=$(RUN_AS_UID) \
		--build-arg RUN_AS_UID=$(RUN_AS_UID) \
		--tag $(CONTAINER) \
		.

.PHONY: build
build: image ## build current source in container
	docker run --rm -t \
		--platform $(PLATFORM) \
		--volume $(PWD):$(WORKSPACE) \
		--name $(PACKAGE) \
		$(CONTAINER) \
		/bin/bash -ic "colcon build"

.PHONY: clean
clean: ## remove colcon build artifacts
	rm -rf ./build ./install ./log

.PHONY: talker-demo
talker-demo: ## run demo talker node
	docker run $(DOCKER_RUN_ARGS) \
		--name $(PACKAGE)-talker \
		$(CONTAINER) \
		/bin/bash -ic "ros2 run demo_nodes_cpp talker"

.PHONY: listener-demo
listener-demo: ## run demo talker node
	docker run $(DOCKER_RUN_ARGS) \
		--name $(PACKAGE)-listener \
		$(CONTAINER) \
		/bin/bash -ic "ros2 run demo_nodes_cpp listener"

.PHONY: install-multiarch
install-multiarch: ## setup multiarch support on ubuntu
	sudo apt-get install -y qemu-user-static
	docker run --privileged --rm tonistiigi/binfmt --install all
