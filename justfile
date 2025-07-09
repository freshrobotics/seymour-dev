image-name := "seymour-develop-base"
version := "0.1.0"
run-platform := "linux/amd64"
# run-platform := "linux/arm64"
build-platforms := "linux/amd64,linux/arm64"
tag := "ghcr.io" / "freshrobotics" / image-name + ":" + version

# by default list available recipes
default:
  @just --list

# create container manifest for image tag
create-manifest:
  @podman manifest create {{tag}}

# build container image
build-image: create-manifest
  @podman build \
    --platform {{build-platforms}} \
    --manifest {{tag}} \
    .

# push manifest to registry
push-image:
  @podman manifest push {{tag}}

# run container with set shell
run:
  @podman run \
    --platform {{run-platform}} \
		--privileged \
		--network host \
		--ipc host \
		--env DISPLAY=${DISPLAY} \
		--name {{image-name}} \
    /bin/bash
