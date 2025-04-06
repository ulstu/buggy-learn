-MAKEFLAGS+=--silent
UID:=$(shell id -u)
PROJECT_NAME ?= Thymio2
DOCKER_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
WEBOTS_ORIGINAL_PROJECT:=$(shell dirname ${DOCKER_DIR})
DIR_FOR_COPY_PROJECT := $(shell dirname $(WEBOTS_ORIGINAL_PROJECT))
PROJECT_DIR:=${DOCKER_DIR}
ROBOTBENCHMARK_ROOT_DIR := $(realpath $(PROJECT_DIR)/../../..)
APP_MEDIA_DIR := ${ROBOTBENCHMARK_ROOT_DIR}/application/media
NVIDIA_GPU:=$(shell (docker info | grep Runtimes | grep nvidia 1> /dev/null && command -v nvidia-smi 1>/dev/null 2>/dev/null && nvidia-smi | grep Processes 1>/dev/null 2>/dev/null) && echo '--runtime nvidia --gpus all' || echo '')
IMAGE=ulstub
FLAVOR ?= devel
ROBOT_PANEL_PORT ?= 8009
VS_PORT ?= 31416
WEBOTS_STREAM_PORT ?= 1235
DISPLAY=:1
CONTAINER_NAME:=${IMAGE}-${FLAVOR}

.PHONY: all

all: copy-working-folders run copy-working-files start-dev-server start-webots
# make all FLAVOR='hibers-3' ROBOT_PANEL_PORT=1564 VS_PORT=1573 WEBOTS_STREAM_PORT=1583
colors:
	$(eval NC=\033[1;0m)
	$(eval RED=\033[1;31m)
	$(eval GREEN=\033[1;32m)
	$(eval BOLD=\033[1;37m)

build:
	echo ${NO_CACHE_ARG}
	DOCKER_BUILDKIT=1 docker build ${DOCKER_DIR} -f ${DOCKER_DIR}/Dockerfile.base -t ${IMAGE} ${DOCKER_ARGS} --build-arg UID=${UID}

run: 
	@if docker ps -q -f name=${CONTAINER_NAME} | grep -q .; then \
	    echo "Контейнер ${CONTAINER_NAME} уже запущен"; \
	else \
	    echo "Контейнер ${CONTAINER_NAME} запускается..."; \
		docker run --ipc=host \
			--cap-add SYS_ADMIN \
			--name ${IMAGE}-${FLAVOR} \
			--add-host=host.docker.internal:host-gateway \
			--privileged \
			--restart unless-stopped \
			-p ${ROBOT_PANEL_PORT}:8008 -p ${VS_PORT}:31415 -p ${WEBOTS_STREAM_PORT}:1234 \
			-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
			-e DISPLAY=${DISPLAY} \
			-v ~/.Xauthority:/ulstu/.host/.Xauthority:ro \
			-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
			-e DISPLAY=:1 \
			--device=/dev/dxg \
			-i --gpus all --device /dev/dri:/dev/dri \
			-e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
			-v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
			-v /dev:/dev:rw \
			-v ${DIR_FOR_COPY_PROJECT}/projects/${FLAVOR}:/ulstu/repositories:rw \
			-d -i ${IMAGE} 1>/dev/null ; \
	fi

copy-working-files:
	docker exec -i ${IMAGE}-${FLAVOR} sh -c "chown -R ulstu /ulstu/repositories" && \
	docker exec -i ${IMAGE}-${FLAVOR} sh -c "chgrp -R ulstu /ulstu/repositories" ; \

copy-working-folders:
	@if [ -d ${DIR_FOR_COPY_PROJECT}/projects/${FLAVOR} ]; then \
	    echo "Blockly dir exists"; \
	else \
	    mkdir -p ${DIR_FOR_COPY_PROJECT}/projects/${FLAVOR}; \
	fi

	@if [ -d ${DIR_FOR_COPY_PROJECT}/projects/${FLAVOR}/${PROJECT_NAME} ]; then \
	    echo "Project dir ${DIR_FOR_COPY_PROJECT}/projects/${FLAVOR}/${PROJECT_NAME} exists"; \
	else \
	    echo "Copying project directory..."; \
	    cp -r ${PROJECT_DIR}/projects/${PROJECT_NAME} ${DIR_FOR_COPY_PROJECT}/projects/${FLAVOR}/; \
	fi

td:
	echo ${PROJECT_DIR}

start-webots:
	echo "Project ${PROJECT_NAME} — перезапускаем Webots..."; \
	docker exec -d -i ${IMAGE}-${FLAVOR} bash -c '\
		killall webots || true && \
		xvfb-run webots --stdout --stderr --no-rendering --minimize --batch \
		--port=1234 --mode=pause --heartbeat=5000 --stream=x3d \
		/ulstu/repositories/${PROJECT_NAME}/worlds/scene.wbt'

start-dev-server:
	docker exec -d -i ${IMAGE}-${FLAVOR} bash -c 'python3 /ulstu/server/send_cmd_server.py'

stop-webots:
	docker exec -i ${IMAGE}-${FLAVOR} pkill -f webots

exec:
	docker exec -i ${IMAGE}-${FLAVOR} bash

destroy:
	docker container kill ${IMAGE}-${FLAVOR} 1>/dev/null || true
	docker container rm -f ${IMAGE}-${FLAVOR} 1>/dev/null || true

