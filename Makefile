nav.build:
	@docker build -t nav -f ./docker/Dockerfile.nav .

nav.build.xavier:
	@docker build -t nav-xavier -f ./docker/Dockerfile.nav.xavier .

nav.build.mediapipe:
	@docker build -t nav-mediapipe -f ./docker/Dockerfile.nav.mediapipe .

nav.create:
	@(if [ ! -z ${DISPLAY} ]; then xhost +; fi)
	@./docker/scripts/run.bash --volumes=./ws --name='nav'

nav.create.xavier:
	@./docker/scripts/run.bash --volumes=./ws --name='nav-xavier' --xavier

nav.create.mediapipe:
	@./docker/scripts/run.bash --volumes=./ws --name='nav-mediapipe' --mediapipe

nav.up:
	@(if [ ! -z ${DISPLAY} ]; then xhost +; fi)
	@docker start nav

make nav.up.jetson:
	@docker start nav

nav.up.xavier:
	@docker start nav-xavier

nav.up.mediapipe:
	@docker start nav-mediapipe

nav.down:
	@docker stop nav

nav.down.xavier:
	@docker stop nav-xavier

nav.down.mediapipe:
	@docker stop nav-mediapipe

nav.shell:
	@docker exec -it --user $(shell id -u):$(shell id -g) nav bash

nav.shell.xavier:
	@docker exec -it --user $(shell id -u):$(shell id -g) nav-xavier bash

nav.shell.mediapipe:
	@docker exec -it --user $(shell id -u):$(shell id -g) nav-mediapipe bash

nav.remove:
	@docker container rm nav

nav.remove.xavier:
	@docker container rm nav-xavier

nav.remove.mediapipe:
	@docker container rm nav-mediapipe