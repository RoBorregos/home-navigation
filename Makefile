nav.build:
	@docker build -t nav -f ./docker/Dockerfile.nav .

nav.create:
	@./docker/scripts/run.bash --volumes=./ws --name='nav'

nav.up:
	@xhost +
	@docker start nav

make nav.up.jetson:
	@docker start nav

nav.down:
	@docker stop nav

nav.shell:
	@docker exec -it --user $(shell id -u):$(shell id -g) nav bash

nav.remove:
	@docker container rm nav

