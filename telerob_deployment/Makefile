SHELL := /bin/bash
build_dir := build
archive_dir := $(build_dir)/flash_archive
ansible_dir := $(archive_dir)/ansible
deployment_role_build_dir := $(ansible_dir)/roles/deployment/files/build
system_setup_role_build_dir := $(ansible_dir)/roles/system_setup/files/build
production_compose := -f deployment/compose/docker-compose.yaml
development_compose := -f deployment/compose/docker-compose.yaml -f deployment/compose/overlays/development.yaml

.PHONY: build archive clean checks check-env check-deployment-hash check-docker-ids-prod check-docker-ids-dev pull pull-development download-packages check-package-deps

checks: check-env check-deployment-hash check-docker-ids-prod

checks-dev: check-env check-deployment-hash check-docker-ids-dev

check-env:
	@cmp --silent $(deployment_role_build_dir)/env_stamp <(echo "SLAM_BRINGUP_VERSION=$${SLAM_BRINGUP_VERSION:-nightly}"; echo "INTERFACE_VERSION=$${INTERFACE_VERSION:-nightly}") || \
	rm -f $(deployment_role_build_dir)/env_stamp 2>/dev/null || true

$(deployment_role_build_dir)/env_stamp:
	@echo "SLAM_BRINGUP_VERSION=$${SLAM_BRINGUP_VERSION:-nightly}" > $@
	@echo "INTERFACE_VERSION=$${INTERFACE_VERSION:-nightly}" >> $@

check-deployment-hash:
	@cmp --silent $(deployment_role_build_dir)/deployment_files.md5 <(find deployment -type f -exec md5sum {} \; | sort) || \
	rm -f $(deployment_role_build_dir)/deployment_files.md5 || true

$(deployment_role_build_dir)/deployment_files.md5:
	@find deployment -type f -exec md5sum {} \; | sort > $@

$(archive_dir)/created:
	mkdir -p $(archive_dir)
	@touch $@

$(ansible_dir)/copied: $(archive_dir)/created $(shell find ansible -type f)
	mkdir -p $(ansible_dir)
	find $(ansible_dir) -type f ! -wholename "$(deployment_role_build_dir)/*" -exec bash -c 'rm -f "$0"' {} \;
	cp -rf ansible/* $(ansible_dir)
	@touch $@

$(deployment_role_build_dir)/deployment_files.tar: $(deployment_role_build_dir)/deployment_files.md5
	tar -cf $@ -C deployment .

check-docker-ids-prod:
	@cmp --silent $(build_dir)/docker.ids.prod <(docker image inspect --platform=linux/arm64 $$(docker compose $(production_compose) config --images | sort) | jq -r '.[].Id') || \
	rm -f $(build_dir)/docker.ids.prod || true

check-docker-ids-dev:
	@cmp --silent $(build_dir)/docker.ids.dev <(docker image inspect --platform=linux/arm64 $$(docker compose $(development_compose) config --images | sort) | jq -r '.[].Id') || \
	rm -f $(build_dir)/docker.ids.dev || true

check-package-deps:
	@cmp --silent $(build_dir)/package-deps.md5 <(md5sum ../docker/package-downloader.Dockerfile) || \
	rm -f $(build_dir)/package-deps.md5 $(build_dir)/package-downloader.built || true

$(build_dir)/package-deps.md5: ../docker/package-downloader.Dockerfile
	@mkdir -p $(build_dir)
	@md5sum ../docker/package-downloader.Dockerfile > $@

$(build_dir)/package-downloader.built: $(build_dir)/package-deps.md5
	@echo "Building package downloader image for arm64..."
	@docker buildx build \
		--platform linux/arm64 \
		--file ../docker/package-downloader.Dockerfile \
		--tag telerob-package-downloader:latest \
		--load \
		..
	@touch $@

$(system_setup_role_build_dir)/built: $(build_dir)/package-downloader.built
	@echo "Extracting packages from image..."
	docker run --rm --platform linux/arm64 telerob-package-downloader:latest > $(system_setup_role_build_dir)/packages.tar.gz
	@echo "Packages extracted to: $(system_setup_role_build_dir)"
	@touch $@

download-packages: $(system_setup_role_build_dir)/built
	@echo "Package cache ready at: $(system_setup_role_build_dir)/packages.tar.gz"

$(build_dir)/docker.ids.prod: deployment/compose/docker-compose.yaml
	@docker image inspect --platform=linux/arm64 $$(docker compose $(production_compose) config --images | sort) | jq -r '.[].Id' > $@

$(build_dir)/docker.ids.dev: deployment/compose/docker-compose.yaml deployment/compose/overlays/development.yaml
	@docker image inspect --platform=linux/arm64 $$(docker compose $(development_compose) config --images | sort) | jq -r '.[].Id' > $@

$(build_dir)/docker.imgs.prod: $(build_dir)/docker.ids.prod
	docker save --platform arm64 $$(docker compose $(production_compose) config --images) | gzip > $@

$(build_dir)/docker.imgs.dev: $(build_dir)/docker.ids.dev
	docker save --platform arm64 $$(docker compose $(development_compose) config --images) | gzip > $@

$(build_dir)/versions.txt: $(deployment_role_build_dir)/env_stamp
	@commit_hash=$$(git rev-parse --short HEAD 2>/dev/null || echo ""); \
	add_hash() { \
		local version="$$1"; \
		if [[ "$$version" == "nightly" || "$$version" == "dev" ]] && [ -n "$$commit_hash" ]; then \
			echo "$${version}+$${commit_hash}"; \
		else \
			echo "$$version"; \
		fi; \
	}; \
	echo "SLAM_BRINGUP_VERSION:$$(add_hash "$${SLAM_BRINGUP_VERSION:-nightly}")" > $@; \
	echo "INTERFACE_VERSION:$$(add_hash "$${INTERFACE_VERSION:-nightly}")" >> $@


$(deployment_role_build_dir)/.env: $(deployment_role_build_dir)/env_stamp
	@echo "SLAM_BRINGUP_VERSION=$${SLAM_BRINGUP_VERSION:-nightly}" > $@
	@echo "INTERFACE_VERSION=$${INTERFACE_VERSION:-nightly}" >> $@

$(ansible_dir)/built: \
		$(ansible_dir)/copied \
		$(deployment_role_build_dir)/.env \
		$(build_dir)/versions.txt \
		$(deployment_role_build_dir)/deployment_files.tar \
		$(system_setup_role_build_dir)/built
	@touch $@

$(ansible_dir)/built-prod: \
		$(ansible_dir)/built \
		$(build_dir)/docker.imgs.prod
	cp -f $(build_dir)/docker.imgs.prod $(deployment_role_build_dir)/docker.imgs
	cp -f $(build_dir)/versions.txt $(deployment_role_build_dir)/versions.txt
	echo "compose/docker-compose.yaml" > $(deployment_role_build_dir)/compose_files
	rm -f $(ansible_dir)/built-dev
	@touch $@

$(ansible_dir)/built-dev: \
		$(ansible_dir)/built \
		$(build_dir)/docker.imgs.dev
	cp -f $(build_dir)/docker.imgs.dev $(deployment_role_build_dir)/docker.imgs
	sed -E '/\+/{s/\+/-dev+/;b};s/$$/-dev/' $(build_dir)/versions.txt > $(deployment_role_build_dir)/versions.txt
	echo "compose/docker-compose.yaml compose/overlays/development.yaml" > $(deployment_role_build_dir)/compose_files
	rm -f $(ansible_dir)/built-prod
	@touch $@

$(archive_dir)/deploy.sh: $(archive_dir)/created deploy.sh
	cp -f deploy.sh $@
	chmod +x $@

$(archive_dir)/built-prod: checks $(ansible_dir)/built-prod $(archive_dir)/deploy.sh
	@touch $@
	rm -f $(archive_dir)/built-dev

$(archive_dir)/built-dev: checks-dev $(ansible_dir)/built-dev $(archive_dir)/deploy.sh
	@touch $@
	rm -f $(archive_dir)/built-prod

build/telerob_flash_archive.prod.tar.gz: $(archive_dir)/built-prod
	tar -czf $@ -C $(archive_dir) .

build/telerob_flash_archive.dev.tar.gz: $(archive_dir)/built-dev
	tar -czf $@ -C $(archive_dir) .

build: $(archive_dir)/built-prod

build-development: $(archive_dir)/built-dev

archive: build/telerob_flash_archive.prod.tar.gz

archive-development: build/telerob_flash_archive.dev.tar.gz

clean:
	rm -rf build

pull:
	docker compose $(production_compose) pull

pull-development:
	docker compose $(development_compose) pull
