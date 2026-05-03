export DOCKER_BUILDKIT := 1

MUJOCO_VERSION := $(shell tr -d '[:space:]' < ../MUJOCO_VERSION)
export MUJOCO_VERSION

BUILD_VOLUME := mc_mujoco_build_$(MUJOCO_VERSION)

.PHONY: run ci-test

run:
	@# Ensure the build cache for the current MUJOCO_VERSION exists, and remove
	@# any build cache from previous versions (incl. the legacy unversioned name).
	@# This is what prevents stale .o files compiled against an older MuJoCo
	@# from silently surviving a version bump.
	@docker volume create $(BUILD_VOLUME) >/dev/null
	@for v in $$(docker volume ls -q --filter name=mc_mujoco_build); do \
	  if [ "$$v" != "$(BUILD_VOLUME)" ]; then \
	    if docker volume rm "$$v" >/dev/null 2>&1; then \
	      echo "Removed stale build volume: $$v"; \
	    else \
	      echo "WARNING: could not remove stale build volume $$v (likely held by a running container — stop it and re-run 'make' to reclaim)"; \
	    fi; \
	  fi; \
	done
	@echo "Granting X11 access to docker..."
	@xhost +local:docker >/dev/null
	@echo "Building container..."
	@docker compose -f docker-compose.yml build
	@echo "Building mc_mujoco and starting shell..."
	@docker compose -f docker-compose.yml run --rm mc-rtc-mujoco bash -c "\
		cmake -S /workspace/mc_mujoco -B /build -G Ninja \
		    -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=/opt/mujoco && \
		cmake --build /build && \
		cmake --install /build ; \
		exec bash"
	@echo "Revoking X11 access..."
	@xhost -local:docker >/dev/null

ci-test:
	@echo "Building environment image..."
	@docker compose -f docker-compose.yml build
	@echo "Running CI build..."
	@docker compose -f docker-compose.ci.yml run --rm mc-mujoco-ci \
		bash -c "cmake -S /workspace -B /ci-build -G Ninja \
		    -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=/opt/mujoco && \
		cmake --build /ci-build && \
		cmake --install /ci-build && \
		echo 'Build passed! Running standalone test...' && \
		cmake -S /workspace/.github/workflows/test-standalone \
		    -B /ci-build-standalone -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
		cmake --build /ci-build-standalone && \
		ctest -V --test-dir /ci-build-standalone && \
		echo 'CI test passed!'"
