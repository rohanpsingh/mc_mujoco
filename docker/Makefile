export DOCKER_BUILDKIT := 1

.PHONY: run ci-test

run:
	@echo "Granting X11 access to docker..."
	@xhost +local:docker >/dev/null
	@echo "Building container..."
	@docker compose -f docker-compose.yml build
	@echo "Building mc_mujoco and starting shell..."
	@docker compose -f docker-compose.yml run --rm mc-rtc-mujoco bash -c "\
		cmake -S /workspace/mc_mujoco -B /workspace/mc_mujoco/build -G Ninja \
		    -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=/opt/mujoco \
		    -DLIBTORCH_ROOT=/opt/libtorch && \
		cmake --build /workspace/mc_mujoco/build && \
		cmake --install /workspace/mc_mujoco/build ; \
		exec bash"
	@echo "Revoking X11 access..."
	@xhost -local:docker >/dev/null

ci-test:
	@echo "Building environment image..."
	@docker compose -f docker-compose.yml build
	@echo "Running CI build..."
	@docker compose -f docker-compose.ci.yml run --rm mc-mujoco-ci \
		bash -c "cmake -S /workspace -B /ci-build -G Ninja \
		    -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=/opt/mujoco \
		    -DLIBTORCH_ROOT=/opt/libtorch && \
		cmake --build /ci-build && \
		cmake --install /ci-build && \
		echo 'Build passed! Running standalone test...' && \
		cmake -S /workspace/.github/workflows/test-standalone \
		    -B /ci-build-standalone -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
		cmake --build /ci-build-standalone && \
		ctest -V --test-dir /ci-build-standalone && \
		echo 'CI test passed!'"
