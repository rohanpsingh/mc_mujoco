.PHONY: run ci-test

run:
	@echo "Granting X11 access to docker..."
	@xhost +local:docker >/dev/null
	@echo "Building container..."
	@docker compose -f docker-compose.yml build
	@echo "Running container..."
	@docker compose -f docker-compose.yml run --rm mc-rtc-mujoco
	@echo "Revoking X11 access..."
	@xhost -local:docker >/dev/null

ci-test:
	@echo "Building CI container..."
	@docker compose -f docker-compose.ci.yml build
	@echo "Running CI build..."
	@docker compose -f docker-compose.ci.yml run --rm mc-mujoco-ci \
		bash -c "rm -rf build && mkdir -p build && cd build && \
			cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=/opt/mujoco && \
			cmake --build . --config RelWithDebInfo && \
			cmake --install . && \
			echo 'Build passed! Running standalone test...' && \
			cd /workspace/.github/workflows/test-standalone && \
			rm -rf build && mkdir build && cd build && \
			cmake ../ -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
			cmake --build . --config RelWithDebInfo && \
			ctest -V -C RelWithDebInfo && \
			echo 'CI test passed!'"
