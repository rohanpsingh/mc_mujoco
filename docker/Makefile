.PHONY: run-docker-container
run:
	@echo "Granting X11 access to docker..."
	@xhost +local:docker >/dev/null
	@echo "Building container..."
	@docker compose -f docker-compose.yml build
	@echo "Running container..."
	@docker compose -f docker-compose.yml run --rm mc-rtc-mujoco
	@echo "Revoking X11 access..."
	@xhost -local:docker >/dev/null
