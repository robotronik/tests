
PROJECT_DIR = test

.PHONY: $(PROJECT_DIR)

$(PROJECT_DIR): libopencm3-lib
	@echo "Building $@"
	make -C $@/

libopencm3/:
	@echo "getting libopencm3"
	$(shell git submodule init && git submodule update )

libopencm3-lib: libopencm3/
	@echo "Building $@"
	@make -C libopencm3/

clean:
	@echo "Cleaning $(PROJECT_DIR)"
	@make -C $(PROJECT_DIR) clean

clean-all: clean
	@echo "Cleaning libopencm3"
	@make -C libopencm3 clean
