PROJECT=TestsLibopencm3
default: all
# Default Options
export ARCH   = libopencm3
export ROBOT ?= gros
export SDL   ?= no
export DEBUG ?= 0

PARENT_DIR = ../../

# Constantes de compilation
EXEC    = tests

include $(PARENT_DIR)/hardware/common.mk
################################################################################
# Fichiers du projet

FICHIERS_C = \
	test.c

CFLAGS +=	-Ihardware/$(ARCH)/ \
			-Ihardware/ 		\
			-IInc

FICHIERS_O  += $(addprefix $(BUILD_DIR)/, $(FICHIERS_C:.c=.o) )

################################################################################
.PHONY: all view

all: $(BUILD_DIR)/$(EXEC)

$(BUILD_DIR)/$(EXEC): $(FICHIERS_O) libHardware
	@echo "	CC	$(PROJECT)|$(notdir $@)"
	@$(CC) -o $@ $(FICHIERS_O) $(CFLAGS) -lHardware $(LDFLAGS)


$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

