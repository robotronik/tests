

SRC = test_fork.c

OBJ = $(SRC:.c=.o)

%.o: %.c
	@echo "	CC	$(notdir $@)"
	@$(CC) $(CFLAGS) $< -o $@ -c

test-fork: $(OBJ)
	@$(CC) $(CFLAGS) $^ -o $@


run: test-fork
	./$<
