LDFLAGS ?= -lc
CFLAGS ?= -O3 -Wall

serialtest: serialtest.c
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

clean:
	rm -f serialtest

.PHONY: clean
