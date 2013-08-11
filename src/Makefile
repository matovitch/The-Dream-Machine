ifeq ($(MODE), release)
   OBJS = bvh_opt.o linalg.o mem.o intr_box_opt.o intr_tri_opt.o intr_bvh.o intr_frustum_opt.o view.o shader_opt.o sse.o 
   CFLAGS = -march=native -O3 -funroll-loops -Wall -Werror -pedantic -std=c99
else
   OBJS = bvh.o linalg.o mem.o intr_box.o intr_tri.o intr_bvh.o intr_frustum.o view.o shader.o sse.o
   CFLAGS = -march=native -O1 -g -pg -Wall -Werror -pedantic -std=c99
endif
LDFLAGS =
CC = gcc
RM = rm
.PHONY: all

all: libdream.a

libdream.a: $(OBJS)
	ar rvs $@ $^

%.o: %.c
	$(CC) -c $(CFLAGS) $^ -o $@

clean:
	$(RM) *.o libdream.a *~
