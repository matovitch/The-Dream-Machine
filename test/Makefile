CFLAGS = -O3 -march=native -fopenmp -Wall -pedantic -I../src -std=c99
LDFLAGS = -L../src -ldream -lm -fopenmp
CC = gcc
RM = rm

.PHONY: all
all: viewer bvh_test scale_obj intr_box_test intr_box_bench intr_tri_test intr_frustum_test intr_tri_bench intr_bvh_test view_test

intr_bvh_test: intr_bvh_test.o
	$(CC) $^ $(LDFLAGS) -o $@

intr_tri_bench: intr_tri_bench.o
	$(CC) $^ $(LDFLAGS) -o $@

intr_frustum_test: intr_frustum_test.o
	$(CC) $^ $(LDFLAGS) -o $@

intr_tri_test: intr_tri_test.o
	$(CC) $^ $(LDFLAGS) -o $@

intr_box_test: intr_box_test.o
	$(CC) $^ $(LDFLAGS) -o $@

intr_box_bench: intr_box_bench.o
	$(CC) $^ $(LDFLAGS) -o $@

view_test: view_test.o
	$(CC) $^ $(LDFLAGS) -o $@

bvh_test: bvh_test.o
	$(CC) $^ $(LDFLAGS) -o $@

scale_obj: scale_obj.o
	$(CC) $^ $(LDFLAGS) -o $@

viewer: viewer.o obj_loader.o tga_loader.o
	$(CC) $^ $(LDFLAGS) -lSDLmain -lSDL -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@

clean:
	$(RM) *.o *~ bvh_test viewer scale_obj intr_box_test intr_box_bench intr_tri_bench intr_tri_test intr_frustum_test intr_bvh_test view_test
