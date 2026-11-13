CFLAGS = -O2 -Wall -fPIC

libik.so: ik.c
	$(CC) $(CFLAGS) -shared -o libik.so ik.c -lm

clean:
	rm -f libik.so
