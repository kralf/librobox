TARGET  = powerpc

ROBOXLIB_SRC = $(wildcard src/roboxlib/*.c)

XCROOT  = ../../linux/.xcomp.root/$(TARGET)
CC      = $(XCROOT)/bin/$(TARGET)-linux-gcc
CFLAGS  += -Isrc
LDFLAGS += 

OBJS    = lib/libroboxlib.so bin/roboxtestInit bin/roboxtestOdometry bin/roboxtestspeed \
				  bin/serialtest bin/lasertest

all: $(OBJS) 

lib/libroboxlib.so: $(ROBOXLIB_SRC)
	mkdir -p lib
	$(CC) $< $(CFLAGS) -shared -fPIC -Wall -lm -soname,$@ -o $@
  
bin/serialtest: src/sandbox/serialtest.c
	mkdir -p bin
	$(CC) $< $(CFLAGS) -Wall -Wl,-rpath,../lib:lib -Llib -lpthread -o $@

bin/lasertest: src/sandbox/lasertest.c
	mkdir -p bin
	$(CC) $< $(CFLAGS) -Wall -Wl,-rpath,../lib:lib -Llib -lpthread -o $@

bin/roboxtestInit: src/sandbox/roboxtestInit.c lib/libroboxlib.so
	mkdir -p bin
	$(CC) $< $(CFLAGS) -Wall -Wl,-rpath,../lib:lib -Llib -lroboxlib -l pthread -o $@

bin/roboxtestOdometry: src/sandbox/roboxtestOdometry.c lib/libroboxlib.so
	mkdir -p bin
	$(CC) $< $(CFLAGS) -Wall -Wl,-rpath,../lib:lib -Llib -lroboxlib -l pthread -o $@

bin/roboxtestspeed: src/sandbox/roboxtestspeed.c lib/libroboxlib.so
	mkdir -p bin
	$(CC) $< $(CFLAGS) -Wall -Wl,-rpath,../lib:lib -Llib -lroboxlib -l pthread -o $@

install: all
	scp -r lib bin root@shantippc0:/usr

clean:
	rm -rf lib
	rm -rf bin
