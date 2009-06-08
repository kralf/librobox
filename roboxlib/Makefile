TARGET  = powerpc

ROBOXLIB_SRC = $(wildcard src/roboxlib/*.c)

XCROOT  = ../../linux/.xcomp.root/$(TARGET)
CC      = $(XCROOT)/bin/$(TARGET)-linux-gcc
CFLAGS  += -Isrc
LDFLAGS +=

OBJS    = lib/libroboxlib.so bin/roboxtest bin/roboxtest2

all: $(OBJS) 

lib/libroboxlib.so: $(ROBOXLIB_SRC)
	mkdir -p lib
	$(CC) $< $(CFLAGS) -shared -fPIC -Wall -soname,$@ -o $@
  
bin/roboxtest: src/sandbox/roboxtest.c lib/libroboxlib.so
	mkdir -p bin
	$(CC) $< $(CFLAGS) -Wall -Wl,-rpath,../lib:lib -Llib -lroboxlib -l pthread -o $@

bin/roboxtest2: src/sandbox/roboxtest2.c lib/libroboxlib.so
	mkdir -p bin
	$(CC) $< $(CFLAGS) -Wall -Wl,-rpath,../lib:lib -Llib -lroboxlib -l pthread -o $@

install: all
	scp -r lib bin root@shantippc0:/usr

clean:
	rm -rf lib
	rm -rf bin
