TARGET  = powerpc

ROBOXLIB_SRC = $(wildcard src/roboxlib/*.c)

XCROOT  = ../../linux/.xcomp.root/$(TARGET)
CC      = $(XCROOT)/bin/$(TARGET)-linux-gcc
CFLAGS  += -Isrc
LDFLAGS +=

OBJS    = lib/libroboxlib.so bin/roboxtest bin/roboxtest2

all: $(OBJS) 

lib/libroboxlib.so: $(ROBOXLIB_SRC)
	$(CC) $< $(CFLAGS) -shared -fPIC -Wall  -soname,$@ -o $@
  
bin/roboxtest: src/sandbox/roboxtest.c lib/libroboxlib.so
	$(CC) $< $(CFLAGS) -Wall -Wl,-rpath,../lib:lib -Llib -lroboxlib -l pthread -o $@

bin/roboxtest2: src/sandbox/roboxtest2.c lib/libroboxlib.so
	$(CC) $< $(CFLAGS) -Wall -Wl,-rpath,../lib:lib -Llib -lroboxlib -l pthread -o $@

install: all
	install lib/* /usr/lib
	install src/roboxlib/*.h /usr/include/roboxlib

clean:
	rm -f *.o $(OBJS)
