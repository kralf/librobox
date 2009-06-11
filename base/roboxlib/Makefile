include $(CARMEN_HOME)/src/Makefile.conf

ifndef CYGWIN
CFLAGS += -fPIC
endif
IFLAGS += -I../../../posix/thread
LFLAGS += -lglobal -lipc -lparam_interface

MODULE_NAME = "ROBOX"
MODULE_COMMENT = "RoboX Base Module"

SOURCES = device.c power.c security.c bumper.c robox_lib.c
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = librobox.a

TARGETS = librobox.a

librobox.a: device.o power.o security.o bumper.o robox_lib.o

include $(CARMEN_HOME)/src/Makefile.rules
