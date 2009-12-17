include $(CARMEN_HOME)/src/Makefile.conf

SUBDIRS =

CFLAGS +=
IFLAGS += -I../robot -I../laser
LFLAGS += -L../../../../posix/config -lconfig -L../../../../posix/timer \
	-ltimer -L../../../../posix/thread -lthread -lparam_interface -lglobal \
	-lcarmenserial -lgeometry -lipc -lrobox_base

MODULE_NAME = BASE
MODULE_COMMENT = Generic interface to all robot bases

SOURCES =
PUBLIC_INCLUDES =
PUBLIC_BINARIES = robox

TARGETS = robox

robox: robox.o

include $(CARMEN_HOME)/src/Makefile.rules
