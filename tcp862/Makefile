include $(CARMEN_HOME)/src/Makefile.conf

CFLAGS += -I../../drivers/vanilla/tcp862-non-free
IFLAGS +=
LFLAGS +=

MODULE_NAME = "TCP862"
MODULE_COMMENT = "Library for the TCP862 serial interface card"

PUBLIC_INCLUDES = tcp862_serial.h
PUBLIC_LIBRARIES = libtcp862.a
TARGETS = libtcp862.a

libtcp862.a: tcp862_serial.o

include $(CARMEN_HOME)/src/Makefile.rules
