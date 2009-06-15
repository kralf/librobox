include $(CARMEN_HOME)/src/Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lrobot_interface -lipc -lglobal -lparam_interface

MODULE_NAME = JOYROBOX
MODULE_COMMENT = RoboX joystick control module

SOURCES = joyctrl.c joyrobox.c
PUBLIC_INCLUDES =
PUBLIC_LIBRARIES =
PUBLIC_BINARIES = joyrobox

TARGETS = joyrobox

joyrobox: joyrobox.o joyctrl.o ../era/libera_interface.a

include $(CARMEN_HOME)/src/Makefile.rules
