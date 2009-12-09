include $(CARMEN_HOME)/src/Makefile.conf

CFLAGS += -DUSE_TCP862 -DLASER_USE_PTHREAD -std=gnu99
LFLAGS += -ltcp862 -lparam_interface -llaser_interface -lcarmenserial -lglobal -lipc -lpthread

MODULE_NAME = "LASER"
MODULE_COMMENT = "The laser module for the RoboX SICK lasers"

SOURCES = carmen_laser_device.c carmen_laser_message_queue.c laser.c \
          carmen_laser_device_init.c carmen_sick.c sick_laser.c

PUBLIC_LIBRARIES = liblaser.a
PUBLIC_BINARIES = laser

TARGETS = laser

liblaser.a: carmen_laser_device.o carmen_laser_message_queue.o \
			carmen_laser_device_init.o sick_laser.o carmen_sick.o

laser: laser.o liblaser.a

include $(CARMEN_HOME)/src/Makefile.rules
