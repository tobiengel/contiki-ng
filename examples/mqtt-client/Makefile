CONTIKI_PROJECT = mqtt-client
all: $(CONTIKI_PROJECT)

TARGET = simplelink
BOARD = sensortag/cc1352r
CFLAGS += -g -Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-function

CONTIKI = ../..

include $(CONTIKI)/Makefile.dir-variables
MODULES += $(CONTIKI_NG_APP_LAYER_DIR)/mqtt

-include $(CONTIKI)/Makefile.identify-target

MODULES_REL += arch/platform/$(TARGET)

PLATFORMS_ONLY =  simplelink 

include $(CONTIKI)/Makefile.include
