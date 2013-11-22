COMPONENT=CTPEventBasedSensorAppC
PFLAGS += -I$(TOSROOT)/tos/lib/printf
PFLAGS += -DNEW_PRINTF_SEMANTICS
PFLAGS += -DCC2420_DEF_CHANNEL=15
PFLAGS += -DCC2420_DEF_RFPOWER=20
CFLAGS += -I$(TOSDIR)/lib/net \
          -I$(TOSDIR)/lib/net/le \
          -I$(TOSDIR)/lib/net/ctp
CFLAGS += -I$(shell pwd)/..
include $(MAKERULES)
