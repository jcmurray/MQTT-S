PROGNAME := MQTTSClientApp
SUBDIR := mqttslib
SRCS := MqttsClientApp.cpp \
$(SUBDIR)/MQTTS.cpp \
$(SUBDIR)/MqttsClient.cpp \
$(SUBDIR)/MqttsClientAppFw4Arduino.cpp \
$(SUBDIR)/ZBeeStack.cpp 

CXX := g++
CPPFLAGS += 
DEFS :=
LDFLAGS +=
LIBS +=

CXXFLAGS := -Wall  -O3

OUTDIR := Build

PROG := $(OUTDIR)/$(PROGNAME)
OBJS := $(SRCS:%.cpp=$(OUTDIR)/%.o)
DEPS := $(SRCS:%.cpp=$(OUTDIR)/%.d)

.PHONY: install clean distclean

all: $(PROG)

-include $(DEPS)

$(PROG): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

$(OUTDIR)/%.o:%.cpp
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(DEFS) -o $@ -c -MMD -MP -MF $(@:%.o=%.d) $<

clean:
	rm -rf $(OUTDIR)

distclean:
	rm -rf Build
