SRC_FILES := $(wildcard *.cir)
CPP_FILES := $(patsubst %.cir,%.cpp,$(SRC_FILES))

all: $(CPP_FILES)

%.cpp: %.cir
	ATKModellingGenerator $< $@

PHONY: all