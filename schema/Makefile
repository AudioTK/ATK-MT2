SRC_FILES := $(wildcard *.cir)
CPP_FILES := $(patsubst %.cir,%.cpp,$(SRC_FILES))
EXE_FILES := $(patsubst %.cir,%.exe,$(SRC_FILES))
DAT_FILES := $(patsubst %.cir,%.dat,$(SRC_FILES))
PNG_FILES := $(patsubst %.cir,%.png,$(SRC_FILES))

all: $(EXE_FILES) $(CPP_FILES) $(DAT_FILES) $(PNG_FILES)

%.cpp: %.cir
	ATKModellingGenerator $< $@

%.exe: %.cpp generate.cpp
	${CXX} -std=c++17 -O3 -DNDEBUG $< generate.cpp -o $@ -I /Users/matthieu/local/lib -I /Users/matthieu/local/include -I /Users/matthieu/local/boost_1_75_0 -L /Users/matthieu/local/lib -L /Users/matthieu/local/lib -lATKCore -lATKModelling

%.dat: %.exe
	./$< $@

%.png: %.dat display.py
	python3 display.py $< $@

.PHONY: all