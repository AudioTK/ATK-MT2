SRC_FILES := $(wildcard *.cir)
CPP_FILES := $(patsubst %.cir,%.cpp,$(SRC_FILES)) 
EXE_FILES := $(patsubst %.cpp,%.exe,$(CPP_FILES)) generate_full.exe
DAT_FILES := $(patsubst %.exe,%.dat,$(EXE_FILES))
PNG_FILES := $(patsubst %.exe,%.png,$(EXE_FILES))

all: $(EXE_FILES) $(CPP_FILES) $(DAT_FILES) $(PNG_FILES)

%.cpp: %.cir
	ATKModellingGenerator $< $@

%.exe: %.cpp generate.cpp
	${CXX} -std=c++17 -O3 -DNDEBUG $< generate.cpp -o $@ $(CXXFLAGS) -lATKCore -lATKModelling

generate_full.exe: generate_full.cpp
	${CXX} -std=c++17 -O3 -DNDEBUG $< ../MT2/Source/0*.cpp -o $@ $(CXXFLAGS) -lATKCore -lATKEQ -lATKTools -lATKModelling

%.dat: %.exe
	./$< $@

%.png: %.dat display.py
	python3 display.py $< $@

clean:
	rm -f $(CPP_FILES) $(EXE_FILES) $(DAT_FILES) $(PNG_FILES)

.PHONY: all clean