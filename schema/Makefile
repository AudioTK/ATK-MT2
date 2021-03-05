export DYLD_LIBRARY_PATH=/Users/matthieu/local/lib:/Users/matthieu/local/boost_1_75_0/stage/lib

CXXFLAGS= -I /Users/matthieu/local/lib -I /Users/matthieu/local/include -I /Users/matthieu/local/boost_1_75_0 -L /Users/matthieu/local/lib -L /Users/matthieu/local/lib

SRC_FILES := $(wildcard *.cir)
CPP_FILES := $(patsubst %.cir,%.cpp,$(SRC_FILES)) 
EXE_FILES := $(patsubst %.cpp,%.exe,$(CPP_FILES)) generate_full.exe test_high_svf.exe test_mid_svf.exe
DAT_FILES := $(patsubst %.exe,%.dat,$(EXE_FILES))
PNG_FILES := $(patsubst %.exe,%.png,$(EXE_FILES))

all: $(EXE_FILES) $(CPP_FILES) $(DAT_FILES) $(PNG_FILES)

%.cpp: %.cir
	ATKModellingGenerator $< $@

%.exe: %.cpp generate.cpp
	${CXX} -std=c++17 -O3 -DNDEBUG $< generate.cpp -o $@ $(CXXFLAGS) -lATKCore -lATKModelling

generate_full.exe: generate_full.cpp
	${CXX} -std=c++17 -O3 -DNDEBUG $< ../MT2/Source/0*.cpp -o $@ $(CXXFLAGS) -lATKCore -lATKEQ -lATKTools -lATKModelling

test_high_svf.exe: test_high_svf.cpp
	${CXX} -std=c++17 -O3 -DNDEBUG $< -o $@ $(CXXFLAGS) -lATKCore -lATKEQ -lATKTools

test_mid_svf.exe: test_mid_svf.cpp
	${CXX} -std=c++17 -O3 -DNDEBUG $< -o $@ $(CXXFLAGS) -lATKCore -lATKEQ -lATKTools

%.dat: %.exe
	./$< $@

%.png: %.dat display.py
	python3 display.py $< $@

clean:
	rm -f $(CPP_FILES) $(EXE_FILES) $(DAT_FILES) $(PNG_FILES)

.PHONY: all clean