LEMON=../lemon-1.3

all: mwbm

%: %.cpp
	g++ -O3 $? -o $@ -I$(LEMON) -I$(LEMON)/build -L$(LEMON)/build/lemon -lemon

%_debug: %.cpp
	g++ -O0 -g $? -o $@ -I$(LEMON) -I$(LEMON)/build -L$(LEMON)/build/lemon -lemon

clean:
	rm -f mwbm mwbm_debug
