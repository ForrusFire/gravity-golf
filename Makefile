all: compile link


OBJECTS := $(patsubst src/%.cpp, build/%.o, $(wildcard src/*.cpp))

build/%.o: src/%.cpp
	g++ -I include -c -o $@ $<


compile: $(OBJECTS)

link:
	g++ $(OBJECTS) -o main -L lib -l sfml-graphics -l sfml-window -l sfml-system