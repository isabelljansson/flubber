OBJECTS :=\
	main.o\
	ParticleSystem.o

SOURCE :=\
	src/ParticleSystem.cpp\
	src/main.cpp
 
 
 
 
CC := g++
HEADERFILES := -I./include/
LDFLAGS := -o
CPFLAGS := -c -g -std=c++11


all: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) out $(INCLUDES)

$(OBJECTS): $(SOURCE)
	$(CC) $(HEADERFILES) $(SOURCE) $(CPFLAGS)

clean:
	rm -f *.o
