CC = g++
DEBUG = -g
CFLAGS = -Iinclude -Wall $(DEBUG)
LFLAGS = -Llibs -Lbin -ltinyxml $(DEBUG) 
SRC = $(wildcard src/*.cpp)
INCLUDES = $(wildcard include/*.h)
BIN = $(wildcard bin/*.o)
OBJ = $(SRC:.cpp=.o)
LIB = libs/libutility.so

all: $(LIB) post-build


pre-build:
	-@echo 'pre build commands'
	
$(LIB): $(OBJ)
	$(CC) -o $(LIB) src/*.o $(LFLAGS) -shared

src/%.o: src/%.cpp
	$(CC) -o $@ -c $< $(CFLAGS) -fPIC
	 	

post-build:	
	cp $(LIB) ../op_simu/libs/
	
.PHONY: clean

clean:
	rm -rf src/*.o *.o libs/libutility.so 
