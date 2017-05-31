CC = g++
DEBUG = -g
CFLAGS = -Iinclude -I../op_utilityh/include -Wall $(DEBUG)
#For GPS enabled conversion ! 
#LFLAGS = -Lbin -Llibs -lutilityh -lproj $(DEBUG) -std=c++11
LFLAGS = -L../op_utilityh/libs -Llibs -lutilityh $(DEBUG) -std=c++11
SRC = $(wildcard src/*.cpp)
BIN = $(wildcard bin/*.o)
INCLUDES = $(wildcard include/*.h)
OBJ = $(SRC:.cpp=.o)
LIB = libs/libplannerh.so


all:  $(LIB)  post-build

pre-build:
	-@echo ./src/$(OBJ)
	
$(LIB): $(OBJ)
	$(CC) -o $(LIB) src/*.o $(LFLAGS) -shared
	
src/%.o: src/%.cpp
	$(CC) -o $@ -c $< $(CFLAGS) -fPIC
#	mv src/*.o ./bin/	
	
post-build:	
	cp $(LIB) ../op_simuh/libs/

.PHONY: clean

clean:
	rm -rf src/*.o *.o libs/libplannerh.so 
