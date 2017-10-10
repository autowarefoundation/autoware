CC = g++
DEBUG = -g
CFLAGS = -Iinclude -I../op_utility/include -Wall $(DEBUG)
#For GPS enabled conversion ! 
#LFLAGS = -Lbin -Llibs -lutilityh -lproj $(DEBUG) -std=c++11
LFLAGS = -L../op_utility/libs -Llibs -lutility $(DEBUG) -std=c++11
SRC = $(wildcard src/*.cpp)
BIN = $(wildcard bin/*.o)
INCLUDES = $(wildcard include/*.h)
OBJ = $(SRC:.cpp=.o)
LIB = libs/libplanner.so


all:  $(LIB)  post-build

pre-build:
	-@echo ./src/$(OBJ)
	
$(LIB): $(OBJ)
	$(CC) -o $(LIB) src/*.o $(LFLAGS) -shared
	
src/%.o: src/%.cpp
	$(CC) -o $@ -c $< $(CFLAGS) -fPIC
#	mv src/*.o ./bin/	
	
post-build:	
	cp $(LIB) ../op_simu/libs/

.PHONY: clean

clean:
	rm -rf src/*.o *.o libs/libplanner.so 
