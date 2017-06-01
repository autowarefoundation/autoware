CC = g++
DEBUG = -g
CFLAGS = -Iinclude -I../op_utility/include -I../op_planner/include -Wall $(DEBUG) 
LFLAGS = -Llibs -L../op_planner/libs -L../op_utility/libs -lGL -lGLU -lglut -lpthread -lutility -lplanner -lopencv_core -lopencv_video -lopencv_highgui -Wl,-rpath=.:../op_utility/libs -Wl,-rpath=.:../op_planner/libs
SRC = $(wildcard src/*.cpp)
INCLUDES = $(wildcard include/*.h)
OBJ = $(SRC:.cpp=.o)
EXE = libs/Simu
LIB = libs/libsimu.so


all: $(LIB) $(EXE) 

pre-build:
	-@echo 'Post build commands'
	
$(EXE): main.o $(OBJ) 
	$(CC) -o $@ $< src/*.o $(LFLAGS) 
	
$(LIB): $(OBJ)
	$(CC) -o $(LIB) src/*.o $(LFLAGS) -shared

main.o: main.cpp
	$(CC) -o $@ $< $(CFLAGS) -c -fPIC

src/%.o: src/%.cpp
	$(CC) -o $@ -c $< $(CFLAGS) -fPIC
		
.PHONY: clean

clean:
	rm -rf src/*.o *.o libs/Simu  libs/libsimu.so
