CC = g++
DEBUG = -g
CFLAGS = -Iinclude -I../op_utilityh/include -I../op_plannerh/include -Wall $(DEBUG) 
LFLAGS = -Llibs -L../op_plannerh/libs -L../op_utilityh/libs -lGL -lGLU -lglut -lpthread -lutilityh -lplannerh -lopencv_core -lopencv_video -lopencv_highgui -Wl,-rpath=.:../op_utilityh/libs -Wl,-rpath=.:../op_plannerh/libs
SRC = $(wildcard src/*.cpp)
INCLUDES = $(wildcard include/*.h)
OBJ = $(SRC:.cpp=.o)
EXE = libs/Simu
LIB = libs/libsimuh.so


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
	rm -rf src/*.o *.o libs/Simu  libs/libsimuh.so
