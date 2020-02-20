SRC = src/
BIN = bin/
OBJ = obj/

.PHONY: clean

################################
# Rules to compile under windows
all: windows

windows: Windows/$(BIN)driverSensor 
	
Windows/$(BIN)driverSensor: Windows/$(OBJ)main.o Windows/$(OBJ)libRQSensorWin.a
	gcc $^ -o $@

Windows/$(OBJ)%.o: Windows/$(SRC)%.c
	gcc -c -Wall -std=c99 -o $@ $<

data_logger: Windows/$(OBJ)libRQSensorWin.a
	gcc -c -Wall -std=c99 Windows/Data_logger/src/main.c -o Windows/Data_logger/obj/main.o
	gcc Windows/Data_logger/obj/main.o $< -o Windows/Data_logger/$(BIN)data_logger

# Compile the static library
Windows/$(OBJ)libRQSensorWin.a: $(OBJ)rq_sensor_com.o $(OBJ)rq_sensor_state.o
	@for i in $^; do cp $$i Windows/$$i; done
	ar rcs $@ $(addprefix Windows/,$^)

################################
# Rules to compile under linux
linux: Linux/$(BIN)driverSensor

Linux/$(BIN)driverSensor: Linux/$(BIN)libRQSensorLinux.a
	gcc -c Linux/$(SRC)main.c -o Linux/$(OBJ)main.o
	gcc Linux/$(OBJ)main.o $< -L./Linux/bin/ -lRQSensorLinux -o $@

Linux/$(OBJ)%.o: Linux/$(SRC)%.c
	gcc -c -Wall -std=c99 -o $@ $<

# regle pour creer la libRQSensorLinux
Linux/$(BIN)libRQSensorLinux.a: $(OBJ)rq_sensor_com.o $(OBJ)rq_sensor_state.o
	@for i in $^; do cp $$i Linux/$$i; done
	ar rcs $@ $(addprefix Linux/,$^)

# regle pour creer la libRQSensorUR
Linux/UR/$(BIN)libRQSensorUR.a: $(OBJ)rq_sensor_com.o $(OBJ)rq_sensor_state.o Linux/$(OBJ)rq_sensor_socket.o
	ar rcs $@ $^

Linux/$(OBJ)rq_sensor_socket.o:
	gcc -c Linux/$(SRC)rq_sensor_socket.c -o Linux/$(OBJ)rq_sensor_socket.o

universal: Linux/UR/$(BIN)libRQSensorUR.a
	gcc -c Linux/UR/main.c -o Linux/UR/main.o
	gcc -c $(SRC)Thread/rq_thread.c -o $(OBJ)Thread/thread.o
	gcc -c $(SRC)Thread/mutex.c -o $(OBJ)Thread/mutex.o
	gcc Linux/UR/main.o $(OBJ)Thread/thread.o $(OBJ)Thread/mutex.o -L./Linux/UR/$(BIN) -lRQSensorUR -lpthread -o Linux/UR/$(BIN)driverSensor

# regle pour compiler les .c en .o
$(OBJ)%.o: $(SRC)%.c
	gcc -c -Wall -std=c99 -o $@ $<

clean:
	-rm $(OBJ)Thread/*o
	-rm $(OBJ)*o
	-rm $(BIN)*
	-rm Linux/$(OBJ)*o
	-rm Linux/$(BIN)*
	-rm Linux/UR/*o
	-rm Linux/UR/bin/*
	-rm Windows/Data_logger/*o
	-rm Windows/$(OBJ)*
	-rm Windows/$(BIN)*
