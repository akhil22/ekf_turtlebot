INC_DIR= ../../include
LIB_DIR= ../../lib
BIN_DIR= ../../bin

LDFLAG=-L/usr/X11R6/lib -L$(LIB_DIR)
LIBS= -lARgsub -lARvideo -lAR -lglut -lGLU -lGL -lXi -lX11 -lm -lpthread
CFLAG= -O -I/usr/X11R6/include -I$(INC_DIR)

OBJS = draw_object.o
HEADDERS = draw_object.h

all: $(BIN_DIR)/exview

$(BIN_DIR)/exview: exview.o $(OBJS)
	cc -o $(BIN_DIR)/exview exview.o $(OBJS) $(LDFLAG) $(LIBS)

exview.o: exview.c $(HEADDERS)
	cc -c $(CFLAG) exview.c

draw_object.o: draw_object.c $(HEADDERS)
	cc -c $(CFLAG) draw_object.c

clean:
	rm -f *.o
	rm -f $(BIN_DIR)/exview

allclean:
	rm -f *.o
	rm -f $(BIN_DIR)/exview
	rm -f Makefile
