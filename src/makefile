CXX	= g++
CONFIG  = `pkg-config --cflags --libs opencv`
GLFLAGS = -I/usr/X11R6/include
GLLIBS = -L/usr/X11R6/lib -lglut -lGLU -lGL -lXmu -lXi -lXext -lX11 -lm -lpthread
MAIN_NAME = TGmain
OBJS = CVRCParameter.o particle.o StringModel.o FCalculator.o timer.o box.o polygon.o ClothSimulator.o attractor.o TrajectoryGenerator.o
MAIN = TGmain.cpp
.SUFFIXES: .o .cpp 

$(MAIN_NAME): $(MAIN) $(OBJS)
	$(CXX) $(GLFLAGS) -o $@ $^ $(CONFIG) $(GLLIBS)

.cpp.o:
	$(CXX) $(CONFIG) -c $<

clean:
	rm -f *.o $(MAIN_NAME)
