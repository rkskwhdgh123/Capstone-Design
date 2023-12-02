CX = g++
CCFLAGS = -g -Wall

DXLFLAGS = -I/usr/local/include/dynamixel_sdk_cpp
DXLFLAGS += -ldxl_x64_cpp
DXLFLAGS += -lrt

TARGET = linetest
OBJS = main.o line.o dxl.o
OPENCV = `pkg-config opencv4 --cflags --libs`   # opencv 헤더파일경로, 링크할 라이브러리 파일을 자동으로 찾아줌, ` 는 탭키위에 있는 grave accent 문자임
LIBS = $(OPENCV)

$(TARGET) :  $(OBJS)
	$(CX) $(CCFLAGS) -o $(TARGET) $(OBJS) $(LIBS) $(DXLFLAGS)
main.o : main.cpp line.cpp dxl.cpp
	$(CX) $(CCFLAGS) -c main.cpp line.cpp $(LIBS) $(DXLFLAGS)
line.o : line.hpp line.cpp 
	$(CX) $(CCFLAGS) -c line.cpp  $(LIBS) 
dxl.o : dxl.cpp dxl.hpp
	$(CX) $(CCFLAGS) -c dxl.cpp  $(LIBS) $(DXLFLAGS)
.PHONY: all clean

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(OBJS)
