ARTIFACT = diff_drive
all:
	g++ -o $(ARTIFACT) diff_drive.cpp -lwiringPi -lpthread

clean:
	rm $(ARTIFACT)
