all: comparisonArray.o voteAndClassify.o main.o 
	gcc -Wall -g -std=c11 -c -o comparisonArray.o comparisonArray.c
	gcc -Wall -g -std=c11 -c -o voteAndClassify.o voteAndClassify.c
	gcc -Wall -g -std=c11 -c -o main.o main.c
	gcc -Wall -g -std=c11 -o main main.o comparisonArray.o voteAndClassify.o

OBJFILES = comparisonArray.o voteAndClassify.o main.o
widget: $(OBJFILES)
	gcc -o widget $(OBJFILES)
clean:
	rm$(OBJFILES) widget *~