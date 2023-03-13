all : test.exe

test.exe : test.o
	gcc -Wall -Werror -std=c99 -o test.exe test.o

test.o :
	gcc -c test.c

clean:
	del *.o