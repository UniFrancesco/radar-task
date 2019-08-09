#---------------------------------------------------
# Target file to be compiled by default
#---------------------------------------------------
MAIN = radar
#---------------------------------------------------
# CC will be the compiler to use
#---------------------------------------------------
CC = gcc
#---------------------------------------------------
# CFLAGS will be the options passed to the compiler
#---------------------------------------------------
CFLAGS = -Wall -lpthread -lrt
#---------------------------------------------------
# Dependencies
#---------------------------------------------------
$(MAIN): $(MAIN).o 
	$(CC) $(CFLAGS) -o $(MAIN) $(MAIN).o libptask.a  -lpthread -lrt -lm     `allegro-config --libs`

$(MAIN).o: $(MAIN).c
	$(CC) -c $(MAIN).c -I./include

