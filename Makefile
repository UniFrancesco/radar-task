#-------------------------------------------------------------------------------
# Target file to be compiled by default
#-------------------------------------------------------------------------------
MAIN = radar

#-------------------------------------------------------------------------------
# CC will be the compiler to use
#-------------------------------------------------------------------------------
CC = gcc

#-------------------------------------------------------------------------------
# Compiler options
#-------------------------------------------------------------------------------
CFLAGS = -Wall -lrt -lm

#-------------------------------------------------------------------------------
# External headers
#-------------------------------------------------------------------------------
INCLUDE = -I./include

#-------------------------------------------------------------------------------
# Allegro Dependencies
#-------------------------------------------------------------------------------
ALIBS = -lpthread  `allegro-config --libs`

#-------------------------------------------------------------------------------
# Ptask static library
#-------------------------------------------------------------------------------
PLIBS = libptask.a

$(MAIN): $(MAIN).o 
	$(CC) -o $(MAIN) $(MAIN).o $(PLIBS) $(CFLAGS) $(ALIBS)

$(MAIN).o: $(MAIN).c
	$(CC) -c $(MAIN).c $(INCLUDE) -Wall
	
clean:
	rm $(MAIN).o $(MAIN)

