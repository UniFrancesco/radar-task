#---------------------------------------------------
# Target file to be compiled by default
#---------------------------------------------------
MAIN = radar
#---------------------------------------------------
# CC will be the compiler to use
#---------------------------------------------------
CC = gcc
#---------------------------------------------------
# Dependencies
#---------------------------------------------------
$(MAIN): $(MAIN).o 
	$(CC) -o $(MAIN) $(MAIN).o libptask.a  -lpthread -lrt -lm  `allegro-config --libs`

$(MAIN).o: $(MAIN).c
	$(CC) -c $(MAIN).c -I./include -Wall

