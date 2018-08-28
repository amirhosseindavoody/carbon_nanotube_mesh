CC=g++-8

OPT = -O3 -Wall
# OPT = -g

CFLAGS = -std=c++17 -fopenmp

LFLAGS =  -std=c++17 -lz -fopenmp

SRCDIR = ./src
OBJDIR = ./obj
HOMDIR = .


# $(OBJDIR)/%.o: $(SRCDIR)/%.c
# 	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

# main: $(SRCDIR)/*.cpp
# 	@echo
# 	$(CC) $(OPT) -o $@.exe $(OBJDIR)/*.o $(LFLAGS)
# 	@echo

object:
	@echo
	@mkdir -p $(OBJDIR)
	$(CC) $(OPT) $(CFLAGS) -c $(SRCDIR)/*.cpp
	@mv -f ./*.o $(OBJDIR)

main: object
	@echo
	$(CC) $(OPT) -o $@.exe $(OBJDIR)/*.o $(LFLAGS)
	@echo

# Utility targets
.PHONY: clean
clean:
	@rm -f *.o *.exe
	@rm -rf $(OBJDIR)