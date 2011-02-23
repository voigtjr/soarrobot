# This Makefile was tested with GNU Make

# Use pkg-config to lookup the proper compiler and linker flags for LCM
CFLAGS= -g -O0 -Wall -Werror `c_urg-config --cflags` -I$(HOME)/sandbox/include
LDFLAGS= `c_urg-config --libs` -L$(HOME)/sandbox/lib -llcm

# list the LCM types here
lcmtypes=urg_range_t

# derive the file names of the .c, .h, and .o files corresponding
# to the LCM types here
lcm_c_h_files=$(lcmtypes:%=%.c) $(lcmtypes:%=%.h)
lcm_obj_files=$(lcmtypes:%=%.o)

# compile the LCM types to a static library 
all: libtypes.a urg_lcm

urg_lcm: urg_lcm.o libtypes.a
	$(CC) -o $@ $^ $(LDFLAGS)

# prevent auto-generated lcm .c/.h files from being deleted
.SECONDARY : $(lcm_c_h_files)

libtypes.a: $(lcm_obj_files)
	ar rc $@ $(lcm_obj_files)
	ranlib $@

%.o: %.c %.h
	gcc $(CFLAGS) -c $< 

%.c %.h: %.lcm
	lcm-gen -c $<

clean:
	rm -f urg_lcm
	rm -f *.o
	rm -f libtypes.a
	# rm -f $(lcm_c_h_files)
