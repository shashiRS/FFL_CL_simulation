#******************************************************************************
#**  CarMaker - Version 8.1.2
#**  Vehicle Dynamics Simulation Toolkit
#**
#**  Copyright (C)   IPG Automotive GmbH
#**                  Bannwaldallee 60             Phone  +49.721.98520.0
#**                  76185 Karlsruhe              Fax    +49.721.98520.99
#**                  Germany                      WWW    www.ipg-automotive.com
#******************************************************************************
#**
#**  Library Makefile
#**  Run with 'make V=1 ...' for a verbose build.
#**
#******************************************************************************

include C:/IPG/carmaker/win64-8.1.2/include/MakeDefs.xeno

LIB_EXT =       .a
LIB_VER =		"lib_vcus_v1.0"
LIB_NAME =		lib_vcus.$(ARCH)$(LIB_EXT)

#OPT_CFLAGS =		-g -O1

LD_LIBS =	
OBJS =			Init_Clean.o Reflection_Generator.o

OBJS_xeno =		
OBJS_xeno64 =	

# Prepend local include/library directory to include path:
# PREINC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib

# Append local include/library directory to include path:
INC_CFLAGS +=		-I../ -I./

CXXFLAGS += -std=c++11
#CXXFLAGS += -DCML_CMATH_USE_STD_LIB -DAP_USE_NAMESPACES -DAP_USE_ENUM_CLASSES

### Linking with RTW-built Simulink models

#MATSUPP_MATVER =	R2015b
#LD_LIBS +=		$(MATSUPP_LIB)


# @@PLUGIN-BEGIN-LIBS@@ - Automatically generated code - don't edit!
# @@PLUGIN-END@@

### END (Linking with RTW-built Simulink models)


default:	$(LIB_NAME)
$(LIB_NAME):	$(OBJS) $(LD_LIBS_MK)
	$(QECHO) " AR     $@"
#$Q $(CXX) $(CXXFLAGS) $(LDFLAGS) -c -o $@ \
#		$(OBJS) $(LD_LIBS)
	$Q $(AR) rcs $@ *.o


install: $(LIB_NAME)
	$(INSTALL_APP) $(APP_NAME) $(ARCH)

clean:
	-rm -f 	*~ *% *.o *.a core

#app_tmp.c: Makefile $(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK)
#	$(QECHO) " MK     $@"
#	$Q $(CREATE_INFO_CMD)

depend .depend: Makefile
	$(QECHO) " MK     $@"
#	$Q-$(CC)  $(CFLAGS)   $(DEPCFLAGS)   *.c   >  .depend 2>/dev/null
	$Q-$(CXX) $(CXXFLAGS) $(DEPCXXFLAGS) *.cpp >> .depend 2>/dev/null
include .depend
