#////////////////////////////////////////////////////////////////////////////
#//
#//  This file is part of goalmax
#//


# Compiler, tools and options

RTIMULIBPATH       = IMU/RTIMULib
RTIMULibDrivePATH  = IMU/Linux/RTIMULibDrive
RTKSRCPATH         = RtkLib/src
RTKSRCPATH2        = RtkLib/src/rcv
RTKRCVPATH         = RtkLib/app/rtkrcv
FUSIONPATH	   = GpsImuFusion
HEARTPATH          = HeartRate

CTARGET		   = -DENAGLO -DENAQZS -DENACMP -DENAGAL -DENAIRN -DNFREQ=3 -DSVR_REUSEADDR

CC    		= gcc
CXX   		= g++

DEFINES       	=
RTKCFLAGS	= -Wall -O3 -ansi -pedantic -Wno-unused-but-set-variable -I$(RTKSRCPATH) -I.. -DTRACE $(CTARGET) -g
IMUCFLAGS	= -pipe -O2 -Wall -W $(DEFINES)
IMUCXXFLAGS     = -pipe -O2 -Wall -W $(DEFINES) -std=c++11
CXXFLAGS	= -std=c++11 -fPIC -Wall -g -pthread

INCPATH       	= -I. -I$(RTIMULIBPATH)
INCPATH2       	= -I. -I$(RTIMULibDrivePATH)
RTKINCPATH      = -I. -I$(RTKSRCPATH)
FUSIONINCPATH   = -I. -I$(FUSIONPATH)
HEARTINCPATH    = -I. -I$(HEARTPATH)

LINK  		= g++
LFLAGS		= -shared -O1 

#LIBS  		= -L/usr/lib/arm-linux-gnueabihf
LIBS  		= -lm -lrt -lpthread -lstdc++ -lmraa

COPY  		= cp -f
COPY_FILE     	= $(COPY)
COPY_DIR      	= $(COPY) -r

STRIP 		= strip
INSTALL_FILE  	= install -m 644 -p
INSTALL_DIR   	= $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      	= rm -f
SYMLINK       	= ln -f -s
DEL_DIR       	= rmdir
MOVE  		= mv -f
CHK_DIR_EXISTS	= test -d
MKDIR		= mkdir -p

# Output directory

OBJECTS_DIR   = objects/

# Files


GOALMAXDEPS  = goalmax.h \
    $(RTKSRCPATH)/rtklib.h \
    $(RTIMULIBPATH)/RTIMULib.h \
    $(FUSIONPATH)/GpsImuKalman.h

HEARTDEPS   = $(HEARTPATH)/heartRate.h \
    $(HEARTPATH)/SparkFunADS1015.h

FUSIONDEPS   = $(FUSIONPATH)/GpsImuKalman.h \
    $(FUSIONPATH)/basicMatrix.h

RTKDEPS      = $(RTKRCVPATH)/vt.h \
    $(RTKSRCPATH)/rtklib.h

IMUDEPS      = $(RTIMULibDrivePATH)/kiss_fftr.h \
    $(RTIMULibDrivePATH)/kiss_fft.h \
    $(RTIMULibDrivePATH)/data_processor.h \
    $(RTIMULibDrivePATH)/_kiss_fft_guts.h \
    $(RTIMULIBPATH)/RTMath.h \
    $(RTIMULIBPATH)/RTIMULib.h \
    $(RTIMULIBPATH)/RTIMULibDefs.h \
    $(RTIMULIBPATH)/RTIMUHal.h \
    $(RTIMULIBPATH)/RTFusion.h \
    $(RTIMULIBPATH)/RTFusionKalman4.h \
    $(RTIMULIBPATH)/RTFusionRTQF.h \
    $(RTIMULIBPATH)/RTIMUSettings.h \
    $(RTIMULIBPATH)/RTIMUAccelCal.h \
    $(RTIMULIBPATH)/RTIMUMagCal.h \
    $(RTIMULIBPATH)/RTIMUCalDefs.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMU.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUNull.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUMPU9150.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUMPU9250.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUGD20HM303D.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUGD20M303DLHC.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUGD20HM303DLHC.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMULSM9DS0.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMULSM9DS1.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUBMX055.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUBNO055.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressure.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureBMP180.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureLPS25H.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureMS5611.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureMS5637.h 

OBJECTS = objects/goalmax.o \
    objects/data_processor.o \
    objects/kiss_fft.o \
    objects/kiss_fftr.o \
    objects/heartRate.o \
    objects/SparkFunADS1015.o \
    objects/basicMatrix.o \
    objects/GpsImuKalman.o \
    objects/RTIMULibDrive.o \
    objects/RTMath.o \
    objects/RTIMUHal.o \
    objects/RTFusion.o \
    objects/RTFusionKalman4.o \
    objects/RTFusionRTQF.o \
    objects/RTIMUSettings.o \
    objects/RTIMUAccelCal.o \
    objects/RTIMUMagCal.o \
    objects/RTIMU.o \
    objects/RTIMUNull.o \
    objects/RTIMUMPU9150.o \
    objects/RTIMUMPU9250.o \
    objects/RTIMUGD20HM303D.o \
    objects/RTIMUGD20M303DLHC.o \
    objects/RTIMUGD20HM303DLHC.o \
    objects/RTIMULSM9DS0.o \
    objects/RTIMULSM9DS1.o \
    objects/RTIMUBMX055.o \
    objects/RTIMUBNO055.o \
    objects/RTPressure.o \
    objects/RTPressureBMP180.o \
    objects/RTPressureLPS25H.o \
    objects/RTPressureMS5611.o \
    objects/RTPressureMS5637.o \
    objects/rtkrcv.o \
    objects/vt.o \
    objects/rtkcmn.o \
    objects/rtksvr.o \
    objects/rtkpos.o \
    objects/geoid.o \
    objects/solution.o \
    objects/lambda.o \
    objects/sbas.o \
    objects/stream.o \
    objects/rcvraw.o \
    objects/rtcm.o \
    objects/preceph.o \
    objects/options.o \
    objects/pntpos.o \
    objects/ppp.o \
    objects/ppp_ar.o \
    objects/novatel.o \
    objects/ublox.o \
    objects/swiftnav.o \
    objects/tersus.o \
    objects/crescent.o \
    objects/skytraq.o \
    objects/gw10.o \
    objects/javad.o \
    objects/nvs.o \
    objects/binex.o \
    objects/rt17.o \
    objects/ephemeris.o \
    objects/rinex.o \
    objects/ionex.o \
    objects/rtcm2.o \
    objects/rtcm3.o \
    objects/rtcm3e.o \
    objects/qzslex.o \
    objects/ppp_corr.o \
    objects/tides.o \
    objects/septentrio.o \
    objects/cmr.o



MAKE_TARGET	= goalmax
DESTDIR		= Output/
TARGET		= $(MAKE_TARGET)

# Build rules

$(TARGET): $(OBJECTS)
	@$(CHK_DIR_EXISTS) Output/ || $(MKDIR) Output/
	$(LINK) $(LFLAGS) -Wl,-soname,$(TARGET).so -o $(TARGET).so $(OBJECTS) $(LIBS)

clean:
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


# Compile

$(OBJECTS_DIR)%.o : $(RTIMULibDrivePATH)/%.c $(IMUDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(IMUCFLAGS) $(INCPATH2)
	
$(OBJECTS_DIR)%.o : $(FUSIONPATH)/%.cpp $(FUSIONDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(FUSIONINCPATH)
	
$(OBJECTS_DIR)%.o : $(RTIMULIBPATH)/%.cpp $(IMUDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(IMUCFLAGS) $(INCPATH)
	
$(OBJECTS_DIR)%.o : $(RTIMULIBPATH)/IMUDrivers/%.cpp $(IMUDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(IMUCFLAGS) $(INCPATH)

$(OBJECTS_DIR)%.o : $(RTKSRCPATH)/%.c $(RTKDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(RTKCFLAGS) $(RTKINCPATH)
	
$(OBJECTS_DIR)%.o : $(RTKSRCPATH2)/%.c $(RTKDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(RTKCFLAGS) $(RTKINCPATH)

$(OBJECTS_DIR)%.o : $(HEARTPATH)/%.cpp $(HEARTDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(HEARTINCPATH)
	
$(OBJECTS_DIR)RTIMULibDrive.o : $(RTIMULibDrivePATH)/RTIMULibDrive.cpp $(IMUDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $(RTIMULibDrivePATH)/RTIMULibDrive.cpp $(IMUCXXFLAGS) $(INCPATH)

$(OBJECTS_DIR)goalmax.o : goalmax.cpp $(GOALMAXDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ goalmax.cpp $(CXXFLAGS)

$(OBJECTS_DIR)rtkrcv.o : $(RTKRCVPATH)/rtkrcv.c $(RTKDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $(RTKRCVPATH)/rtkrcv.c $(RTKCFLAGS) $(RTKINCPATH)

$(OBJECTS_DIR)vt.o : $(RTKRCVPATH)/vt.c $(RTKDEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $(RTKRCVPATH)/vt.c $(RTKCFLAGS) $(RTKINCPATH)

# Install

install_target: FORCE
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/usr/local/bin/ || $(MKDIR) $(INSTALL_ROOT)/usr/local/bin/
	-$(INSTALL_PROGRAM) "Output/$(MAKE_TARGET)" "$(INSTALL_ROOT)/usr/local/bin/$(MAKE_TARGET)"
	-$(STRIP) "$(INSTALL_ROOT)/usr/local/bin/$(MAKE_TARGET)"

uninstall_target:  FORCE
	-$(DEL_FILE) "$(INSTALL_ROOT)/usr/local/bin/$(MAKE_TARGET)"


install:  install_target  FORCE

uninstall: uninstall_target   FORCE

FORCE:

