QT -= core
QT -= gui

TARGET = bubbleRob_test
TEMPLATE = app

DEFINES -= UNICODE
CONFIG   += console
CONFIG   -= app_bundle

DEFINES += NON_MATLAB_PARSING=1
DEFINES += MAX_EXT_API_CONNECTIONS=255

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings

    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs
}


win32 {
    LIBS += -lwinmm
    LIBS += -lWs2_32
}

macx {
}

unix:!macx {
    LIBS += -lrt
}


INCLUDEPATH += "../include"
INCLUDEPATH += "../remoteApi"

SOURCES += \
    bubbleRob_test.cpp \
    ../remoteApi/extApi.c \
    ../remoteApi/extApiPlatform.c \
    ../common/shared_memory.c

HEADERS +=\
    ../remoteApi/extApi.h \
    ../remoteApi/extApiPlatform.h \
    ../include/shared_memory.h

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}


















