TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

QT_CONFIG -= no-pkg-config
CONFIG += link_pkgconfig
PKGCONFIG += libusb-1.0

SOURCES += \
    sonix.c \
#    sensors/sn9c102_ov7660.c

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    sonix.h \
    sensors/sn9c102_sensor.h

