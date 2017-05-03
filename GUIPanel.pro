#-------------------------------------------------
#
# Project created by QtCreator 2013-04-05T19:35:15
#
# Compatible desde Qt5.3 en adelante
#-------------------------------------------------

QT       += core gui serialport widgets

TARGET = GUIPanel
TEMPLATE = app


SOURCES += main.cpp\
        guipanel.cpp \
    crc.c \
    protocol.c

HEADERS  += guipanel.h \
    crc.h \
    protocol.h

FORMS    += guipanel.ui

win32:debug{
    LIBS += -lqwtd -lanalogwidgetsd # Añadido Necesario para encontrar implementación de librerias (si estoy en windows y en modo debug)!!
}

win32:release{
    LIBS += -lqwt -lanalogwidgets # Añadido Necesario para encontrar implementación de librerias (si estoy en windows y en modo release)!!
}

unix{
    LIBS += -lqwt -lanalogwidgets # Añadido Necesario para encontrar implementación de librerias (Linux)!!
    LIBS += -lColorWidgets-qt5

}
