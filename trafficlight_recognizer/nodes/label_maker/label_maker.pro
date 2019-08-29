QT  += widgets

FORMS += \
    label_maker_gui.ui

INCLUDEPATH += $$_PRO_FILE_PWD_/../../include/trafficlight_recognizer/label_maker

HEADERS += \
    label_maker_gui.h \
    custom_graphics_view.h \
    file_system_operator.h

SOURCES += \
    label_maker_gui.cpp \
    label_maker.cpp \
    custom_graphics_view.cpp \
    file_system_operator.cpp
