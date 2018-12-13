/****************************************************************************
** Meta object code from reading C++ file 'motioncontrol.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../motioncontrol.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'motioncontrol.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MotionControl_t {
    QByteArrayData data[15];
    char stringdata0[282];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MotionControl_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MotionControl_t qt_meta_stringdata_MotionControl = {
    {
QT_MOC_LITERAL(0, 0, 13), // "MotionControl"
QT_MOC_LITERAL(1, 14, 8), // "sendInfo"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 9), // "toPLCInfo"
QT_MOC_LITERAL(4, 34, 4), // "Byte"
QT_MOC_LITERAL(5, 39, 24), // "on_forwardButton_pressed"
QT_MOC_LITERAL(6, 64, 25), // "on_forwardButton_released"
QT_MOC_LITERAL(7, 90, 25), // "on_backwardButton_pressed"
QT_MOC_LITERAL(8, 116, 26), // "on_backwardButton_released"
QT_MOC_LITERAL(9, 143, 21), // "on_leftButton_pressed"
QT_MOC_LITERAL(10, 165, 22), // "on_leftButton_released"
QT_MOC_LITERAL(11, 188, 22), // "on_rightButton_pressed"
QT_MOC_LITERAL(12, 211, 23), // "on_rightButton_released"
QT_MOC_LITERAL(13, 235, 22), // "on_resetButton_clicked"
QT_MOC_LITERAL(14, 258, 23) // "on_rotateButton_clicked"

    },
    "MotionControl\0sendInfo\0\0toPLCInfo\0"
    "Byte\0on_forwardButton_pressed\0"
    "on_forwardButton_released\0"
    "on_backwardButton_pressed\0"
    "on_backwardButton_released\0"
    "on_leftButton_pressed\0on_leftButton_released\0"
    "on_rightButton_pressed\0on_rightButton_released\0"
    "on_resetButton_clicked\0on_rotateButton_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MotionControl[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   74,    2, 0x06 /* Public */,
       3,    1,   79,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   82,    2, 0x0a /* Public */,
       6,    0,   83,    2, 0x0a /* Public */,
       7,    0,   84,    2, 0x0a /* Public */,
       8,    0,   85,    2, 0x0a /* Public */,
       9,    0,   86,    2, 0x0a /* Public */,
      10,    0,   87,    2, 0x0a /* Public */,
      11,    0,   88,    2, 0x0a /* Public */,
      12,    0,   89,    2, 0x0a /* Public */,
      13,    0,   90,    2, 0x0a /* Public */,
      14,    0,   91,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    2,    2,
    QMetaType::Void, QMetaType::QByteArray,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MotionControl::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MotionControl *_t = static_cast<MotionControl *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sendInfo((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: _t->toPLCInfo((*reinterpret_cast< QByteArray(*)>(_a[1]))); break;
        case 2: _t->on_forwardButton_pressed(); break;
        case 3: _t->on_forwardButton_released(); break;
        case 4: _t->on_backwardButton_pressed(); break;
        case 5: _t->on_backwardButton_released(); break;
        case 6: _t->on_leftButton_pressed(); break;
        case 7: _t->on_leftButton_released(); break;
        case 8: _t->on_rightButton_pressed(); break;
        case 9: _t->on_rightButton_released(); break;
        case 10: _t->on_resetButton_clicked(); break;
        case 11: _t->on_rotateButton_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MotionControl::*_t)(QString , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MotionControl::sendInfo)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (MotionControl::*_t)(QByteArray );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MotionControl::toPLCInfo)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject MotionControl::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_MotionControl.data,
      qt_meta_data_MotionControl,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *MotionControl::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MotionControl::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MotionControl.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int MotionControl::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void MotionControl::sendInfo(QString _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MotionControl::toPLCInfo(QByteArray _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
