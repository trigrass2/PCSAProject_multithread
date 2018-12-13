/****************************************************************************
** Meta object code from reading C++ file 'networkcommunication.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../networkcommunication.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'networkcommunication.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_NetworkCommunication_t {
    QByteArrayData data[14];
    char stringdata0[211];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_NetworkCommunication_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_NetworkCommunication_t qt_meta_stringdata_NetworkCommunication = {
    {
QT_MOC_LITERAL(0, 0, 20), // "NetworkCommunication"
QT_MOC_LITERAL(1, 21, 14), // "stringReceived"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 8), // "sendInfo"
QT_MOC_LITERAL(4, 46, 24), // "on_ConnectButton_clicked"
QT_MOC_LITERAL(5, 71, 27), // "on_DisconnectButton_clicked"
QT_MOC_LITERAL(6, 99, 21), // "on_SendButton_clicked"
QT_MOC_LITERAL(7, 121, 21), // "on_SaveButton_clicked"
QT_MOC_LITERAL(8, 143, 14), // "displayReceive"
QT_MOC_LITERAL(9, 158, 4), // "info"
QT_MOC_LITERAL(10, 163, 15), // "checkConnection"
QT_MOC_LITERAL(11, 179, 4), // "erro"
QT_MOC_LITERAL(12, 184, 16), // "receiveSendAgain"
QT_MOC_LITERAL(13, 201, 9) // "readyRead"

    },
    "NetworkCommunication\0stringReceived\0"
    "\0sendInfo\0on_ConnectButton_clicked\0"
    "on_DisconnectButton_clicked\0"
    "on_SendButton_clicked\0on_SaveButton_clicked\0"
    "displayReceive\0info\0checkConnection\0"
    "erro\0receiveSendAgain\0readyRead"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_NetworkCommunication[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x06 /* Public */,
       3,    2,   72,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   77,    2, 0x0a /* Public */,
       5,    0,   78,    2, 0x0a /* Public */,
       6,    0,   79,    2, 0x0a /* Public */,
       7,    0,   80,    2, 0x0a /* Public */,
       8,    1,   81,    2, 0x0a /* Public */,
      10,    0,   84,    2, 0x0a /* Public */,
      11,    0,   85,    2, 0x0a /* Public */,
      12,    0,   86,    2, 0x0a /* Public */,
      13,    0,   87,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    2,    2,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    9,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void NetworkCommunication::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        NetworkCommunication *_t = static_cast<NetworkCommunication *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->stringReceived((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->sendInfo((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->on_ConnectButton_clicked(); break;
        case 3: _t->on_DisconnectButton_clicked(); break;
        case 4: _t->on_SendButton_clicked(); break;
        case 5: _t->on_SaveButton_clicked(); break;
        case 6: _t->displayReceive((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->checkConnection(); break;
        case 8: _t->erro(); break;
        case 9: _t->receiveSendAgain(); break;
        case 10: _t->readyRead(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (NetworkCommunication::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&NetworkCommunication::stringReceived)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (NetworkCommunication::*_t)(QString , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&NetworkCommunication::sendInfo)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject NetworkCommunication::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_NetworkCommunication.data,
      qt_meta_data_NetworkCommunication,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *NetworkCommunication::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *NetworkCommunication::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_NetworkCommunication.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int NetworkCommunication::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void NetworkCommunication::stringReceived(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void NetworkCommunication::sendInfo(QString _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
