/****************************************************************************
** Meta object code from reading C++ file 'processthread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.6.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../processthread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'processthread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.6.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ProcessThread_t {
    QByteArrayData data[29];
    char stringdata0[464];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ProcessThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ProcessThread_t qt_meta_stringdata_ProcessThread = {
    {
QT_MOC_LITERAL(0, 0, 13), // "ProcessThread"
QT_MOC_LITERAL(1, 14, 16), // "sendStatusUpdate"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 3), // "str"
QT_MOC_LITERAL(4, 36, 3), // "num"
QT_MOC_LITERAL(5, 40, 11), // "sendStepNum"
QT_MOC_LITERAL(6, 52, 16), // "sendImageAndSave"
QT_MOC_LITERAL(7, 69, 7), // "HObject"
QT_MOC_LITERAL(8, 77, 5), // "image"
QT_MOC_LITERAL(9, 83, 4), // "name"
QT_MOC_LITERAL(10, 88, 25), // "sendProcessingInformation"
QT_MOC_LITERAL(11, 114, 15), // "inputImageFirst"
QT_MOC_LITERAL(12, 130, 16), // "inputImageSecond"
QT_MOC_LITERAL(13, 147, 20), // "sendProcessingResult"
QT_MOC_LITERAL(14, 168, 16), // "planeparamStatus"
QT_MOC_LITERAL(15, 185, 26), // "CoreAlgorithm::PlaneNormal"
QT_MOC_LITERAL(16, 212, 10), // "planeparam"
QT_MOC_LITERAL(17, 223, 27), // "CoreAlgorithm::StereoCircle"
QT_MOC_LITERAL(18, 251, 12), // "centerResult"
QT_MOC_LITERAL(19, 264, 35), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(20, 300, 15), // "borderCloud_ptr"
QT_MOC_LITERAL(21, 316, 12), // "sendImageNum"
QT_MOC_LITERAL(22, 329, 16), // "receiveStepParam"
QT_MOC_LITERAL(23, 346, 14), // "CamNoScanSpeed"
QT_MOC_LITERAL(24, 361, 15), // "CamScanDistance"
QT_MOC_LITERAL(25, 377, 22), // "CamFirstNoScanDistance"
QT_MOC_LITERAL(26, 400, 23), // "CamSecondNoScanDistance"
QT_MOC_LITERAL(27, 424, 23), // "receiveProcessingResult"
QT_MOC_LITERAL(28, 448, 15) // "receiveImageNum"

    },
    "ProcessThread\0sendStatusUpdate\0\0str\0"
    "num\0sendStepNum\0sendImageAndSave\0"
    "HObject\0image\0name\0sendProcessingInformation\0"
    "inputImageFirst\0inputImageSecond\0"
    "sendProcessingResult\0planeparamStatus\0"
    "CoreAlgorithm::PlaneNormal\0planeparam\0"
    "CoreAlgorithm::StereoCircle\0centerResult\0"
    "pcl::PointCloud<pcl::PointXYZ>::Ptr\0"
    "borderCloud_ptr\0sendImageNum\0"
    "receiveStepParam\0CamNoScanSpeed\0"
    "CamScanDistance\0CamFirstNoScanDistance\0"
    "CamSecondNoScanDistance\0receiveProcessingResult\0"
    "receiveImageNum"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ProcessThread[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   59,    2, 0x06 /* Public */,
       5,    1,   64,    2, 0x06 /* Public */,
       6,    3,   67,    2, 0x06 /* Public */,
      10,    3,   74,    2, 0x06 /* Public */,
      13,    5,   81,    2, 0x06 /* Public */,
      21,    0,   92,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      22,    4,   93,    2, 0x0a /* Public */,
      27,    5,  102,    2, 0x0a /* Public */,
      28,    1,  113,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    3,    4,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void, 0x80000000 | 7, QMetaType::Int, QMetaType::QString,    8,    4,    9,
    QMetaType::Void, QMetaType::Int, 0x80000000 | 7, 0x80000000 | 7,    4,   11,   12,
    QMetaType::Void, QMetaType::Bool, 0x80000000 | 15, 0x80000000 | 17, 0x80000000 | 19, QMetaType::Int,   14,   16,   18,   20,    4,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   23,   24,   25,   26,
    QMetaType::Void, QMetaType::Bool, 0x80000000 | 15, 0x80000000 | 17, 0x80000000 | 19, QMetaType::Int,   14,   16,   18,   20,    4,
    QMetaType::Void, QMetaType::Int,    4,

       0        // eod
};

void ProcessThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ProcessThread *_t = static_cast<ProcessThread *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sendStatusUpdate((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: _t->sendStepNum((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->sendImageAndSave((*reinterpret_cast< HObject(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 3: _t->sendProcessingInformation((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< HObject(*)>(_a[2])),(*reinterpret_cast< HObject(*)>(_a[3]))); break;
        case 4: _t->sendProcessingResult((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< CoreAlgorithm::PlaneNormal(*)>(_a[2])),(*reinterpret_cast< CoreAlgorithm::StereoCircle(*)>(_a[3])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5]))); break;
        case 5: _t->sendImageNum(); break;
        case 6: _t->receiveStepParam((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        case 7: _t->receiveProcessingResult((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< CoreAlgorithm::PlaneNormal(*)>(_a[2])),(*reinterpret_cast< CoreAlgorithm::StereoCircle(*)>(_a[3])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5]))); break;
        case 8: _t->receiveImageNum((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ProcessThread::*_t)(QString , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ProcessThread::sendStatusUpdate)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (ProcessThread::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ProcessThread::sendStepNum)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (ProcessThread::*_t)(HObject , int , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ProcessThread::sendImageAndSave)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (ProcessThread::*_t)(int , HObject , HObject );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ProcessThread::sendProcessingInformation)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (ProcessThread::*_t)(bool , CoreAlgorithm::PlaneNormal , CoreAlgorithm::StereoCircle , pcl::PointCloud<pcl::PointXYZ>::Ptr , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ProcessThread::sendProcessingResult)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (ProcessThread::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ProcessThread::sendImageNum)) {
                *result = 5;
                return;
            }
        }
    }
}

const QMetaObject ProcessThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_ProcessThread.data,
      qt_meta_data_ProcessThread,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ProcessThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ProcessThread::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ProcessThread.stringdata0))
        return static_cast<void*>(const_cast< ProcessThread*>(this));
    return QThread::qt_metacast(_clname);
}

int ProcessThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void ProcessThread::sendStatusUpdate(QString _t1, int _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ProcessThread::sendStepNum(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ProcessThread::sendImageAndSave(HObject _t1, int _t2, QString _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void ProcessThread::sendProcessingInformation(int _t1, HObject _t2, HObject _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void ProcessThread::sendProcessingResult(bool _t1, CoreAlgorithm::PlaneNormal _t2, CoreAlgorithm::StereoCircle _t3, pcl::PointCloud<pcl::PointXYZ>::Ptr _t4, int _t5)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void ProcessThread::sendImageNum()
{
    QMetaObject::activate(this, &staticMetaObject, 5, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
