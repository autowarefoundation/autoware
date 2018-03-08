/****************************************************************************
** Meta object code from reading C++ file 'calibrationtoolkit.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "calibrationtoolkit.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QVector>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'calibrationtoolkit.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_CalibrationToolkitBase_t {
    QByteArrayData data[17];
    char stringdata0[379];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CalibrationToolkitBase_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CalibrationToolkitBase_t qt_meta_stringdata_CalibrationToolkitBase = {
    {
QT_MOC_LITERAL(0, 0, 22), // "CalibrationToolkitBase"
QT_MOC_LITERAL(1, 23, 22), // "calibDataGrabbedSignal"
QT_MOC_LITERAL(2, 46, 0), // ""
QT_MOC_LITERAL(3, 47, 27), // "calibDataGrabbedErrorSignal"
QT_MOC_LITERAL(4, 75, 22), // "calibDataRemovedSignal"
QT_MOC_LITERAL(5, 98, 27), // "calibDataRemovedErrorSignal"
QT_MOC_LITERAL(6, 126, 22), // "sensorCalibratedSignal"
QT_MOC_LITERAL(7, 149, 27), // "sensorCalibratedErrorSignal"
QT_MOC_LITERAL(8, 177, 23), // "calibResultLoadedSignal"
QT_MOC_LITERAL(9, 201, 28), // "calibResultLoadedErrorSignal"
QT_MOC_LITERAL(10, 230, 22), // "calibResultSavedSignal"
QT_MOC_LITERAL(11, 253, 27), // "calibResultSavedErrorSignal"
QT_MOC_LITERAL(12, 281, 17), // "grabCalibDataSlot"
QT_MOC_LITERAL(13, 299, 19), // "removeCalibDataSlot"
QT_MOC_LITERAL(14, 319, 19), // "calibrateSensorSlot"
QT_MOC_LITERAL(15, 339, 19), // "loadCalibResultSlot"
QT_MOC_LITERAL(16, 359, 19) // "saveCalibResultSlot"

    },
    "CalibrationToolkitBase\0calibDataGrabbedSignal\0"
    "\0calibDataGrabbedErrorSignal\0"
    "calibDataRemovedSignal\0"
    "calibDataRemovedErrorSignal\0"
    "sensorCalibratedSignal\0"
    "sensorCalibratedErrorSignal\0"
    "calibResultLoadedSignal\0"
    "calibResultLoadedErrorSignal\0"
    "calibResultSavedSignal\0"
    "calibResultSavedErrorSignal\0"
    "grabCalibDataSlot\0removeCalibDataSlot\0"
    "calibrateSensorSlot\0loadCalibResultSlot\0"
    "saveCalibResultSlot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CalibrationToolkitBase[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      10,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   89,    2, 0x06 /* Public */,
       3,    0,   90,    2, 0x06 /* Public */,
       4,    0,   91,    2, 0x06 /* Public */,
       5,    0,   92,    2, 0x06 /* Public */,
       6,    0,   93,    2, 0x06 /* Public */,
       7,    0,   94,    2, 0x06 /* Public */,
       8,    0,   95,    2, 0x06 /* Public */,
       9,    0,   96,    2, 0x06 /* Public */,
      10,    0,   97,    2, 0x06 /* Public */,
      11,    0,   98,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      12,    0,   99,    2, 0x0a /* Public */,
      13,    0,  100,    2, 0x0a /* Public */,
      14,    0,  101,    2, 0x0a /* Public */,
      15,    0,  102,    2, 0x0a /* Public */,
      16,    0,  103,    2, 0x0a /* Public */,

 // signals: parameters
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

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CalibrationToolkitBase::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        CalibrationToolkitBase *_t = static_cast<CalibrationToolkitBase *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->calibDataGrabbedSignal(); break;
        case 1: _t->calibDataGrabbedErrorSignal(); break;
        case 2: _t->calibDataRemovedSignal(); break;
        case 3: _t->calibDataRemovedErrorSignal(); break;
        case 4: _t->sensorCalibratedSignal(); break;
        case 5: _t->sensorCalibratedErrorSignal(); break;
        case 6: _t->calibResultLoadedSignal(); break;
        case 7: _t->calibResultLoadedErrorSignal(); break;
        case 8: _t->calibResultSavedSignal(); break;
        case 9: _t->calibResultSavedErrorSignal(); break;
        case 10: _t->grabCalibDataSlot(); break;
        case 11: _t->removeCalibDataSlot(); break;
        case 12: _t->calibrateSensorSlot(); break;
        case 13: _t->loadCalibResultSlot(); break;
        case 14: _t->saveCalibResultSlot(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (CalibrationToolkitBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrationToolkitBase::calibDataGrabbedSignal)) {
                *result = 0;
            }
        }
        {
            typedef void (CalibrationToolkitBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrationToolkitBase::calibDataGrabbedErrorSignal)) {
                *result = 1;
            }
        }
        {
            typedef void (CalibrationToolkitBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrationToolkitBase::calibDataRemovedSignal)) {
                *result = 2;
            }
        }
        {
            typedef void (CalibrationToolkitBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrationToolkitBase::calibDataRemovedErrorSignal)) {
                *result = 3;
            }
        }
        {
            typedef void (CalibrationToolkitBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrationToolkitBase::sensorCalibratedSignal)) {
                *result = 4;
            }
        }
        {
            typedef void (CalibrationToolkitBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrationToolkitBase::sensorCalibratedErrorSignal)) {
                *result = 5;
            }
        }
        {
            typedef void (CalibrationToolkitBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrationToolkitBase::calibResultLoadedSignal)) {
                *result = 6;
            }
        }
        {
            typedef void (CalibrationToolkitBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrationToolkitBase::calibResultLoadedErrorSignal)) {
                *result = 7;
            }
        }
        {
            typedef void (CalibrationToolkitBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrationToolkitBase::calibResultSavedSignal)) {
                *result = 8;
            }
        }
        {
            typedef void (CalibrationToolkitBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrationToolkitBase::calibResultSavedErrorSignal)) {
                *result = 9;
            }
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject CalibrationToolkitBase::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_CalibrationToolkitBase.data,
      qt_meta_data_CalibrationToolkitBase,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CalibrationToolkitBase::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CalibrationToolkitBase::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CalibrationToolkitBase.stringdata0))
        return static_cast<void*>(const_cast< CalibrationToolkitBase*>(this));
    return QWidget::qt_metacast(_clname);
}

int CalibrationToolkitBase::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void CalibrationToolkitBase::calibDataGrabbedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void CalibrationToolkitBase::calibDataGrabbedErrorSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void CalibrationToolkitBase::calibDataRemovedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void CalibrationToolkitBase::calibDataRemovedErrorSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void CalibrationToolkitBase::sensorCalibratedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}

// SIGNAL 5
void CalibrationToolkitBase::sensorCalibratedErrorSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 5, Q_NULLPTR);
}

// SIGNAL 6
void CalibrationToolkitBase::calibResultLoadedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 6, Q_NULLPTR);
}

// SIGNAL 7
void CalibrationToolkitBase::calibResultLoadedErrorSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 7, Q_NULLPTR);
}

// SIGNAL 8
void CalibrationToolkitBase::calibResultSavedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 8, Q_NULLPTR);
}

// SIGNAL 9
void CalibrationToolkitBase::calibResultSavedErrorSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 9, Q_NULLPTR);
}
struct qt_meta_stringdata_CalibrateCameraBase_t {
    QByteArrayData data[6];
    char stringdata0[107];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CalibrateCameraBase_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CalibrateCameraBase_t qt_meta_stringdata_CalibrateCameraBase = {
    {
QT_MOC_LITERAL(0, 0, 19), // "CalibrateCameraBase"
QT_MOC_LITERAL(1, 20, 20), // "imageRefreshedSignal"
QT_MOC_LITERAL(2, 41, 0), // ""
QT_MOC_LITERAL(3, 42, 25), // "imageRefreshedErrorSignal"
QT_MOC_LITERAL(4, 68, 16), // "refreshImageSlot"
QT_MOC_LITERAL(5, 85, 21) // "refreshParametersSlot"

    },
    "CalibrateCameraBase\0imageRefreshedSignal\0"
    "\0imageRefreshedErrorSignal\0refreshImageSlot\0"
    "refreshParametersSlot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CalibrateCameraBase[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x06 /* Public */,
       3,    0,   35,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   36,    2, 0x09 /* Protected */,
       5,    0,   37,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CalibrateCameraBase::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        CalibrateCameraBase *_t = static_cast<CalibrateCameraBase *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->imageRefreshedSignal(); break;
        case 1: _t->imageRefreshedErrorSignal(); break;
        case 2: _t->refreshImageSlot(); break;
        case 3: _t->refreshParametersSlot(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (CalibrateCameraBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrateCameraBase::imageRefreshedSignal)) {
                *result = 0;
            }
        }
        {
            typedef void (CalibrateCameraBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrateCameraBase::imageRefreshedErrorSignal)) {
                *result = 1;
            }
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject CalibrateCameraBase::staticMetaObject = {
    { &CalibrationToolkitBase::staticMetaObject, qt_meta_stringdata_CalibrateCameraBase.data,
      qt_meta_data_CalibrateCameraBase,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CalibrateCameraBase::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CalibrateCameraBase::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CalibrateCameraBase.stringdata0))
        return static_cast<void*>(const_cast< CalibrateCameraBase*>(this));
    return CalibrationToolkitBase::qt_metacast(_clname);
}

int CalibrateCameraBase::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = CalibrationToolkitBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void CalibrateCameraBase::imageRefreshedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void CalibrateCameraBase::imageRefreshedErrorSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}
struct qt_meta_stringdata_CalibrateCameraChessboardBase_t {
    QByteArrayData data[1];
    char stringdata0[30];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CalibrateCameraChessboardBase_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CalibrateCameraChessboardBase_t qt_meta_stringdata_CalibrateCameraChessboardBase = {
    {
QT_MOC_LITERAL(0, 0, 29) // "CalibrateCameraChessboardBase"

    },
    "CalibrateCameraChessboardBase"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CalibrateCameraChessboardBase[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void CalibrateCameraChessboardBase::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject CalibrateCameraChessboardBase::staticMetaObject = {
    { &CalibrateCameraBase::staticMetaObject, qt_meta_stringdata_CalibrateCameraChessboardBase.data,
      qt_meta_data_CalibrateCameraChessboardBase,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CalibrateCameraChessboardBase::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CalibrateCameraChessboardBase::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CalibrateCameraChessboardBase.stringdata0))
        return static_cast<void*>(const_cast< CalibrateCameraChessboardBase*>(this));
    return CalibrateCameraBase::qt_metacast(_clname);
}

int CalibrateCameraChessboardBase::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = CalibrateCameraBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_CalibrateCameraChessboardROS_t {
    QByteArrayData data[1];
    char stringdata0[29];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CalibrateCameraChessboardROS_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CalibrateCameraChessboardROS_t qt_meta_stringdata_CalibrateCameraChessboardROS = {
    {
QT_MOC_LITERAL(0, 0, 28) // "CalibrateCameraChessboardROS"

    },
    "CalibrateCameraChessboardROS"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CalibrateCameraChessboardROS[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void CalibrateCameraChessboardROS::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject CalibrateCameraChessboardROS::staticMetaObject = {
    { &CalibrateCameraChessboardBase::staticMetaObject, qt_meta_stringdata_CalibrateCameraChessboardROS.data,
      qt_meta_data_CalibrateCameraChessboardROS,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CalibrateCameraChessboardROS::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CalibrateCameraChessboardROS::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CalibrateCameraChessboardROS.stringdata0))
        return static_cast<void*>(const_cast< CalibrateCameraChessboardROS*>(this));
    return CalibrateCameraChessboardBase::qt_metacast(_clname);
}

int CalibrateCameraChessboardROS::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = CalibrateCameraChessboardBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_CalibrateCameraVelodyneChessboardBase_t {
    QByteArrayData data[12];
    char stringdata0[217];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CalibrateCameraVelodyneChessboardBase_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CalibrateCameraVelodyneChessboardBase_t qt_meta_stringdata_CalibrateCameraVelodyneChessboardBase = {
    {
QT_MOC_LITERAL(0, 0, 37), // "CalibrateCameraVelodyneChessb..."
QT_MOC_LITERAL(1, 38, 23), // "velodyneRefreshedSignal"
QT_MOC_LITERAL(2, 62, 0), // ""
QT_MOC_LITERAL(3, 63, 28), // "velodyneRefreshedErrorSignal"
QT_MOC_LITERAL(4, 92, 19), // "refreshVelodyneSlot"
QT_MOC_LITERAL(5, 112, 20), // "extractionResultSlot"
QT_MOC_LITERAL(6, 133, 36), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(7, 170, 10), // "extraction"
QT_MOC_LITERAL(8, 181, 7), // "cv::Mat"
QT_MOC_LITERAL(9, 189, 6), // "normal"
QT_MOC_LITERAL(10, 196, 2), // "id"
QT_MOC_LITERAL(11, 199, 17) // "projectPointsSlot"

    },
    "CalibrateCameraVelodyneChessboardBase\0"
    "velodyneRefreshedSignal\0\0"
    "velodyneRefreshedErrorSignal\0"
    "refreshVelodyneSlot\0extractionResultSlot\0"
    "pcl::PointCloud<pcl::PointXYZI>::Ptr\0"
    "extraction\0cv::Mat\0normal\0id\0"
    "projectPointsSlot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CalibrateCameraVelodyneChessboardBase[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,
       3,    0,   40,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   41,    2, 0x09 /* Protected */,
       5,    3,   42,    2, 0x0a /* Public */,
      11,    0,   49,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 6, 0x80000000 | 8, QMetaType::Int,    7,    9,   10,
    QMetaType::Void,

       0        // eod
};

void CalibrateCameraVelodyneChessboardBase::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        CalibrateCameraVelodyneChessboardBase *_t = static_cast<CalibrateCameraVelodyneChessboardBase *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->velodyneRefreshedSignal(); break;
        case 1: _t->velodyneRefreshedErrorSignal(); break;
        case 2: _t->refreshVelodyneSlot(); break;
        case 3: _t->extractionResultSlot((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 4: _t->projectPointsSlot(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (CalibrateCameraVelodyneChessboardBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrateCameraVelodyneChessboardBase::velodyneRefreshedSignal)) {
                *result = 0;
            }
        }
        {
            typedef void (CalibrateCameraVelodyneChessboardBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrateCameraVelodyneChessboardBase::velodyneRefreshedErrorSignal)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject CalibrateCameraVelodyneChessboardBase::staticMetaObject = {
    { &CalibrateCameraChessboardBase::staticMetaObject, qt_meta_stringdata_CalibrateCameraVelodyneChessboardBase.data,
      qt_meta_data_CalibrateCameraVelodyneChessboardBase,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CalibrateCameraVelodyneChessboardBase::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CalibrateCameraVelodyneChessboardBase::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CalibrateCameraVelodyneChessboardBase.stringdata0))
        return static_cast<void*>(const_cast< CalibrateCameraVelodyneChessboardBase*>(this));
    return CalibrateCameraChessboardBase::qt_metacast(_clname);
}

int CalibrateCameraVelodyneChessboardBase::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = CalibrateCameraChessboardBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void CalibrateCameraVelodyneChessboardBase::velodyneRefreshedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void CalibrateCameraVelodyneChessboardBase::velodyneRefreshedErrorSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}
struct qt_meta_stringdata_CalibrateCameraVelodyneChessboardROS_t {
    QByteArrayData data[1];
    char stringdata0[37];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CalibrateCameraVelodyneChessboardROS_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CalibrateCameraVelodyneChessboardROS_t qt_meta_stringdata_CalibrateCameraVelodyneChessboardROS = {
    {
QT_MOC_LITERAL(0, 0, 36) // "CalibrateCameraVelodyneChessb..."

    },
    "CalibrateCameraVelodyneChessboardROS"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CalibrateCameraVelodyneChessboardROS[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void CalibrateCameraVelodyneChessboardROS::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject CalibrateCameraVelodyneChessboardROS::staticMetaObject = {
    { &CalibrateCameraVelodyneChessboardBase::staticMetaObject, qt_meta_stringdata_CalibrateCameraVelodyneChessboardROS.data,
      qt_meta_data_CalibrateCameraVelodyneChessboardROS,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CalibrateCameraVelodyneChessboardROS::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CalibrateCameraVelodyneChessboardROS::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CalibrateCameraVelodyneChessboardROS.stringdata0))
        return static_cast<void*>(const_cast< CalibrateCameraVelodyneChessboardROS*>(this));
    return CalibrateCameraVelodyneChessboardBase::qt_metacast(_clname);
}

int CalibrateCameraVelodyneChessboardROS::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = CalibrateCameraVelodyneChessboardBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_CalibrateCameraLidarChessboardBase_t {
    QByteArrayData data[10];
    char stringdata0[170];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CalibrateCameraLidarChessboardBase_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CalibrateCameraLidarChessboardBase_t qt_meta_stringdata_CalibrateCameraLidarChessboardBase = {
    {
QT_MOC_LITERAL(0, 0, 34), // "CalibrateCameraLidarChessboar..."
QT_MOC_LITERAL(1, 35, 20), // "lidarRefreshedSignal"
QT_MOC_LITERAL(2, 56, 0), // ""
QT_MOC_LITERAL(3, 57, 25), // "lidarRefreshedErrorSignal"
QT_MOC_LITERAL(4, 83, 16), // "refreshLidarSlot"
QT_MOC_LITERAL(5, 100, 20), // "extractionResultSlot"
QT_MOC_LITERAL(6, 121, 16), // "QVector<QPointF>"
QT_MOC_LITERAL(7, 138, 10), // "extraction"
QT_MOC_LITERAL(8, 149, 2), // "id"
QT_MOC_LITERAL(9, 152, 17) // "projectPointsSlot"

    },
    "CalibrateCameraLidarChessboardBase\0"
    "lidarRefreshedSignal\0\0lidarRefreshedErrorSignal\0"
    "refreshLidarSlot\0extractionResultSlot\0"
    "QVector<QPointF>\0extraction\0id\0"
    "projectPointsSlot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CalibrateCameraLidarChessboardBase[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,
       3,    0,   40,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   41,    2, 0x09 /* Protected */,
       5,    2,   42,    2, 0x0a /* Public */,
       9,    0,   47,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 6, QMetaType::Int,    7,    8,
    QMetaType::Void,

       0        // eod
};

void CalibrateCameraLidarChessboardBase::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        CalibrateCameraLidarChessboardBase *_t = static_cast<CalibrateCameraLidarChessboardBase *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->lidarRefreshedSignal(); break;
        case 1: _t->lidarRefreshedErrorSignal(); break;
        case 2: _t->refreshLidarSlot(); break;
        case 3: _t->extractionResultSlot((*reinterpret_cast< QVector<QPointF>(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 4: _t->projectPointsSlot(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QVector<QPointF> >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (CalibrateCameraLidarChessboardBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrateCameraLidarChessboardBase::lidarRefreshedSignal)) {
                *result = 0;
            }
        }
        {
            typedef void (CalibrateCameraLidarChessboardBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CalibrateCameraLidarChessboardBase::lidarRefreshedErrorSignal)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject CalibrateCameraLidarChessboardBase::staticMetaObject = {
    { &CalibrateCameraChessboardBase::staticMetaObject, qt_meta_stringdata_CalibrateCameraLidarChessboardBase.data,
      qt_meta_data_CalibrateCameraLidarChessboardBase,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CalibrateCameraLidarChessboardBase::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CalibrateCameraLidarChessboardBase::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CalibrateCameraLidarChessboardBase.stringdata0))
        return static_cast<void*>(const_cast< CalibrateCameraLidarChessboardBase*>(this));
    return CalibrateCameraChessboardBase::qt_metacast(_clname);
}

int CalibrateCameraLidarChessboardBase::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = CalibrateCameraChessboardBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void CalibrateCameraLidarChessboardBase::lidarRefreshedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void CalibrateCameraLidarChessboardBase::lidarRefreshedErrorSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}
struct qt_meta_stringdata_CalibrateCameraLidarChessboardROS_t {
    QByteArrayData data[1];
    char stringdata0[34];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CalibrateCameraLidarChessboardROS_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CalibrateCameraLidarChessboardROS_t qt_meta_stringdata_CalibrateCameraLidarChessboardROS = {
    {
QT_MOC_LITERAL(0, 0, 33) // "CalibrateCameraLidarChessboar..."

    },
    "CalibrateCameraLidarChessboardROS"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CalibrateCameraLidarChessboardROS[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void CalibrateCameraLidarChessboardROS::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject CalibrateCameraLidarChessboardROS::staticMetaObject = {
    { &CalibrateCameraLidarChessboardBase::staticMetaObject, qt_meta_stringdata_CalibrateCameraLidarChessboardROS.data,
      qt_meta_data_CalibrateCameraLidarChessboardROS,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CalibrateCameraLidarChessboardROS::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CalibrateCameraLidarChessboardROS::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CalibrateCameraLidarChessboardROS.stringdata0))
        return static_cast<void*>(const_cast< CalibrateCameraLidarChessboardROS*>(this));
    return CalibrateCameraLidarChessboardBase::qt_metacast(_clname);
}

int CalibrateCameraLidarChessboardROS::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = CalibrateCameraLidarChessboardBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
QT_END_MOC_NAMESPACE
