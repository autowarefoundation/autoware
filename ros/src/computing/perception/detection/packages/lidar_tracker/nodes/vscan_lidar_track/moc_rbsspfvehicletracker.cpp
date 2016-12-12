/****************************************************************************
** Meta object code from reading C++ file 'rbsspfvehicletracker.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.2.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "rbsspfvehicletracker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'rbsspfvehicletracker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.2.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_RBSSPFVehicleTrackerInstance_t {
    QByteArrayData data[16];
    char stringdata[253];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    offsetof(qt_meta_stringdata_RBSSPFVehicleTrackerInstance_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData) \
    )
static const qt_meta_stringdata_RBSSPFVehicleTrackerInstance_t qt_meta_stringdata_RBSSPFVehicleTrackerInstance = {
    {
QT_MOC_LITERAL(0, 0, 28),
QT_MOC_LITERAL(1, 29, 26),
QT_MOC_LITERAL(2, 56, 0),
QT_MOC_LITERAL(3, 57, 25),
QT_MOC_LITERAL(4, 83, 9),
QT_MOC_LITERAL(5, 93, 23),
QT_MOC_LITERAL(6, 117, 13),
QT_MOC_LITERAL(7, 131, 18),
QT_MOC_LITERAL(8, 150, 7),
QT_MOC_LITERAL(9, 158, 13),
QT_MOC_LITERAL(10, 172, 9),
QT_MOC_LITERAL(11, 182, 5),
QT_MOC_LITERAL(12, 188, 8),
QT_MOC_LITERAL(13, 197, 17),
QT_MOC_LITERAL(14, 215, 23),
QT_MOC_LITERAL(15, 239, 12)
    },
    "RBSSPFVehicleTrackerInstance\0"
    "signalCheckInitStateFinish\0\0"
    "signalUpdateTrackerFinish\0vehicleID\0"
    "TrackerResultContainer*\0trackerResult\0"
    "slotCheckInitState\0initNum\0VehicleState*\0"
    "initState\0bool*\0initFlag\0slotUpdateTracker\0"
    "QMap<int,VehicleState>*\0initStateMap\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RBSSPFVehicleTrackerInstance[] = {

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
       1,    0,   34,    2, 0x06,
       3,    2,   35,    2, 0x06,

 // slots: name, argc, parameters, tag, flags
       7,    3,   40,    2, 0x0a,
      13,    1,   47,    2, 0x0a,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, 0x80000000 | 5,    4,    6,

 // slots: parameters
    QMetaType::Void, QMetaType::Int, 0x80000000 | 9, 0x80000000 | 11,    8,   10,   12,
    QMetaType::Void, 0x80000000 | 14,   15,

       0        // eod
};

void RBSSPFVehicleTrackerInstance::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RBSSPFVehicleTrackerInstance *_t = static_cast<RBSSPFVehicleTrackerInstance *>(_o);
        switch (_id) {
        case 0: _t->signalCheckInitStateFinish(); break;
        case 1: _t->signalUpdateTrackerFinish((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< TrackerResultContainer*(*)>(_a[2]))); break;
        case 2: _t->slotCheckInitState((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< VehicleState*(*)>(_a[2])),(*reinterpret_cast< bool*(*)>(_a[3]))); break;
        case 3: _t->slotUpdateTracker((*reinterpret_cast< QMap<int,VehicleState>*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (RBSSPFVehicleTrackerInstance::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RBSSPFVehicleTrackerInstance::signalCheckInitStateFinish)) {
                *result = 0;
            }
        }
        {
            typedef void (RBSSPFVehicleTrackerInstance::*_t)(int , TrackerResultContainer * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RBSSPFVehicleTrackerInstance::signalUpdateTrackerFinish)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject RBSSPFVehicleTrackerInstance::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_RBSSPFVehicleTrackerInstance.data,
      qt_meta_data_RBSSPFVehicleTrackerInstance,  qt_static_metacall, 0, 0}
};


const QMetaObject *RBSSPFVehicleTrackerInstance::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RBSSPFVehicleTrackerInstance::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_RBSSPFVehicleTrackerInstance.stringdata))
        return static_cast<void*>(const_cast< RBSSPFVehicleTrackerInstance*>(this));
    return QObject::qt_metacast(_clname);
}

int RBSSPFVehicleTrackerInstance::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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
void RBSSPFVehicleTrackerInstance::signalCheckInitStateFinish()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void RBSSPFVehicleTrackerInstance::signalUpdateTrackerFinish(int _t1, TrackerResultContainer * _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
struct qt_meta_stringdata_RBSSPFVehicleTracker_t {
    QByteArrayData data[21];
    char stringdata[336];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    offsetof(qt_meta_stringdata_RBSSPFVehicleTracker_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData) \
    )
static const qt_meta_stringdata_RBSSPFVehicleTracker_t qt_meta_stringdata_RBSSPFVehicleTracker = {
    {
QT_MOC_LITERAL(0, 0, 20),
QT_MOC_LITERAL(1, 21, 20),
QT_MOC_LITERAL(2, 42, 0),
QT_MOC_LITERAL(3, 43, 7),
QT_MOC_LITERAL(4, 51, 13),
QT_MOC_LITERAL(5, 65, 9),
QT_MOC_LITERAL(6, 75, 5),
QT_MOC_LITERAL(7, 81, 8),
QT_MOC_LITERAL(8, 90, 19),
QT_MOC_LITERAL(9, 110, 23),
QT_MOC_LITERAL(10, 134, 12),
QT_MOC_LITERAL(11, 147, 25),
QT_MOC_LITERAL(12, 173, 9),
QT_MOC_LITERAL(13, 183, 4),
QT_MOC_LITERAL(14, 188, 32),
QT_MOC_LITERAL(15, 221, 16),
QT_MOC_LITERAL(16, 238, 24),
QT_MOC_LITERAL(17, 263, 23),
QT_MOC_LITERAL(18, 287, 9),
QT_MOC_LITERAL(19, 297, 23),
QT_MOC_LITERAL(20, 321, 13)
    },
    "RBSSPFVehicleTracker\0signalCheckInitState\0"
    "\0initNum\0VehicleState*\0initState\0bool*\0"
    "initFlag\0signalUpdateTracker\0"
    "QMap<int,VehicleState>*\0initStateMap\0"
    "signalUpdateTrackerFinish\0LaserScan\0"
    "scan\0QMap<int,TrackerResultContainer>\0"
    "trackerresultmap\0slotCheckInitStateFinish\0"
    "slotUpdateTrackerFinish\0vehicleID\0"
    "TrackerResultContainer*\0trackerResult\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RBSSPFVehicleTracker[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,   39,    2, 0x06,
       8,    1,   46,    2, 0x06,
      11,    2,   49,    2, 0x06,

 // slots: name, argc, parameters, tag, flags
      16,    0,   54,    2, 0x0a,
      17,    2,   55,    2, 0x0a,

 // signals: parameters
    QMetaType::Void, QMetaType::Int, 0x80000000 | 4, 0x80000000 | 6,    3,    5,    7,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, 0x80000000 | 12, 0x80000000 | 14,   13,   15,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, 0x80000000 | 19,   18,   20,

       0        // eod
};

void RBSSPFVehicleTracker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RBSSPFVehicleTracker *_t = static_cast<RBSSPFVehicleTracker *>(_o);
        switch (_id) {
        case 0: _t->signalCheckInitState((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< VehicleState*(*)>(_a[2])),(*reinterpret_cast< bool*(*)>(_a[3]))); break;
        case 1: _t->signalUpdateTracker((*reinterpret_cast< QMap<int,VehicleState>*(*)>(_a[1]))); break;
        case 2: _t->signalUpdateTrackerFinish((*reinterpret_cast< LaserScan(*)>(_a[1])),(*reinterpret_cast< QMap<int,TrackerResultContainer>(*)>(_a[2]))); break;
        case 3: _t->slotCheckInitStateFinish(); break;
        case 4: _t->slotUpdateTrackerFinish((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< TrackerResultContainer*(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (RBSSPFVehicleTracker::*_t)(int , VehicleState * , bool * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RBSSPFVehicleTracker::signalCheckInitState)) {
                *result = 0;
            }
        }
        {
            typedef void (RBSSPFVehicleTracker::*_t)(QMap<int,VehicleState> * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RBSSPFVehicleTracker::signalUpdateTracker)) {
                *result = 1;
            }
        }
        {
            typedef void (RBSSPFVehicleTracker::*_t)(LaserScan , QMap<int,TrackerResultContainer> );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RBSSPFVehicleTracker::signalUpdateTrackerFinish)) {
                *result = 2;
            }
        }
    }
}

const QMetaObject RBSSPFVehicleTracker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_RBSSPFVehicleTracker.data,
      qt_meta_data_RBSSPFVehicleTracker,  qt_static_metacall, 0, 0}
};


const QMetaObject *RBSSPFVehicleTracker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RBSSPFVehicleTracker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_RBSSPFVehicleTracker.stringdata))
        return static_cast<void*>(const_cast< RBSSPFVehicleTracker*>(this));
    return QObject::qt_metacast(_clname);
}

int RBSSPFVehicleTracker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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
void RBSSPFVehicleTracker::signalCheckInitState(int _t1, VehicleState * _t2, bool * _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void RBSSPFVehicleTracker::signalUpdateTracker(QMap<int,VehicleState> * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void RBSSPFVehicleTracker::signalUpdateTrackerFinish(LaserScan _t1, QMap<int,TrackerResultContainer> _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
