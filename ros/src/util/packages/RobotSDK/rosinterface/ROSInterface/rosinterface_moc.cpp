/****************************************************************************
** Meta object code from reading C++ file 'rosinterface.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../include/rosinterface/rosinterface.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'rosinterface.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ROSInterfaceBase_t {
    QByteArrayData data[1];
    char stringdata0[17];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ROSInterfaceBase_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ROSInterfaceBase_t qt_meta_stringdata_ROSInterfaceBase = {
    {
QT_MOC_LITERAL(0, 0, 16) // "ROSInterfaceBase"

    },
    "ROSInterfaceBase"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ROSInterfaceBase[] = {

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

void ROSInterfaceBase::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject ROSInterfaceBase::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_ROSInterfaceBase.data,
      qt_meta_data_ROSInterfaceBase,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ROSInterfaceBase::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ROSInterfaceBase::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ROSInterfaceBase.stringdata0))
        return static_cast<void*>(const_cast< ROSInterfaceBase*>(this));
    return QObject::qt_metacast(_clname);
}

int ROSInterfaceBase::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_ROSSubBase_t {
    QByteArrayData data[10];
    char stringdata0[161];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ROSSubBase_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ROSSubBase_t qt_meta_stringdata_ROSSubBase = {
    {
QT_MOC_LITERAL(0, 0, 10), // "ROSSubBase"
QT_MOC_LITERAL(1, 11, 20), // "receiveMessageSignal"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 18), // "startReceiveSignal"
QT_MOC_LITERAL(4, 52, 17), // "stopReceiveSignal"
QT_MOC_LITERAL(5, 70, 24), // "resetQueryIntervalSignal"
QT_MOC_LITERAL(6, 95, 13), // "QueryInterval"
QT_MOC_LITERAL(7, 109, 16), // "startReceiveSlot"
QT_MOC_LITERAL(8, 126, 15), // "stopReceiveSlot"
QT_MOC_LITERAL(9, 142, 18) // "receiveMessageSlot"

    },
    "ROSSubBase\0receiveMessageSignal\0\0"
    "startReceiveSignal\0stopReceiveSignal\0"
    "resetQueryIntervalSignal\0QueryInterval\0"
    "startReceiveSlot\0stopReceiveSlot\0"
    "receiveMessageSlot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ROSSubBase[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,
       3,    0,   50,    2, 0x06 /* Public */,
       4,    0,   51,    2, 0x06 /* Public */,
       5,    1,   52,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   55,    2, 0x0a /* Public */,
       8,    0,   56,    2, 0x0a /* Public */,
       9,    0,   57,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    6,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ROSSubBase::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ROSSubBase *_t = static_cast<ROSSubBase *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->receiveMessageSignal(); break;
        case 1: _t->startReceiveSignal(); break;
        case 2: _t->stopReceiveSignal(); break;
        case 3: _t->resetQueryIntervalSignal((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->startReceiveSlot(); break;
        case 5: _t->stopReceiveSlot(); break;
        case 6: _t->receiveMessageSlot(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ROSSubBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSSubBase::receiveMessageSignal)) {
                *result = 0;
            }
        }
        {
            typedef void (ROSSubBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSSubBase::startReceiveSignal)) {
                *result = 1;
            }
        }
        {
            typedef void (ROSSubBase::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSSubBase::stopReceiveSignal)) {
                *result = 2;
            }
        }
        {
            typedef void (ROSSubBase::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSSubBase::resetQueryIntervalSignal)) {
                *result = 3;
            }
        }
    }
}

const QMetaObject ROSSubBase::staticMetaObject = {
    { &ROSInterfaceBase::staticMetaObject, qt_meta_stringdata_ROSSubBase.data,
      qt_meta_data_ROSSubBase,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ROSSubBase::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ROSSubBase::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ROSSubBase.stringdata0))
        return static_cast<void*>(const_cast< ROSSubBase*>(this));
    return ROSInterfaceBase::qt_metacast(_clname);
}

int ROSSubBase::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ROSInterfaceBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void ROSSubBase::receiveMessageSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void ROSSubBase::startReceiveSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void ROSSubBase::stopReceiveSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void ROSSubBase::resetQueryIntervalSignal(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
struct qt_meta_stringdata_ROSTFPub_t {
    QByteArrayData data[1];
    char stringdata0[9];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ROSTFPub_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ROSTFPub_t qt_meta_stringdata_ROSTFPub = {
    {
QT_MOC_LITERAL(0, 0, 8) // "ROSTFPub"

    },
    "ROSTFPub"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ROSTFPub[] = {

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

void ROSTFPub::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject ROSTFPub::staticMetaObject = {
    { &ROSInterfaceBase::staticMetaObject, qt_meta_stringdata_ROSTFPub.data,
      qt_meta_data_ROSTFPub,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ROSTFPub::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ROSTFPub::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ROSTFPub.stringdata0))
        return static_cast<void*>(const_cast< ROSTFPub*>(this));
    return ROSInterfaceBase::qt_metacast(_clname);
}

int ROSTFPub::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ROSInterfaceBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_ROSTFSub_t {
    QByteArrayData data[10];
    char stringdata0[149];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ROSTFSub_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ROSTFSub_t qt_meta_stringdata_ROSTFSub = {
    {
QT_MOC_LITERAL(0, 0, 8), // "ROSTFSub"
QT_MOC_LITERAL(1, 9, 15), // "receiveTFSignal"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 18), // "startReceiveSignal"
QT_MOC_LITERAL(4, 45, 17), // "stopReceiveSignal"
QT_MOC_LITERAL(5, 63, 24), // "resetQueryIntervalSignal"
QT_MOC_LITERAL(6, 88, 13), // "QueryInterval"
QT_MOC_LITERAL(7, 102, 16), // "startReceiveSlot"
QT_MOC_LITERAL(8, 119, 15), // "stopReceiveSlot"
QT_MOC_LITERAL(9, 135, 13) // "receiveTFSlot"

    },
    "ROSTFSub\0receiveTFSignal\0\0startReceiveSignal\0"
    "stopReceiveSignal\0resetQueryIntervalSignal\0"
    "QueryInterval\0startReceiveSlot\0"
    "stopReceiveSlot\0receiveTFSlot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ROSTFSub[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,
       3,    0,   50,    2, 0x06 /* Public */,
       4,    0,   51,    2, 0x06 /* Public */,
       5,    1,   52,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   55,    2, 0x0a /* Public */,
       8,    0,   56,    2, 0x0a /* Public */,
       9,    0,   57,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    6,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ROSTFSub::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ROSTFSub *_t = static_cast<ROSTFSub *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->receiveTFSignal(); break;
        case 1: _t->startReceiveSignal(); break;
        case 2: _t->stopReceiveSignal(); break;
        case 3: _t->resetQueryIntervalSignal((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->startReceiveSlot(); break;
        case 5: _t->stopReceiveSlot(); break;
        case 6: _t->receiveTFSlot(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ROSTFSub::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSTFSub::receiveTFSignal)) {
                *result = 0;
            }
        }
        {
            typedef void (ROSTFSub::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSTFSub::startReceiveSignal)) {
                *result = 1;
            }
        }
        {
            typedef void (ROSTFSub::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSTFSub::stopReceiveSignal)) {
                *result = 2;
            }
        }
        {
            typedef void (ROSTFSub::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSTFSub::resetQueryIntervalSignal)) {
                *result = 3;
            }
        }
    }
}

const QMetaObject ROSTFSub::staticMetaObject = {
    { &ROSInterfaceBase::staticMetaObject, qt_meta_stringdata_ROSTFSub.data,
      qt_meta_data_ROSTFSub,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ROSTFSub::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ROSTFSub::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ROSTFSub.stringdata0))
        return static_cast<void*>(const_cast< ROSTFSub*>(this));
    return ROSInterfaceBase::qt_metacast(_clname);
}

int ROSTFSub::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ROSInterfaceBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void ROSTFSub::receiveTFSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void ROSTFSub::startReceiveSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void ROSTFSub::stopReceiveSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void ROSTFSub::resetQueryIntervalSignal(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
