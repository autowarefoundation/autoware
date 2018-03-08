/****************************************************************************
** Meta object code from reading C++ file 'selectionwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "selectionwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QVector>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'selectionwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PlaneExtractor_t {
    QByteArrayData data[13];
    char stringdata0[171];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PlaneExtractor_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PlaneExtractor_t qt_meta_stringdata_PlaneExtractor = {
    {
QT_MOC_LITERAL(0, 0, 14), // "PlaneExtractor"
QT_MOC_LITERAL(1, 15, 22), // "extractionResultSignal"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 36), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(4, 76, 10), // "extraction"
QT_MOC_LITERAL(5, 87, 7), // "cv::Mat"
QT_MOC_LITERAL(6, 95, 6), // "normal"
QT_MOC_LITERAL(7, 102, 2), // "id"
QT_MOC_LITERAL(8, 105, 17), // "mousePositionSlot"
QT_MOC_LITERAL(9, 123, 12), // "QMouseEvent*"
QT_MOC_LITERAL(10, 136, 5), // "event"
QT_MOC_LITERAL(11, 142, 17), // "CAMERAPARAMETERS*"
QT_MOC_LITERAL(12, 160, 10) // "parameters"

    },
    "PlaneExtractor\0extractionResultSignal\0"
    "\0pcl::PointCloud<pcl::PointXYZI>::Ptr\0"
    "extraction\0cv::Mat\0normal\0id\0"
    "mousePositionSlot\0QMouseEvent*\0event\0"
    "CAMERAPARAMETERS*\0parameters"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PlaneExtractor[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,   24,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    2,   31,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5, QMetaType::Int,    4,    6,    7,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 9, 0x80000000 | 11,   10,   12,

       0        // eod
};

void PlaneExtractor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PlaneExtractor *_t = static_cast<PlaneExtractor *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->extractionResultSignal((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 1: _t->mousePositionSlot((*reinterpret_cast< QMouseEvent*(*)>(_a[1])),(*reinterpret_cast< CAMERAPARAMETERS*(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (PlaneExtractor::*_t)(pcl::PointCloud<pcl::PointXYZI>::Ptr , cv::Mat , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PlaneExtractor::extractionResultSignal)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject PlaneExtractor::staticMetaObject = {
    { &GLViewer::staticMetaObject, qt_meta_stringdata_PlaneExtractor.data,
      qt_meta_data_PlaneExtractor,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PlaneExtractor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PlaneExtractor::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PlaneExtractor.stringdata0))
        return static_cast<void*>(const_cast< PlaneExtractor*>(this));
    return GLViewer::qt_metacast(_clname);
}

int PlaneExtractor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GLViewer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void PlaneExtractor::extractionResultSignal(pcl::PointCloud<pcl::PointXYZI>::Ptr _t1, cv::Mat _t2, int _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
struct qt_meta_stringdata_PointsExtractor_t {
    QByteArrayData data[6];
    char stringdata0[71];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PointsExtractor_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PointsExtractor_t qt_meta_stringdata_PointsExtractor = {
    {
QT_MOC_LITERAL(0, 0, 15), // "PointsExtractor"
QT_MOC_LITERAL(1, 16, 22), // "extractionResultSignal"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 16), // "QVector<QPointF>"
QT_MOC_LITERAL(4, 57, 10), // "extraction"
QT_MOC_LITERAL(5, 68, 2) // "id"

    },
    "PointsExtractor\0extractionResultSignal\0"
    "\0QVector<QPointF>\0extraction\0id"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PointsExtractor[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   19,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int,    4,    5,

       0        // eod
};

void PointsExtractor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PointsExtractor *_t = static_cast<PointsExtractor *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->extractionResultSignal((*reinterpret_cast< QVector<QPointF>(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
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
            typedef void (PointsExtractor::*_t)(QVector<QPointF> , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointsExtractor::extractionResultSignal)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject PointsExtractor::staticMetaObject = {
    { &QLabel::staticMetaObject, qt_meta_stringdata_PointsExtractor.data,
      qt_meta_data_PointsExtractor,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PointsExtractor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PointsExtractor::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PointsExtractor.stringdata0))
        return static_cast<void*>(const_cast< PointsExtractor*>(this));
    return QLabel::qt_metacast(_clname);
}

int PointsExtractor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void PointsExtractor::extractionResultSignal(QVector<QPointF> _t1, int _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
