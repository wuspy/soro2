/****************************************************************************
** Meta object code from reading C++ file 'gamepadcontroller.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.8.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../soro_mc/gamepadcontroller.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'gamepadcontroller.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.8.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Soro__GamepadController_t {
    QByteArrayData data[14];
    char stringdata0[164];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Soro__GamepadController_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Soro__GamepadController_t qt_meta_stringdata_Soro__GamepadController = {
    {
QT_MOC_LITERAL(0, 0, 23), // "Soro::GamepadController"
QT_MOC_LITERAL(1, 24, 13), // "buttonPressed"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 24), // "SDL_GameControllerButton"
QT_MOC_LITERAL(4, 64, 6), // "button"
QT_MOC_LITERAL(5, 71, 9), // "isPressed"
QT_MOC_LITERAL(6, 81, 11), // "axisChanged"
QT_MOC_LITERAL(7, 93, 22), // "SDL_GameControllerAxis"
QT_MOC_LITERAL(8, 116, 4), // "axis"
QT_MOC_LITERAL(9, 121, 5), // "value"
QT_MOC_LITERAL(10, 127, 14), // "gamepadChanged"
QT_MOC_LITERAL(11, 142, 11), // "isConnected"
QT_MOC_LITERAL(12, 154, 4), // "name"
QT_MOC_LITERAL(13, 159, 4) // "poll"

    },
    "Soro::GamepadController\0buttonPressed\0"
    "\0SDL_GameControllerButton\0button\0"
    "isPressed\0axisChanged\0SDL_GameControllerAxis\0"
    "axis\0value\0gamepadChanged\0isConnected\0"
    "name\0poll"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Soro__GamepadController[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   34,    2, 0x06 /* Public */,
       6,    2,   39,    2, 0x06 /* Public */,
      10,    2,   44,    2, 0x06 /* Public */,
      13,    0,   49,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::Bool,    4,    5,
    QMetaType::Void, 0x80000000 | 7, QMetaType::Float,    8,    9,
    QMetaType::Void, QMetaType::Bool, QMetaType::QString,   11,   12,
    QMetaType::Void,

       0        // eod
};

void Soro::GamepadController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        GamepadController *_t = static_cast<GamepadController *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->buttonPressed((*reinterpret_cast< SDL_GameControllerButton(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 1: _t->axisChanged((*reinterpret_cast< SDL_GameControllerAxis(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 2: _t->gamepadChanged((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 3: _t->poll(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (GamepadController::*_t)(SDL_GameControllerButton , bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GamepadController::buttonPressed)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (GamepadController::*_t)(SDL_GameControllerAxis , float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GamepadController::axisChanged)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (GamepadController::*_t)(bool , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GamepadController::gamepadChanged)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (GamepadController::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GamepadController::poll)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject Soro::GamepadController::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Soro__GamepadController.data,
      qt_meta_data_Soro__GamepadController,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Soro::GamepadController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Soro::GamepadController::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Soro__GamepadController.stringdata0))
        return static_cast<void*>(const_cast< GamepadController*>(this));
    return QObject::qt_metacast(_clname);
}

int Soro::GamepadController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void Soro::GamepadController::buttonPressed(SDL_GameControllerButton _t1, bool _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Soro::GamepadController::axisChanged(SDL_GameControllerAxis _t1, float _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Soro::GamepadController::gamepadChanged(bool _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Soro::GamepadController::poll()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
