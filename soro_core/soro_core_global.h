#ifndef SORO_CORE_GLOBAL_H
#define SORO_CORE_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(SORO_CORE_LIBRARY)
#  define SORO_CORE_EXPORT Q_DECL_EXPORT
#else
#  define SORO_CORE_EXPORT Q_DECL_IMPORT
#endif

#endif // SORO_CORE_GLOBAL_H
