/*
 * Copyright 2017 The University of Oklahoma.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "qquickgstreamersurface.h"
#include <QPainter>
#include <Qt5GStreamer/QGlib/Connect>
#include <Qt5GStreamer/QGlib/Signal>
#include <Qt5GStreamer/QGst/ElementFactory>
#include <QGLContext>

namespace Soro {

QQuickGStreamerSurface::QQuickGStreamerSurface()
{
    _useHardwareRendering = false;
}

QQuickGStreamerSurface::~QQuickGStreamerSurface()
{
    if (!_sink.isNull())
    {
        _sink->setState(QGst::StateNull);
        _sink.clear();
    }
}

void QQuickGStreamerSurface::paint(QPainter *painter)
{
    if (!_sink.isNull())
    {
        // This line will show a syntax error if QT_NO_KEYWORDS is not defined, but it will still compile
        QGlib::emit<void>(_sink, "paint", (void *) painter, (qreal)0, (qreal)0, (qreal)width(), (qreal)height());
    }
}

bool QQuickGStreamerSurface::getEnableHardwareRendering() const
{
    return _useHardwareRendering;
}

void QQuickGStreamerSurface::setEnableHardwareRendering(bool enableHardwareRendering)
{
    // Cannot change this property after sink has already been created
    if (_sink.isNull()) {
        _useHardwareRendering = enableHardwareRendering;
    }
}

QGst::ElementPtr QQuickGStreamerSurface::videoSink()
{
    if (_sink.isNull())
    {
        if (_useHardwareRendering)
        {
            _sink = QGst::ElementFactory::make("qt5glvideosink");

            if (!_sink.isNull())
            {
                _sink->setProperty("glcontext", (void*) QGLContext::currentContext());

                if (_sink->setState(QGst::StateReady) != QGst::StateChangeSuccess)
                {
                    _sink.clear();
                }
            }
        }
        else
        {
            _sink = QGst::ElementFactory::make("qt5videosink");
        }
    }

    if (!_sink.isNull())
    {
        QGlib::connect(_sink, "update", this, &QQuickGStreamerSurface::onUpdate);
    }

    return _sink;
}

void QQuickGStreamerSurface::onUpdate()
{
    update();
}

} // namespace Soro
