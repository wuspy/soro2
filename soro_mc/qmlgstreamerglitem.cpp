/*
 * Copyright 2017 Jacob Jordan <doublejinitials@ou.edu>
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

#include "qmlgstreamerglitem.h"
#include "soro_core/logger.h"

#include <QSGFlatColorMaterial>
#include <QSGGeometryNode>
#include <Qt5GStreamer/QGst/ElementFactory>
#include <Qt5GStreamer/QGlib/Signal>
#include <Qt5GStreamer/QGlib/Connect>

#define LogTag "QmlGStreamerGlItem"

namespace Soro {

QmlGStreamerGlItem::QmlGStreamerGlItem(QQuickItem *parent) : QQuickItem(parent)
{
    setFlag(QQuickItem::ItemHasContents, true);
    _surfaceDirty = true;
}

QmlGStreamerGlItem::~QmlGStreamerGlItem()
{
    if (!_sink.isNull())
    {
        _sink->setState(QGst::StateNull);
        _sink.clear();
    }
}

QSGNode* QmlGStreamerGlItem::updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *data)
{
    Q_UNUSED(data)

    QRectF r = boundingRect();
    QSGNode *newNode = 0;

    if (_surfaceDirty) {
        delete oldNode;
        oldNode = 0;
        _surfaceDirty = false;
    }

    if (_sink.isNull())
    {
        if (!oldNode)
        {
            QSGFlatColorMaterial *material = new QSGFlatColorMaterial;
            material->setColor(Qt::black);

            QSGGeometryNode *node = new QSGGeometryNode;
            node->setMaterial(material);
            node->setFlag(QSGNode::OwnsMaterial);
            node->setFlag(QSGNode::OwnsGeometry);

            newNode = node;
            _targetArea = QRectF(); //force geometry to be set
        }
        else
        {
            newNode = oldNode;
        }

        if (r != _targetArea)
        {
            QSGGeometry *geometry = new QSGGeometry(QSGGeometry::defaultAttributes_Point2D(), 4);
            geometry->vertexDataAsPoint2D()[0].set(r.x(), r.y());
            geometry->vertexDataAsPoint2D()[1].set(r.x(), r.height());
            geometry->vertexDataAsPoint2D()[2].set(r.width(), r.y());
            geometry->vertexDataAsPoint2D()[3].set(r.width(), r.height());

            QSGGeometryNode *node = static_cast<QSGGeometryNode*>(newNode);
            node->setGeometry(geometry);

            _targetArea = r;
        }
    }
    else
    {
        newNode = (QSGNode*) QGlib::emit<void*>(_sink,
                "update-node", (void*)oldNode,
                r.x(), r.y(), r.width(), r.height());
    }

    return newNode;
}

QGst::ElementPtr QmlGStreamerGlItem::videoSink()
{
    if (_sink.isNull())
    {
        _sink = QGst::ElementFactory::make("qtquick2videosink");
    }

    if (!_sink.isNull())
    {
        QGlib::connect(_sink, "update", this, &QmlGStreamerGlItem::onUpdate);
    }
    else
    {
        LOG_E(LogTag, "Failed to create qtquick2videosink");
    }

    return _sink;
}

void QmlGStreamerGlItem::onUpdate()
{
    update();
}

} // namespace Soro
