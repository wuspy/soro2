#include "pitchrollview.h"

namespace Soro {

PitchRollView::PitchRollView()
{
    _pitch = 0;
    _roll = 0;
}

void PitchRollView::paint(QPainter *painter)
{
    painter->setBrush(QBrush(QColor("#ff0000")));
    painter->drawEllipse(QPoint(width() / 2, height() / 2), width() / 2, height() / 2);

    painter->resetTransform();
    painter->translate(width() / 2, height() / 2);
    painter->rotate(_roll);
    painter->setBrush(QBrush(QColor("#ff0000")));

    painter->drawChord(QRectF(0, height() / 2, width(), height() / 2), 30 * 16, 120 * 16);
}

void PitchRollView::setPitch(double pitch)
{
    _pitch = pitch;
    update();
}

double PitchRollView::getPitch() const
{
    return _pitch;
}

void PitchRollView::setRoll(double roll)
{
    _roll = roll;
    update();
}

double PitchRollView::getRoll() const
{
    return _roll;
}

} // namespace Soro
