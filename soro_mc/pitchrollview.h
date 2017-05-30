#ifndef PITCHROLLVIEW_H
#define PITCHROLLVIEW_H

#include <QQuickPaintedItem>
#include <QPainter>

namespace Soro {

class PitchRollView : public QQuickPaintedItem
{
    Q_OBJECT
    Q_PROPERTY(double pitch READ getPitch WRITE setPitch)
    Q_PROPERTY(double roll READ getRoll WRITE setRoll)

public:
    PitchRollView();
    void paint(QPainter *painter);

    double getPitch() const;
    void setPitch(double pitch);

    double getRoll() const;
    void setRoll(double roll);

private:
    double _pitch;
    double _roll;
};

} // namespace Soro

#endif // PITCHROLLVIEW_H
