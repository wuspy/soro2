#ifndef NAMEGEN_H
#define NAMEGEN_H

#include <QString>

namespace Soro {

/* Generates random Ubuntu-version-like names
 */
class NameGen
{
public:
    static QString generate(int adjectiveCount=1);

private:
    NameGen();
};

} // namespace Soro

#endif // NAMEGEN_H
