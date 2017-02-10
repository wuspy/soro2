#include <QString>
#include <QtTest>

class Tests : public QObject
{
    Q_OBJECT

public:
    Tests();

private Q_SLOTS:
    void testExample();
};

Tests::Tests()
{
}

void Tests::testExample()
{
    QString string1 = "Sooner Rover is awesome!";
    QString string2 = "Sooner Rover is awesome!";

    QVERIFY2(string1 == string2, "Two identical QStrings aren't equal to each other. Your computer is probably  broken.");
}

QTEST_GUILESS_MAIN(Tests)

#include "tst_tests.moc"
