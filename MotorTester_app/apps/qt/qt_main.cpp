#include "mks/main_window.h"
#include "mks/DarkStyle.h"

#include <QApplication>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    DarkStyle::apply();
    MainWindow w;
    w.show();
    return app.exec();
}