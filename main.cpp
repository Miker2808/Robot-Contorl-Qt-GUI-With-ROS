#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QQmlContext>
#include <qqml.h>
#include <QQuickItem>

#include <backend.h>

// This is the main cpp, it is consisted of some importent notes related to ROS integration
// Some lines are mendatory in every application.
// Note the main parameters, which are mendatory solely by ROS!

int main(int argc, char *argv[])
{

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling); // Qt Default at project generation

    QGuiApplication app(argc, argv); // QT Default at project generation, but includes the argc, argv parameters!


    QQuickView view; // Qt default at project generation
    view.engine()->addImportPath("qrc:/qml/imports"); // mendatory to include a custom imports for QML, such as png's and so on.
    view.setSource(QUrl("qrc:/qml/SixWheelBotQGUI.qml")); // used to define the source QML file. (please ignore the main.qml)
    if (!view.errors().isEmpty())
        return -1;
    QQuickItem *item = view.rootObject(); // generates and item object and allows backend to modify it

    BackEnd backend(argc, argv, item); // creates a backend object, with argc, argv, and the item property
    view.rootContext()->setContextProperty("backend", &backend); // connects backend to qml, refered in QML as the string "backend", references the object name &backend.


    view.show(); // default QT generation
    return app.exec(); // default QT generation.
}
