#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QQmlContext>
#include <qqml.h>
#include <QQuickItem>

#include <backend.h>


int main(int argc, char *argv[])
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

    QGuiApplication app(argc, argv);

    QQuickView view; // Qt default at project generation
    view.setMaximumSize(QSize(1920,1080));
    view.setMinimumSize(QSize(1920,1080));
    view.engine()->addImportPath("qrc:/qml/imports"); // mendatory to include a custom imports for QML, such as png's and so on.
    view.setSource(QUrl("qrc:/qml/main.qml")); // used to define the source QML file. (please ignore the main.qml)
    if (!view.errors().isEmpty())
        return -1;
    QQuickItem *item = view.rootObject(); // generates and item object and allows backend to modify it

    BackEnd backend(argc, argv, item); // creates a backend object, with argc, argv, and the item property
    view.rootContext()->setContextProperty("backend", &backend); // connects backend to qml, refered in QML as the string "backend", references the object name &backend.


    view.show(); // default QT generation
    return app.exec(); // default QT generation.
}
