#include "render2framebuffer.h"
#include <QtWidgets/QApplication>

int test_render2framebuffer(int argc, char **argv)
{
	QApplication app(argc, argv);

	QImage targetImage = renderOffScreen();
    targetImage.save("screen.png", "PNG");

	QLabel label;
	label.setPixmap(QPixmap::fromImage(targetImage));
	label.show();
	return app.exec();
}