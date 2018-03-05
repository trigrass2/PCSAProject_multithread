#include "displaygui.h"
#include <QtWidgets/QApplication> 
int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	DisplayGUI w;
	w.show();	
	w.initialization();
	return a.exec();
}
