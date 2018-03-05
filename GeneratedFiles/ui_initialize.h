/********************************************************************************
** Form generated from reading UI file 'initialize.ui'
**
** Created by: Qt User Interface Compiler version 5.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_INITIALIZE_H
#define UI_INITIALIZE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Initialize
{
public:
    QLabel *label;
    QLabel *label_2;

    void setupUi(QWidget *Initialize)
    {
        if (Initialize->objectName().isEmpty())
            Initialize->setObjectName(QStringLiteral("Initialize"));
        Initialize->resize(403, 303);
        label = new QLabel(Initialize);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(130, 50, 141, 141));
        label->setPixmap(QPixmap(QString::fromUtf8(":/\346\226\260\345\211\215\347\274\200/gnome_clocks_128px_1174945_easyicon.net.png")));
        label_2 = new QLabel(Initialize);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(100, 200, 261, 51));
        QFont font;
        font.setFamily(QString::fromUtf8("\346\226\260\345\256\213\344\275\223"));
        font.setPointSize(12);
        label_2->setFont(font);

        retranslateUi(Initialize);

        QMetaObject::connectSlotsByName(Initialize);
    } // setupUi

    void retranslateUi(QWidget *Initialize)
    {
        Initialize->setWindowTitle(QApplication::translate("Initialize", "Initialize", Q_NULLPTR));
        label->setText(QString());
        label_2->setText(QApplication::translate("Initialize", "\350\256\276\345\244\207\345\210\235\345\247\213\345\214\226\345\220\257\345\212\250\357\274\214\350\257\267\347\255\211\345\276\205...", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Initialize: public Ui_Initialize {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_INITIALIZE_H
