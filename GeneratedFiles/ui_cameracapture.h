/********************************************************************************
** Form generated from reading UI file 'cameracapture.ui'
**
** Created by: Qt User Interface Compiler version 5.9.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAMERACAPTURE_H
#define UI_CAMERACAPTURE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CameraCapture
{
public:
    QPushButton *openButton;
    QPushButton *closeButton;

    void setupUi(QWidget *CameraCapture)
    {
        if (CameraCapture->objectName().isEmpty())
            CameraCapture->setObjectName(QStringLiteral("CameraCapture"));
        CameraCapture->resize(390, 331);
        openButton = new QPushButton(CameraCapture);
        openButton->setObjectName(QStringLiteral("openButton"));
        openButton->setGeometry(QRect(50, 30, 75, 61));
        closeButton = new QPushButton(CameraCapture);
        closeButton->setObjectName(QStringLiteral("closeButton"));
        closeButton->setGeometry(QRect(210, 30, 75, 61));

        retranslateUi(CameraCapture);

        QMetaObject::connectSlotsByName(CameraCapture);
    } // setupUi

    void retranslateUi(QWidget *CameraCapture)
    {
        CameraCapture->setWindowTitle(QApplication::translate("CameraCapture", "CameraCapture", Q_NULLPTR));
        openButton->setText(QApplication::translate("CameraCapture", "\346\211\223\345\274\200\347\233\270\346\234\272", Q_NULLPTR));
        closeButton->setText(QApplication::translate("CameraCapture", "\345\205\263\351\227\255\347\233\270\346\234\272", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class CameraCapture: public Ui_CameraCapture {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAMERACAPTURE_H
