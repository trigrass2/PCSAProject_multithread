/********************************************************************************
** Form generated from reading UI file 'motioncontrol.ui'
**
** Created by: Qt User Interface Compiler version 5.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MOTIONCONTROL_H
#define UI_MOTIONCONTROL_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MotionControl
{
public:
    QPushButton *forwardButton;
    QPushButton *backwardButton;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QLineEdit *speedEdit;
    QLabel *label_3;
    QPushButton *leftButton;
    QPushButton *rightButton;
    QWidget *layoutWidget_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_5;
    QLineEdit *revolutionEdit;
    QLabel *label_6;
    QPushButton *resetButton;
    QCheckBox *railBox;
    QCheckBox *rotateBox;
    QPushButton *rotateButton;
    QCheckBox *leftBox;
    QCheckBox *rightBox;

    void setupUi(QWidget *MotionControl)
    {
        if (MotionControl->objectName().isEmpty())
            MotionControl->setObjectName(QStringLiteral("MotionControl"));
        MotionControl->resize(598, 454);
        forwardButton = new QPushButton(MotionControl);
        forwardButton->setObjectName(QStringLiteral("forwardButton"));
        forwardButton->setGeometry(QRect(410, 40, 75, 51));
        backwardButton = new QPushButton(MotionControl);
        backwardButton->setObjectName(QStringLiteral("backwardButton"));
        backwardButton->setGeometry(QRect(410, 120, 75, 61));
        layoutWidget = new QWidget(MotionControl);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(30, 70, 207, 22));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout_2->addWidget(label);

        speedEdit = new QLineEdit(layoutWidget);
        speedEdit->setObjectName(QStringLiteral("speedEdit"));

        horizontalLayout_2->addWidget(speedEdit);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_2->addWidget(label_3);

        leftButton = new QPushButton(MotionControl);
        leftButton->setObjectName(QStringLiteral("leftButton"));
        leftButton->setGeometry(QRect(350, 190, 75, 61));
        rightButton = new QPushButton(MotionControl);
        rightButton->setObjectName(QStringLiteral("rightButton"));
        rightButton->setGeometry(QRect(470, 190, 75, 61));
        layoutWidget_2 = new QWidget(MotionControl);
        layoutWidget_2->setObjectName(QStringLiteral("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(40, 210, 207, 22));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget_2);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_5 = new QLabel(layoutWidget_2);
        label_5->setObjectName(QStringLiteral("label_5"));

        horizontalLayout_3->addWidget(label_5);

        revolutionEdit = new QLineEdit(layoutWidget_2);
        revolutionEdit->setObjectName(QStringLiteral("revolutionEdit"));

        horizontalLayout_3->addWidget(revolutionEdit);

        label_6 = new QLabel(layoutWidget_2);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_3->addWidget(label_6);

        resetButton = new QPushButton(MotionControl);
        resetButton->setObjectName(QStringLiteral("resetButton"));
        resetButton->setGeometry(QRect(50, 320, 91, 71));
        resetButton->setStyleSheet(QStringLiteral("background-color: rgb(0, 255, 0);"));
        railBox = new QCheckBox(MotionControl);
        railBox->setObjectName(QStringLiteral("railBox"));
        railBox->setGeometry(QRect(160, 330, 71, 16));
        rotateBox = new QCheckBox(MotionControl);
        rotateBox->setObjectName(QStringLiteral("rotateBox"));
        rotateBox->setGeometry(QRect(160, 360, 71, 16));
        rotateButton = new QPushButton(MotionControl);
        rotateButton->setObjectName(QStringLiteral("rotateButton"));
        rotateButton->setGeometry(QRect(420, 320, 91, 71));
        leftBox = new QCheckBox(MotionControl);
        leftBox->setObjectName(QStringLiteral("leftBox"));
        leftBox->setGeometry(QRect(330, 330, 81, 16));
        rightBox = new QCheckBox(MotionControl);
        rightBox->setObjectName(QStringLiteral("rightBox"));
        rightBox->setGeometry(QRect(330, 360, 81, 16));

        retranslateUi(MotionControl);

        QMetaObject::connectSlotsByName(MotionControl);
    } // setupUi

    void retranslateUi(QWidget *MotionControl)
    {
        MotionControl->setWindowTitle(QApplication::translate("MotionControl", "MotionControl", Q_NULLPTR));
        forwardButton->setText(QApplication::translate("MotionControl", "forward", Q_NULLPTR));
        backwardButton->setText(QApplication::translate("MotionControl", "backward", Q_NULLPTR));
        label->setText(QApplication::translate("MotionControl", "\351\200\237\345\272\246\357\274\232", Q_NULLPTR));
        speedEdit->setText(QApplication::translate("MotionControl", "10", Q_NULLPTR));
        label_3->setText(QApplication::translate("MotionControl", "mm/s", Q_NULLPTR));
        leftButton->setText(QApplication::translate("MotionControl", "left", Q_NULLPTR));
        rightButton->setText(QApplication::translate("MotionControl", "right", Q_NULLPTR));
        label_5->setText(QApplication::translate("MotionControl", "\350\275\254\345\272\246\357\274\232", Q_NULLPTR));
        revolutionEdit->setText(QApplication::translate("MotionControl", "2", Q_NULLPTR));
        label_6->setText(QApplication::translate("MotionControl", "deg/s", Q_NULLPTR));
        resetButton->setText(QApplication::translate("MotionControl", "\345\244\215\344\275\215", Q_NULLPTR));
        railBox->setText(QApplication::translate("MotionControl", "\345\257\274\350\275\250\345\244\215\344\275\215", Q_NULLPTR));
        rotateBox->setText(QApplication::translate("MotionControl", "\350\275\254\345\217\260\345\244\215\344\275\215", Q_NULLPTR));
        rotateButton->setText(QApplication::translate("MotionControl", "\346\227\213\350\275\254", Q_NULLPTR));
        leftBox->setText(QApplication::translate("MotionControl", "\346\227\213\350\275\254\350\207\263\345\267\246\344\276\247", Q_NULLPTR));
        rightBox->setText(QApplication::translate("MotionControl", "\346\227\213\350\275\254\350\207\263\345\217\263\344\276\247", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MotionControl: public Ui_MotionControl {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MOTIONCONTROL_H
