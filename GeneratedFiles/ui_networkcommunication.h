/********************************************************************************
** Form generated from reading UI file 'networkcommunication.ui'
**
** Created by: Qt User Interface Compiler version 5.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NETWORKCOMMUNICATION_H
#define UI_NETWORKCOMMUNICATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_NetworkCommunication
{
public:
    QLabel *label_12;
    QPushButton *DisconnectButton;
    QLabel *label_11;
    QPushButton *SendButton;
    QTextEdit *sendEdit;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_6;
    QLineEdit *lineEdit_1;
    QLabel *label_7;
    QLineEdit *lineEdit_2;
    QLabel *label_8;
    QLineEdit *lineEdit_3;
    QLabel *label_9;
    QLineEdit *lineEdit_4;
    QLabel *label_10;
    QLineEdit *lineEdit_5;
    QPushButton *ConnectButton;
    QTextEdit *ReceiveEdit;
    QPushButton *SaveButton;

    void setupUi(QWidget *NetworkCommunication)
    {
        if (NetworkCommunication->objectName().isEmpty())
            NetworkCommunication->setObjectName(QStringLiteral("NetworkCommunication"));
        NetworkCommunication->resize(853, 410);
        label_12 = new QLabel(NetworkCommunication);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(10, 260, 101, 41));
        QFont font;
        font.setFamily(QString::fromUtf8("\346\245\267\344\275\223"));
        font.setPointSize(20);
        label_12->setFont(font);
        DisconnectButton = new QPushButton(NetworkCommunication);
        DisconnectButton->setObjectName(QStringLiteral("DisconnectButton"));
        DisconnectButton->setGeometry(QRect(140, 90, 75, 23));
        label_11 = new QLabel(NetworkCommunication);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(10, 140, 101, 41));
        label_11->setFont(font);
        SendButton = new QPushButton(NetworkCommunication);
        SendButton->setObjectName(QStringLiteral("SendButton"));
        SendButton->setGeometry(QRect(710, 310, 121, 71));
        sendEdit = new QTextEdit(NetworkCommunication);
        sendEdit->setObjectName(QStringLiteral("sendEdit"));
        sendEdit->setGeometry(QRect(10, 310, 661, 71));
        layoutWidget = new QWidget(NetworkCommunication);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 10, 823, 41));
        horizontalLayout_8 = new QHBoxLayout(layoutWidget);
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        horizontalLayout_8->setSizeConstraint(QLayout::SetDefaultConstraint);
        horizontalLayout_8->setContentsMargins(0, 0, 0, 0);
        label_6 = new QLabel(layoutWidget);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_8->addWidget(label_6);

        lineEdit_1 = new QLineEdit(layoutWidget);
        lineEdit_1->setObjectName(QStringLiteral("lineEdit_1"));

        horizontalLayout_8->addWidget(lineEdit_1);

        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setFont(font);

        horizontalLayout_8->addWidget(label_7);

        lineEdit_2 = new QLineEdit(layoutWidget);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));

        horizontalLayout_8->addWidget(lineEdit_2);

        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setFont(font);

        horizontalLayout_8->addWidget(label_8);

        lineEdit_3 = new QLineEdit(layoutWidget);
        lineEdit_3->setObjectName(QStringLiteral("lineEdit_3"));

        horizontalLayout_8->addWidget(lineEdit_3);

        label_9 = new QLabel(layoutWidget);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setFont(font);

        horizontalLayout_8->addWidget(label_9);

        lineEdit_4 = new QLineEdit(layoutWidget);
        lineEdit_4->setObjectName(QStringLiteral("lineEdit_4"));

        horizontalLayout_8->addWidget(lineEdit_4);

        label_10 = new QLabel(layoutWidget);
        label_10->setObjectName(QStringLiteral("label_10"));

        horizontalLayout_8->addWidget(label_10);

        lineEdit_5 = new QLineEdit(layoutWidget);
        lineEdit_5->setObjectName(QStringLiteral("lineEdit_5"));

        horizontalLayout_8->addWidget(lineEdit_5);

        ConnectButton = new QPushButton(NetworkCommunication);
        ConnectButton->setObjectName(QStringLiteral("ConnectButton"));
        ConnectButton->setGeometry(QRect(40, 90, 75, 23));
        ReceiveEdit = new QTextEdit(NetworkCommunication);
        ReceiveEdit->setObjectName(QStringLiteral("ReceiveEdit"));
        ReceiveEdit->setGeometry(QRect(10, 180, 661, 71));
        SaveButton = new QPushButton(NetworkCommunication);
        SaveButton->setObjectName(QStringLiteral("SaveButton"));
        SaveButton->setGeometry(QRect(240, 90, 75, 23));

        retranslateUi(NetworkCommunication);

        QMetaObject::connectSlotsByName(NetworkCommunication);
    } // setupUi

    void retranslateUi(QWidget *NetworkCommunication)
    {
        NetworkCommunication->setWindowTitle(QApplication::translate("NetworkCommunication", "NetworkCommunication", Q_NULLPTR));
        label_12->setText(QApplication::translate("NetworkCommunication", "\345\217\221\351\200\201\357\274\232", Q_NULLPTR));
        DisconnectButton->setText(QApplication::translate("NetworkCommunication", "\346\226\255\345\274\200", Q_NULLPTR));
        label_11->setText(QApplication::translate("NetworkCommunication", "\346\216\245\346\224\266\357\274\232", Q_NULLPTR));
        SendButton->setText(QApplication::translate("NetworkCommunication", "\345\217\221\351\200\201", Q_NULLPTR));
        label_6->setText(QApplication::translate("NetworkCommunication", "IP\357\274\232", Q_NULLPTR));
        label_7->setText(QApplication::translate("NetworkCommunication", ".", Q_NULLPTR));
        label_8->setText(QApplication::translate("NetworkCommunication", ".", Q_NULLPTR));
        label_9->setText(QApplication::translate("NetworkCommunication", ".", Q_NULLPTR));
        label_10->setText(QApplication::translate("NetworkCommunication", "\347\253\257\345\217\243\357\274\232", Q_NULLPTR));
        ConnectButton->setText(QApplication::translate("NetworkCommunication", "\350\277\236\346\216\245", Q_NULLPTR));
        SaveButton->setText(QApplication::translate("NetworkCommunication", "\344\277\235\345\255\230\345\234\260\345\235\200", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class NetworkCommunication: public Ui_NetworkCommunication {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NETWORKCOMMUNICATION_H
