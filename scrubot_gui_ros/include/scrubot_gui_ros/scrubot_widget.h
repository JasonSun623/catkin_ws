#ifndef SCRUBOT_WIDGET_H
#define SCRUBOT_WIDGET_H
#include "qroundprocess/QRoundProgressBar.h"
#include "temperature/thermometer.h"
#include <QTextCodec>
#include <QGridLayout>
#include <QWidget>
#include <QSlider>
class scrubot_widget : public QWidget
{
    Q_OBJECT
public:
    explicit scrubot_widget(QWidget *parent = 0);
    void connectToSlider(QRoundProgressBar *bar);
    void initUiPercent();
signals:

public slots:
private:
    QRoundProgressBar *progressBar_process;
    QSlider *slider;
};

#endif // SCRUBOT_WIDGET_H
