#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qroundprocess/QRoundProgressBar.h"
#include "temperature/thermometer.h"
#include <QTextCodec>
#include <QGridLayout>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void connectToSlider(QRoundProgressBar *bar);
    void initUiPercent();
private slots:
    void on_comboBox_currentIndexChanged(int index);

    void on_pushButton_turnmotor_updown_clicked();

    void on_pushButton_turnmotor_en_clicked();

    void on_pushButton_speedforward_clicked();

    void on_pushButton_speedbackward_clicked();

    void on_pushButton_speedturnleft_clicked();

    void on_pushButton_sppedturnright_clicked();

    void on_pushButton_speedstop_clicked();

    void on_pushButton_waterscraper_updown_clicked();

    void on_pushButton_sewage_absorb_clicked();

    void on_pushButton_water_en_clicked();



    void on_pushButton_startstop_clicked();

    void on_pushButton_pause_clicked();

    //void on_pushButton_emergency_en_clicked();

    //void on_pushButton_anticollision_en_clicked();

    //void on_pushButton_ultro_en_clicked();

    void on_pushButton_light_en_clicked();

    void on_pushButton_stop_en_clicked();

    void on_pushButton_obs_avoid_en_clicked();

    //void on_pushButton_autocharge_en_clicked();

    //void on_pushButton_autowatering_clicked();


    //void on_pushButton_autodrain_clicked();

private:
    Ui::MainWindow *ui;
    QGridLayout *process_layout;
};

#endif // MAINWINDOW_H
