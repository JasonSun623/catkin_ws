#ifndef SCRUBOT_GUI_H
#define SCRUBOT_GUI_H

#include <QWidget>
#include "qroundprocess/QRoundProgressBar.h"
#include "temperature/thermometer.h"
#include <QTextCodec>
#include <QGridLayout>
#include <QDebug>
#include "ui_scrubot_gui.h"
namespace Ui {
class scrubot_gui;
}

class scrubot_gui : public QWidget
{
    Q_OBJECT

public:
    explicit scrubot_gui(QWidget *parent = 0);
    ~scrubot_gui();
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


private:
    Ui::scrubot_gui *ui;
    //task btn
    bool btn_task_start;
    bool btn_task_continue;
    //device manage
    // forward
    bool brush_down;//前部刷盘
    bool brush_motor_run;//刷盘启动
    // back ward
    bool squeegee_down;//尾部水扒
    bool suction_motor_run;//吸水电机
    bool water_value_open;//清水阀
    //sensor
    bool lamp_with_on;//灯带
    //soft func
    bool obs_stop_on;//停障
    bool obs_avoid_on;//绕障



};

#endif // SCRUBOT_GUI_H
