#include "scrubot_gui_ros/mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
     ui->setupUi(this);
     qApp->setStyle("fusion");
     initUiPercent();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    qDebug() << "cur index change!";
}


void MainWindow::on_pushButton_turnmotor_updown_clicked()
{
        static bool cur_state = false;
        if(!cur_state)ui->pushButton_turnmotor_updown->setText(tr("降电机"));
        else ui->pushButton_turnmotor_updown->setText(tr("升电机"));
        cur_state = !cur_state;
}

void MainWindow::on_pushButton_turnmotor_en_clicked()
{
        static bool cur_state = false;
        if(!cur_state)ui->pushButton_turnmotor_en->setText(tr("关闭电机"));
        else ui->pushButton_turnmotor_en->setText(tr("转动电机"));
        cur_state = !cur_state;
}

void MainWindow::on_pushButton_speedforward_clicked()
{
    qDebug() << "speed forward ";
}

void MainWindow::on_pushButton_speedbackward_clicked()
{
    qDebug() << "speed backward ";
}

void MainWindow::on_pushButton_speedturnleft_clicked()
{
    qDebug() << "speed turn left ";
}

void MainWindow::on_pushButton_sppedturnright_clicked()
{
    qDebug() << "speed turn right ";
}

void MainWindow::on_pushButton_speedstop_clicked()
{
     qDebug() << "speed stop ";
}

void MainWindow::on_pushButton_waterscraper_updown_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_waterscraper_updown->setText(tr("降下电机"));
    else ui->pushButton_waterscraper_updown->setText(tr("升起电机"));
    cur_state = !cur_state;

}

void MainWindow::on_pushButton_sewage_absorb_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_sewage_absorb->setText(tr("停止吸污"));
    else ui->pushButton_sewage_absorb->setText(tr("启动吸污"));
    cur_state = !cur_state;
} 

void MainWindow::on_pushButton_water_en_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_water_en->setText(tr("关闭水阀"));
    else ui->pushButton_water_en->setText(tr("打开水阀"));
    cur_state = !cur_state;
}


void MainWindow::on_pushButton_startstop_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_startstop ->setText(tr("停止"));
    else ui->pushButton_startstop->setText(tr("启动"));
    cur_state = !cur_state;
}

void MainWindow::on_pushButton_pause_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_pause->setText(tr("继续"));
    else ui->pushButton_pause->setText(tr("暂停"));
    cur_state = !cur_state;
}


void MainWindow::on_pushButton_light_en_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_light_en->setText(tr("关闭灯带"));
    else ui->pushButton_light_en->setText(tr("开启灯带"));
    cur_state = !cur_state;

}


void MainWindow::on_pushButton_stop_en_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_stop_en->setText(tr("关闭停障"));
    else ui->pushButton_stop_en->setText("开启停障");
    cur_state = !cur_state;
}


void MainWindow::on_pushButton_obs_avoid_en_clicked()
{
    static bool cur_state = false;
        if(!cur_state)ui->pushButton_obs_avoid_en->setText(tr("关闭绕障"));
        else ui->pushButton_obs_avoid_en->setText(tr("开启绕障"));
        cur_state = !cur_state;
}

void MainWindow::connectToSlider(QRoundProgressBar *bar)
{
    bar->setRange(ui->slider->minimum(), ui->slider->maximum());
    bar->setValue(ui->slider->value());
    connect(ui->slider, SIGNAL(valueChanged(int)), bar, SLOT(setValue(int)));
}

void MainWindow::initUiPercent(){
    // 1,line for task process

    ui->tab->setAutoFillBackground(true); // 这句要加上, 否则可能显示不出背景图.
    //QPalette palette = this->palette();
    //palette.setBrush(QPalette::Window, QColor(0xff,0x00,0xff,0xf0));
    //ui->tab->setPalette(palette);                          // 给widget加上背景图

     //QPalette p = this->palette();
     //ui->tabWidget->setAutoFillBackground(true);
     //QT tabWidget 设置背景透明
    //ui->tab->setStyleSheet("QTabWidget:pane {border-top:0px solid #e8f3f9;background:  transparent; }");


    //ui->tabWidget->setPalette(p);
    // donut (default style)

    ui->progressBar_process->setDecimals(0);
    ui->progressBar_process->setBarStyle(QRoundProgressBar::StyleLine);
    connectToSlider(ui->progressBar_process);

    // 2,pie for water
    ui->progressBar_water->setNullPosition(QRoundProgressBar::PositionRight);
    ui->progressBar_water->setBarStyle(QRoundProgressBar::StylePie);
    connectToSlider(ui->progressBar_water);

    // 3,pie for sewage
    ui->progressBar_sewage->setNullPosition(QRoundProgressBar::PositionRight);
    ui->progressBar_sewage->setBarStyle(QRoundProgressBar::StylePie);
    connectToSlider(ui->progressBar_sewage);

    // 4,donut for power(customized via QPalette + gradient)
    // pie (customized via QPalette + gradient)

    // make a gradient from green over yellow to red
    ui->progressBar_power->setDecimals(2);
    ui->progressBar_power->setBarStyle(QRoundProgressBar::StyleLine);
    ui->progressBar_power->setOutlinePenWidth(12);
    ui->progressBar_power->setDataPenWidth(10);
    connectToSlider(ui->progressBar_power);

    //5,temperature
    //ui->temperatute_widget->setValue(40);
    //bar->setRange(-100, 100);
   // bar->setValue(40);
    connect(ui->slider, SIGNAL(valueChanged(int)), ui->temperatute_widget, SLOT(setValue(int)));

    // 6 for humidity line style with custom outline
    ui->progressBar_humidity->setDecimals(0);
    ui->progressBar_humidity->setBarStyle(QRoundProgressBar::StyleLine);
    ui->progressBar_humidity->setOutlinePenWidth(12);
    ui->progressBar_humidity->setDataPenWidth(10);
    connectToSlider(ui->progressBar_humidity);



}

