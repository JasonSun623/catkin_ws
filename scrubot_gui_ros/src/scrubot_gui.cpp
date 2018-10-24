
#include "scrubot_gui_ros/scrubot_gui.h"
#include <iostream>
#include <QDir>
using namespace std;
scrubot_gui::scrubot_gui(QWidget *parent) :
    QWidget(parent,Qt::FramelessWindowHint),
    ui(new Ui::scrubot_gui)
{
    ui->setupUi(this);
    qApp->setStyle("fusion");

    // set widget black backgroundimg
    QPalette Pal(palette());
    //使用paletette填充widget背景，确保子类背景不被覆盖
    //冒号后面跟的是qrc里面的图片路径
    QString filename = ":/company/widget_bk.png";
    QPixmap pixmap(filename);
    QBrush b(pixmap);
    Pal.setBrush(QPalette::Background,b);
    setAutoFillBackground(true);
    setPalette(Pal);
    btn_task_start = false;
    btn_task_continue =false;
    //device manage
    // forward
    brush_down = false;//前部刷盘
    brush_motor_run = false;//刷盘启动
    // back ward
     squeegee_down = false;//尾部水扒
     suction_motor_run = false;//吸水电机
     water_value_open = false;//清水阀
    //sensor
     lamp_with_on = true;//灯带
    //soft func
     obs_stop_on = false;//停障
     obs_avoid_on = false;//绕障
     initUiPercent();

}

scrubot_gui::~scrubot_gui()
{
    delete ui;
}
void scrubot_gui::on_comboBox_currentIndexChanged(int index)
{
    qDebug() << "cur index change!";
}

//按照命名规则 这些槽函数自动链接
void scrubot_gui::on_pushButton_turnmotor_updown_clicked()
{
        static bool cur_state = false;
        QString s = "border:2px groove gray;border-radius:10px;padding:2px 4px;color: rgb(255, 255, 255);";
        if(!cur_state){
              ui->label_brush_down_light->setStyleSheet("border-image: url(:/util/elp_green.png);");//灯
              ui->pushButton_turnmotor_updown->setText(tr("升起刷盘"));//文字
              s = s + ";background-color: rgb(0, 170, 255);";//背景颜色
              ui->pushButton_turnmotor_updown->setStyleSheet(s);
        }
        else {
             ui->label_brush_down_light->setStyleSheet("border-image: url(:/util/elp_gray.png);");
             ui->pushButton_turnmotor_updown->setText(tr("降下刷盘"));
             s = s + ";background-color: rgb(96,96,96);";
             ui->pushButton_turnmotor_updown->setStyleSheet(s);
        }
        cur_state = !cur_state;

}

void scrubot_gui::on_pushButton_turnmotor_en_clicked()
{
        static bool cur_state = false;
        if(!cur_state)ui->pushButton_turnmotor_en->setText(tr("关闭电机"));
        else ui->pushButton_turnmotor_en->setText(tr("转动电机"));
        cur_state = !cur_state;
}

void scrubot_gui::on_pushButton_speedforward_clicked()
{
    qDebug() << "speed forward ";
}

void scrubot_gui::on_pushButton_speedbackward_clicked()
{
    qDebug() << "speed backward ";
}

void scrubot_gui::on_pushButton_speedturnleft_clicked()
{
    qDebug() << "speed turn left ";
}

void scrubot_gui::on_pushButton_sppedturnright_clicked()
{
    qDebug() << "speed turn right ";
}

void scrubot_gui::on_pushButton_speedstop_clicked()
{
     qDebug() << "speed stop ";
}

void scrubot_gui::on_pushButton_waterscraper_updown_clicked()
{
    /*static bool cur_state = false;
    if(!cur_state)ui->pushButton_waterscraper_updown->setText(tr("降下电机"));
    else ui->pushButton_waterscraper_updown->setText(tr("升起电机"));
    cur_state = !cur_state;
    */

    static bool cur_state = false;
    QString s = "border:2px groove gray;border-radius:10px;padding:2px 4px;color: rgb(255, 255, 255);";
    if(!cur_state){

          ui->pushButton_waterscraper_updown->setText(tr("降下电机"));
          s = s + ";background-color: rgb(255,0,0);";
          ui->pushButton_waterscraper_updown->setStyleSheet(s);
    }
    else {
         ui->pushButton_waterscraper_updown->setText(tr("升起电机"));
         s = s + ";background-color: rgb(96,96,96);";
         ui->pushButton_waterscraper_updown->setStyleSheet(s);
    }
    cur_state = !cur_state;

}

void scrubot_gui::on_pushButton_sewage_absorb_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_sewage_absorb->setText(tr("停止吸污"));
    else ui->pushButton_sewage_absorb->setText(tr("启动吸污"));
    cur_state = !cur_state;
}

void scrubot_gui::on_pushButton_water_en_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_water_en->setText(tr("关闭水阀"));
    else ui->pushButton_water_en->setText(tr("打开水阀"));
    cur_state = !cur_state;
}


void scrubot_gui::on_pushButton_startstop_clicked()
{
    static bool cur_state = false;
    QString s = "border:2px groove gray;border-radius:10px;padding:2px 4px;color: rgb(255, 255, 255);";
    if(!cur_state){

          ui->pushButton_startstop->setText(tr("停止"));
          s = s + ";background-color: rgb(255,0,0);";
          ui->pushButton_startstop->setStyleSheet(s);
    }
    else {
         ui->pushButton_startstop->setText(tr("启动"));
         s = s + ";background-color: rgb(0, 170, 255);";
         ui->pushButton_startstop->setStyleSheet(s);
    }
    cur_state = !cur_state;
}


void scrubot_gui::on_pushButton_pause_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_pause->setText(tr("继续"));
    else ui->pushButton_pause->setText(tr("暂停"));
    cur_state = !cur_state;
}


void scrubot_gui::on_pushButton_light_en_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_light_en->setText(tr("关闭灯带"));
    else ui->pushButton_light_en->setText(tr("开启灯带"));
    cur_state = !cur_state;

}


void scrubot_gui::on_pushButton_stop_en_clicked()
{
    static bool cur_state = false;
    if(!cur_state)ui->pushButton_stop_en->setText(tr("关闭停障"));
    else ui->pushButton_stop_en->setText("开启停障");
    cur_state = !cur_state;
}


void scrubot_gui::on_pushButton_obs_avoid_en_clicked()
{
    static bool cur_state = false;
        if(!cur_state)ui->pushButton_obs_avoid_en->setText(tr("关闭绕障"));
        else ui->pushButton_obs_avoid_en->setText(tr("开启绕障"));
        cur_state = !cur_state;
}
void scrubot_gui::connectToSlider(QRoundProgressBar *bar)
{
    bar->setRange(ui->slider->minimum(), ui->slider->maximum());
    bar->setValue(ui->slider->value());
    connect(ui->slider, SIGNAL(valueChanged(int)), bar, SLOT(setValue(int)));
}

void scrubot_gui::initUiPercent(){
    // 1,line for task process

    //ui->tab->setAutoFillBackground(true); // 这句要加上, 否则可能显示不出背景图.
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
    ui->progressBar_process->setOutlinePenWidth(12);
    ui->progressBar_process->setDataPenWidth(10);
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
    // line  for power(customized via QPalette + gradient)
    ui->progressBar_power->setDecimals(2);
    ui->progressBar_power->setBarStyle(QRoundProgressBar::StyleLine);
    ui->progressBar_power->setOutlinePenWidth(14);
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
   //添加网格布局
    //process_layout = new QGridLayout;
    //process_layout->addWidget()

    //set label group frame color
   ui->label_group_process->setFrameShape (QFrame::Box);
   //ui->label_group_process->setStyleSheet("QLabel{border:1px solid rgb(255, 255, 255);}");
   //ui->label_group_process->setStyleSheet(QString(":focus{ background-color: red; }"));

   ui->label_group_water->setFrameShape (QFrame::Box);
   //ui->label_group_water->setStyleSheet("QLabel{border:1px solid rgb(255, 255, 255);}");

   ui->label_group_sewage->setFrameShape (QFrame::Box);
   //ui->label_group_sewage->setStyleSheet("QLabel{border:1px solid rgb(255, 255, 255);}");

   ui->label_group_power->setFrameShape (QFrame::Box);
   //ui->label_group_power->setStyleSheet("QLabel{border:1px solid rgb(255, 255, 255);}");

   ui->label_group_humidity->setFrameShape (QFrame::Box);
   //ui->label_group_humidity->setStyleSheet("QLabel{border:1px solid rgb(255, 255, 255);}");

   ui->label_group_temperature->setFrameShape (QFrame::Box);
   //ui->label_group_temperature->setStyleSheet("QLabel{border:1px solid rgb(255, 255, 255);}");
   //ui->label_group_process->setAttribute(Qt::WA_TranslucentBackground);
//#endif

}
