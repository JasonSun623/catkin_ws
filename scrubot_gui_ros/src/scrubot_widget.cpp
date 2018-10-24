#include "scrubot_gui_ros/scrubot_widget.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
scrubot_widget::scrubot_widget(QWidget *parent) : QWidget(parent)
{
    //qApp->setStyle("fusion");
    progressBar_process = new QRoundProgressBar;
    progressBar_process->setFixedWidth(100);
    progressBar_process->setFixedHeight(100);

    this->setAutoFillBackground(true);

    QPalette palette;

    //palette.setColor(QPalette::Background, QColor(192,253,123));

    palette.setBrush(QPalette::Background, QBrush(QPixmap("/home/it-robot/catkin_ws/src/scrubot_gui_ros/image/company/bk.png")));

    this->setPalette(palette);


    slider = new QSlider(Qt::Horizontal);
    QVBoxLayout *cmdLayout=new QVBoxLayout;
    cmdLayout->addWidget(progressBar_process);
    cmdLayout->addWidget(slider);
    this->setLayout(cmdLayout);
    initUiPercent();
}
void scrubot_widget::connectToSlider(QRoundProgressBar *bar)
{
    bar->setRange(slider->minimum(), slider->maximum());
    bar->setValue(slider->value());
    connect(slider, SIGNAL(valueChanged(int)), bar, SLOT(setValue(int)));
}

void scrubot_widget::initUiPercent(){
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

    progressBar_process->setDecimals(0);
    progressBar_process->setBarStyle(QRoundProgressBar::StyleLine);
    progressBar_process->setOutlinePenWidth(12);
    progressBar_process->setDataPenWidth(10);
    connectToSlider(progressBar_process);
   #if 0
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
    process_layout = new QGridLayout;
    //process_layout->addWidget()
     //set label group color
     ui->label_group_process->setFrameShape (QFrame::Box);
    ui->label_group_process->setStyleSheet("QLabel{border:2px solid rgb(0, 255, 0);}");
#endif

}
