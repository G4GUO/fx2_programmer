#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "fx2.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_cvid = 0x04B4;
    m_cpid = 0x8613;
    m_nvid = 0x04B4;
    m_npid = 0x8613;
    //
    UpdateDisplay();
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::UpdateDisplay(void)
{
    QString text;
    text.sprintf("0x%.4X",m_cvid);
    ui->lineEditCurrentVID->setText(text);
    text.sprintf("0x%.4X",m_nvid);
    ui->lineEditNewVID->setText(text);
    text.sprintf("0x%.4X",m_cpid);
    ui->lineEditCurrentPID->setText(text);
    text.sprintf("0x%.4X",m_npid);
    ui->lineEditNewPID->setText(text);
}

void MainWindow::on_pushButtonProgramVID_clicked()
{
    // Update the VID
    bool r;
    m_cvid = ui->lineEditCurrentVID->displayText().toInt( &r, 16);
    m_cpid = ui->lineEditCurrentPID->displayText().toInt( &r, 16);
    m_nvid = ui->lineEditNewVID->displayText().toInt( &r, 16);
    m_npid = ui->lineEditNewPID->displayText().toInt( &r, 16);

    ui->labelResult->setText("");

    fx2_init( m_cvid, m_cpid, m_nvid, m_npid );

    m_cvid = m_nvid;
    m_cpid = m_npid;
    UpdateDisplay();
    ui->labelResult->setText(fx2_result_message());
}
