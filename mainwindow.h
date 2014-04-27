#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void UpdateDisplay(void);

private slots:
    void on_pushButtonProgramVID_clicked();

private:
    Ui::MainWindow *ui;
    int m_cvid;
    int m_cpid;
    int m_nvid;
    int m_npid;
};

#endif // MAINWINDOW_H
