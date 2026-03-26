#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QGridLayout>
#include <QString>

// Task 2 – Simple Qt 6 calculator.
// Demonstrates: QMainWindow, QLabel, QLineEdit, QPushButton, signals & slots.
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onDigitClicked(const QString& digit);
    void onOperatorClicked(const QString& op);
    void onEquals();
    void onClear();
    void onBackspace();
    void onToggleSign();
    void onPercent();

private:
    QLabel*    m_equation;   // shows the running equation above the display
    QLineEdit* m_display;

    // Calculator state
    double m_operand1{0.0};
    QString m_pendingOp{};
    QString m_operand1Str{}; // string representation of first operand for display
    bool m_newNumber{true};  // next digit starts a fresh number

    QPushButton* makeButton(const QString& text,
                            const QString& style = {});
};
