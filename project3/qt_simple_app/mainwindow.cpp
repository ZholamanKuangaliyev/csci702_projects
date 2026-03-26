#include "mainwindow.h"
#include <QVBoxLayout>
#include <QWidget>
#include <cmath>

// ── Button factory ────────────────────────────────────────────────────────────
QPushButton* MainWindow::makeButton(const QString& text, const QString& style) {
    auto* btn = new QPushButton(text);
    btn->setMinimumSize(70, 60);
    btn->setFont(QFont("Arial", 16, QFont::Medium));
    if (!style.isEmpty()) btn->setStyleSheet(style);
    return btn;
}

// ── Constructor ───────────────────────────────────────────────────────────────
MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("Qt 6 Calculator – Task 2");
    setFixedSize(340, 500);

    // ── Display ────────────────────────────────────────────────────────────
    m_display = new QLineEdit("0");
    m_display->setReadOnly(true);
    m_display->setAlignment(Qt::AlignRight);
    m_display->setFont(QFont("Arial", 26, QFont::Bold));
    m_display->setFixedHeight(80);
    m_display->setStyleSheet(
        "QLineEdit { background:#1c1c1e; color:#ffffff; "
        "border:none; border-radius:8px; padding:0 12px; }");

    // ── Button styles ──────────────────────────────────────────────────────
    const QString numStyle =
        "QPushButton { background:#333333; color:white; border-radius:8px; }"
        "QPushButton:pressed { background:#555555; }";
    const QString opStyle =
        "QPushButton { background:#ff9f0a; color:white; border-radius:8px; }"
        "QPushButton:pressed { background:#ffb340; }";
    const QString fnStyle =
        "QPushButton { background:#505050; color:white; border-radius:8px; }"
        "QPushButton:pressed { background:#707070; }";
    const QString eqStyle =
        "QPushButton { background:#ff9f0a; color:white; border-radius:8px; }"
        "QPushButton:pressed { background:#ffb340; }";

    // ── Row 0: AC  +/-  %  ÷ ─────────────────────────────────────────────
    auto* btnAC  = makeButton("AC",  fnStyle);
    auto* btnSgn = makeButton("+/-", fnStyle);
    auto* btnPct = makeButton("%",   fnStyle);
    auto* btnDiv = makeButton("÷",   opStyle);

    // ── Row 1: 7  8  9  × ────────────────────────────────────────────────
    auto* btn7   = makeButton("7", numStyle);
    auto* btn8   = makeButton("8", numStyle);
    auto* btn9   = makeButton("9", numStyle);
    auto* btnMul = makeButton("×", opStyle);

    // ── Row 2: 4  5  6  - ────────────────────────────────────────────────
    auto* btn4   = makeButton("4", numStyle);
    auto* btn5   = makeButton("5", numStyle);
    auto* btn6   = makeButton("6", numStyle);
    auto* btnSub = makeButton("-", opStyle);

    // ── Row 3: 1  2  3  + ────────────────────────────────────────────────
    auto* btn1   = makeButton("1", numStyle);
    auto* btn2   = makeButton("2", numStyle);
    auto* btn3   = makeButton("3", numStyle);
    auto* btnAdd = makeButton("+", opStyle);

    // ── Row 4: 0  .  ⌫  = ───────────────────────────────────────────────
    auto* btn0   = makeButton("0",  numStyle);
    auto* btnDot = makeButton(".",  numStyle);
    auto* btnBks = makeButton("⌫", fnStyle);
    auto* btnEq  = makeButton("=",  eqStyle);

    // ── Grid layout ────────────────────────────────────────────────────────
    auto* grid = new QGridLayout;
    grid->setSpacing(8);
    grid->addWidget(btnAC,  0, 0); grid->addWidget(btnSgn, 0, 1);
    grid->addWidget(btnPct, 0, 2); grid->addWidget(btnDiv, 0, 3);

    grid->addWidget(btn7,   1, 0); grid->addWidget(btn8,   1, 1);
    grid->addWidget(btn9,   1, 2); grid->addWidget(btnMul, 1, 3);

    grid->addWidget(btn4,   2, 0); grid->addWidget(btn5,   2, 1);
    grid->addWidget(btn6,   2, 2); grid->addWidget(btnSub, 2, 3);

    grid->addWidget(btn1,   3, 0); grid->addWidget(btn2,   3, 1);
    grid->addWidget(btn3,   3, 2); grid->addWidget(btnAdd, 3, 3);

    grid->addWidget(btn0,   4, 0); grid->addWidget(btnDot, 4, 1);
    grid->addWidget(btnBks, 4, 2); grid->addWidget(btnEq,  4, 3);

    auto* vbox = new QVBoxLayout;
    vbox->setContentsMargins(12, 12, 12, 12);
    vbox->setSpacing(10);
    vbox->addWidget(m_display);
    vbox->addLayout(grid);

    auto* central = new QWidget;
    central->setStyleSheet("background:#000000;");
    central->setLayout(vbox);
    setCentralWidget(central);

    // ── Connections ────────────────────────────────────────────────────────
    // Digit & decimal buttons
    for (auto [btn, ch] : std::initializer_list<std::pair<QPushButton*, QString>>{
            {btn0,"0"},{btn1,"1"},{btn2,"2"},{btn3,"3"},{btn4,"4"},
            {btn5,"5"},{btn6,"6"},{btn7,"7"},{btn8,"8"},{btn9,"9"},{btnDot,"."}}) {
        connect(btn, &QPushButton::clicked, this, [this, ch]{ onDigitClicked(ch); });
    }

    // Operator buttons
    connect(btnAdd, &QPushButton::clicked, this, [this]{ onOperatorClicked("+"); });
    connect(btnSub, &QPushButton::clicked, this, [this]{ onOperatorClicked("-"); });
    connect(btnMul, &QPushButton::clicked, this, [this]{ onOperatorClicked("*"); });
    connect(btnDiv, &QPushButton::clicked, this, [this]{ onOperatorClicked("/"); });

    // Function buttons
    connect(btnAC,  &QPushButton::clicked, this, &MainWindow::onClear);
    connect(btnBks, &QPushButton::clicked, this, &MainWindow::onBackspace);
    connect(btnSgn, &QPushButton::clicked, this, &MainWindow::onToggleSign);
    connect(btnPct, &QPushButton::clicked, this, &MainWindow::onPercent);
    connect(btnEq,  &QPushButton::clicked, this, &MainWindow::onEquals);
}

// ── Slot implementations ──────────────────────────────────────────────────────

void MainWindow::onDigitClicked(const QString& digit) {
    if (m_newNumber) {
        if (digit == ".") {
            m_display->setText("0.");
        } else {
            m_display->setText(digit);
        }
        m_newNumber = false;
    } else {
        // Prevent duplicate decimal point
        if (digit == "." && m_display->text().contains('.')) return;
        if (m_display->text() == "0" && digit != ".") {
            m_display->setText(digit);
        } else {
            m_display->setText(m_display->text() + digit);
        }
    }
}

void MainWindow::onOperatorClicked(const QString& op) {
    m_operand1   = m_display->text().toDouble();
    m_pendingOp  = op;
    m_newNumber  = true;
}

void MainWindow::onEquals() {
    if (m_pendingOp.isEmpty()) return;
    double operand2 = m_display->text().toDouble();
    double result = 0.0;

    if      (m_pendingOp == "+") result = m_operand1 + operand2;
    else if (m_pendingOp == "-") result = m_operand1 - operand2;
    else if (m_pendingOp == "*") result = m_operand1 * operand2;
    else if (m_pendingOp == "/") {
        if (std::abs(operand2) < 1e-15) {
            m_display->setText("Error"); m_pendingOp.clear(); return;
        }
        result = m_operand1 / operand2;
    }

    // Display without trailing ".0" for integers
    QString s = QString::number(result, 'g', 12);
    m_display->setText(s);
    m_pendingOp.clear();
    m_newNumber = true;
}

void MainWindow::onClear() {
    m_display->setText("0");
    m_operand1  = 0.0;
    m_pendingOp.clear();
    m_newNumber = true;
}

void MainWindow::onBackspace() {
    if (m_newNumber) return;
    QString text = m_display->text();
    text.chop(1);
    if (text.isEmpty() || text == "-") text = "0";
    m_display->setText(text);
}

void MainWindow::onToggleSign() {
    double v = m_display->text().toDouble();
    m_display->setText(QString::number(-v, 'g', 12));
}

void MainWindow::onPercent() {
    double v = m_display->text().toDouble();
    m_display->setText(QString::number(v / 100.0, 'g', 12));
    m_newNumber = true;
}
