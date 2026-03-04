#include "gui/session_dialog.h"
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QSettings>

namespace mocap {

SessionDialog::SessionDialog(Mode mode, QWidget* parent)
    : QDialog(parent)
    , mode_(mode)
{
    setWindowTitle(mode == NewSession ? "New Session" : "Open Session");
    setMinimumWidth(500);

    auto* layout = new QVBoxLayout(this);

    if (mode == NewSession) {
        setupNewSessionUI(layout);
    } else {
        setupOpenSessionUI(layout);
    }

    // Buttons
    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &SessionDialog::onAccept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);

    setLayout(layout);
    setStyleSheet(
        "QDialog { background-color: #2a2a30; }"
        "QLabel { color: #ccc; }"
        "QLineEdit { background-color: #1e1e24; color: #eee; border: 1px solid #444; "
        "border-radius: 3px; padding: 4px; }"
        "QGroupBox { color: #ccc; border: 1px solid #444; border-radius: 4px; "
        "margin-top: 8px; padding-top: 16px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"
    );
}

QString SessionDialog::sessionName() const {
    return name_edit_ ? name_edit_->text() : QString();
}

QString SessionDialog::outputDirectory() const {
    return dir_edit_ ? dir_edit_->text() : QString();
}

double SessionDialog::recordingFps() const {
    return fps_spin_ ? fps_spin_->value() : 60.0;
}

QString SessionDialog::selectedSessionPath() const {
    return selected_session_path_;
}

void SessionDialog::setupNewSessionUI(QVBoxLayout* layout) {
    auto* group = new QGroupBox("New Session", this);
    auto* glayout = new QVBoxLayout(group);

    // Session name
    auto* nameRow = new QHBoxLayout();
    nameRow->addWidget(new QLabel("Name:", this));
    name_edit_ = new QLineEdit(this);
    name_edit_->setPlaceholderText("e.g., walk_cycle_01");
    nameRow->addWidget(name_edit_);
    glayout->addLayout(nameRow);

    // Output directory
    auto* dirRow = new QHBoxLayout();
    dirRow->addWidget(new QLabel("Directory:", this));
    dir_edit_ = new QLineEdit(this);
    dir_edit_->setText(QDir::homePath() + "/MoCapSessions");
    dirRow->addWidget(dir_edit_);
    auto* browseBtn = new QPushButton("Browse...", this);
    connect(browseBtn, &QPushButton::clicked, this, &SessionDialog::onBrowseDirectory);
    dirRow->addWidget(browseBtn);
    glayout->addLayout(dirRow);

    // FPS
    auto* fpsRow = new QHBoxLayout();
    fpsRow->addWidget(new QLabel("Recording FPS:", this));
    fps_spin_ = new QDoubleSpinBox(this);
    fps_spin_->setRange(1.0, 240.0);
    fps_spin_->setValue(60.0);
    fps_spin_->setDecimals(1);
    fpsRow->addWidget(fps_spin_);
    fpsRow->addStretch();
    glayout->addLayout(fpsRow);

    group->setLayout(glayout);
    layout->addWidget(group);
}

void SessionDialog::setupOpenSessionUI(QVBoxLayout* layout) {
    auto* group = new QGroupBox("Open Session", this);
    auto* glayout = new QVBoxLayout(group);

    // Manual path
    auto* pathRow = new QHBoxLayout();
    pathRow->addWidget(new QLabel("Path:", this));
    session_path_edit_ = new QLineEdit(this);
    session_path_edit_->setPlaceholderText("Session directory path...");
    pathRow->addWidget(session_path_edit_);
    auto* browseBtn = new QPushButton("Browse...", this);
    connect(browseBtn, &QPushButton::clicked, this, &SessionDialog::onBrowseDirectory);
    pathRow->addWidget(browseBtn);
    glayout->addLayout(pathRow);

    // Recent sessions
    glayout->addWidget(new QLabel("Recent Sessions:", this));
    session_list_ = new QListWidget(this);
    session_list_->setStyleSheet(
        "QListWidget { background-color: #1e1e24; color: #ccc; border: 1px solid #444; }"
        "QListWidget::item:selected { background-color: #3a3a50; }");
    connect(session_list_, &QListWidget::itemClicked, this, &SessionDialog::onSessionSelected);
    glayout->addWidget(session_list_);

    populateRecentSessions();

    group->setLayout(glayout);
    layout->addWidget(group);
}

void SessionDialog::onBrowseDirectory() {
    if (mode_ == NewSession) {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Output Directory",
                                                         dir_edit_->text());
        if (!dir.isEmpty()) {
            dir_edit_->setText(dir);
        }
    } else {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Session Directory",
                                                         QDir::homePath());
        if (!dir.isEmpty()) {
            session_path_edit_->setText(dir);
            selected_session_path_ = dir;
        }
    }
}

void SessionDialog::onSessionSelected(QListWidgetItem* item) {
    selected_session_path_ = item->data(Qt::UserRole).toString();
    if (session_path_edit_) {
        session_path_edit_->setText(selected_session_path_);
    }
}

void SessionDialog::onAccept() {
    if (mode_ == NewSession) {
        if (name_edit_->text().trimmed().isEmpty()) {
            QMessageBox::warning(this, "Error", "Please enter a session name.");
            return;
        }
        if (dir_edit_->text().trimmed().isEmpty()) {
            QMessageBox::warning(this, "Error", "Please select an output directory.");
            return;
        }
    } else {
        if (selected_session_path_.isEmpty() && session_path_edit_) {
            selected_session_path_ = session_path_edit_->text();
        }
        if (selected_session_path_.isEmpty()) {
            QMessageBox::warning(this, "Error", "Please select a session to open.");
            return;
        }
    }
    accept();
}

void SessionDialog::populateRecentSessions() {
    QSettings settings("MoCapStudio", "MoCapStudio");
    QStringList recent = settings.value("recentSessions").toStringList();
    for (const auto& path : recent) {
        QDir dir(path);
        if (dir.exists()) {
            auto* item = new QListWidgetItem(dir.dirName() + " - " + path);
            item->setData(Qt::UserRole, path);
            session_list_->addItem(item);
        }
    }
    if (session_list_->count() == 0) {
        session_list_->addItem("(No recent sessions)");
        session_list_->item(0)->setFlags(Qt::NoItemFlags);
    }
}

}  // namespace mocap
