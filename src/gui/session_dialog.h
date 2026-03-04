#pragma once

#include <QDialog>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QListWidget>
#include <QTabWidget>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QDir>

namespace mocap {

class SessionDialog : public QDialog {
    Q_OBJECT
public:
    enum Mode { NewSession, OpenSession };

    explicit SessionDialog(Mode mode, QWidget* parent = nullptr);

    QString sessionName() const;
    QString outputDirectory() const;
    double recordingFps() const;
    QString selectedSessionPath() const;

private slots:
    void onBrowseDirectory();
    void onSessionSelected(QListWidgetItem* item);
    void onAccept();

private:
    Mode mode_;

    // New session widgets
    QLineEdit* name_edit_ = nullptr;
    QLineEdit* dir_edit_ = nullptr;
    QDoubleSpinBox* fps_spin_ = nullptr;

    // Open session widgets
    QListWidget* session_list_ = nullptr;
    QLineEdit* session_path_edit_ = nullptr;
    QString selected_session_path_;

    void setupNewSessionUI(QVBoxLayout* layout);
    void setupOpenSessionUI(QVBoxLayout* layout);
    void populateRecentSessions();
};

}  // namespace mocap
