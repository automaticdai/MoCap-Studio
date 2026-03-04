#pragma once

#include <QWidget>
#include <QSlider>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>

namespace mocap {

class TimelineWidget : public QWidget {
    Q_OBJECT
public:
    explicit TimelineWidget(QWidget* parent = nullptr);

    void setDuration(double seconds);
    void setCurrentTime(double seconds);
    void setFrameRate(double fps);

    double currentTime() const;
    double duration() const;

signals:
    void playRequested();
    void pauseRequested();
    void stopRequested();
    void recordRequested();
    void seekRequested(double time_seconds);

private slots:
    void onSliderMoved(int value);
    void onPlayClicked();
    void onStopClicked();
    void onRecordClicked();

private:
    QSlider* scrub_bar_;
    QPushButton* play_btn_;
    QPushButton* stop_btn_;
    QPushButton* record_btn_;
    QLabel* time_label_;
    QLabel* duration_label_;

    double duration_ = 0.0;
    double current_time_ = 0.0;
    double fps_ = 60.0;
    bool is_playing_ = false;
    bool is_recording_ = false;

    QString formatTime(double seconds) const;
    void updateTimeDisplay();
};

}  // namespace mocap
