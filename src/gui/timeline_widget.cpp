#include "gui/timeline_widget.h"

namespace mocap {

TimelineWidget::TimelineWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(8, 4, 8, 4);
    layout->setSpacing(6);

    // Transport buttons
    play_btn_ = new QPushButton("Play", this);
    play_btn_->setFixedWidth(60);
    play_btn_->setStyleSheet(
        "QPushButton { background-color: #2a6e3f; color: white; border: 1px solid #3a8e4f; border-radius: 3px; padding: 4px; }"
        "QPushButton:hover { background-color: #3a8e4f; }"
    );

    stop_btn_ = new QPushButton("Stop", this);
    stop_btn_->setFixedWidth(50);
    stop_btn_->setStyleSheet(
        "QPushButton { background-color: #555; color: white; border: 1px solid #666; border-radius: 3px; padding: 4px; }"
        "QPushButton:hover { background-color: #666; }"
    );

    record_btn_ = new QPushButton("Rec", this);
    record_btn_->setFixedWidth(50);
    record_btn_->setStyleSheet(
        "QPushButton { background-color: #6e2a2a; color: white; border: 1px solid #8e3a3a; border-radius: 3px; padding: 4px; }"
        "QPushButton:hover { background-color: #8e3a3a; }"
    );

    // Time label
    time_label_ = new QLabel("00:00.000", this);
    time_label_->setFixedWidth(80);
    time_label_->setAlignment(Qt::AlignCenter);
    time_label_->setStyleSheet("QLabel { color: #00cc66; font-family: monospace; font-size: 12px; }");

    // Scrub bar
    scrub_bar_ = new QSlider(Qt::Horizontal, this);
    scrub_bar_->setRange(0, 1000);
    scrub_bar_->setValue(0);
    scrub_bar_->setStyleSheet(
        "QSlider::groove:horizontal { background: #333; height: 6px; border-radius: 3px; }"
        "QSlider::handle:horizontal { background: #00cc66; width: 12px; margin: -4px 0; border-radius: 6px; }"
        "QSlider::sub-page:horizontal { background: #006633; border-radius: 3px; }"
    );

    // Duration label
    duration_label_ = new QLabel("00:00.000", this);
    duration_label_->setFixedWidth(80);
    duration_label_->setAlignment(Qt::AlignCenter);
    duration_label_->setStyleSheet("QLabel { color: #999; font-family: monospace; font-size: 12px; }");

    // Layout
    layout->addWidget(play_btn_);
    layout->addWidget(stop_btn_);
    layout->addWidget(record_btn_);
    layout->addWidget(time_label_);
    layout->addWidget(scrub_bar_, 1);
    layout->addWidget(duration_label_);

    setLayout(layout);

    // Connections
    connect(play_btn_, &QPushButton::clicked, this, &TimelineWidget::onPlayClicked);
    connect(stop_btn_, &QPushButton::clicked, this, &TimelineWidget::onStopClicked);
    connect(record_btn_, &QPushButton::clicked, this, &TimelineWidget::onRecordClicked);
    connect(scrub_bar_, &QSlider::sliderMoved, this, &TimelineWidget::onSliderMoved);

    setFixedHeight(40);
    setStyleSheet("QWidget { background-color: #1e1e24; }");
}

void TimelineWidget::setDuration(double seconds) {
    duration_ = seconds;
    duration_label_->setText(formatTime(seconds));
}

void TimelineWidget::setCurrentTime(double seconds) {
    current_time_ = seconds;
    time_label_->setText(formatTime(seconds));

    // Update slider without triggering sliderMoved
    if (!scrub_bar_->isSliderDown()) {
        int pos = (duration_ > 0.0)
            ? static_cast<int>((seconds / duration_) * 1000.0)
            : 0;
        scrub_bar_->setValue(pos);
    }
}

void TimelineWidget::setFrameRate(double fps) {
    fps_ = fps;
}

double TimelineWidget::currentTime() const {
    return current_time_;
}

double TimelineWidget::duration() const {
    return duration_;
}

void TimelineWidget::onSliderMoved(int value) {
    double time = (duration_ * value) / 1000.0;
    current_time_ = time;
    time_label_->setText(formatTime(time));
    emit seekRequested(time);
}

void TimelineWidget::onPlayClicked() {
    is_playing_ = !is_playing_;
    if (is_playing_) {
        play_btn_->setText("Pause");
        play_btn_->setStyleSheet(
            "QPushButton { background-color: #6e6e2a; color: white; border: 1px solid #8e8e3a; border-radius: 3px; padding: 4px; }"
            "QPushButton:hover { background-color: #8e8e3a; }"
        );
        emit playRequested();
    } else {
        play_btn_->setText("Play");
        play_btn_->setStyleSheet(
            "QPushButton { background-color: #2a6e3f; color: white; border: 1px solid #3a8e4f; border-radius: 3px; padding: 4px; }"
            "QPushButton:hover { background-color: #3a8e4f; }"
        );
        emit pauseRequested();
    }
}

void TimelineWidget::onStopClicked() {
    is_playing_ = false;
    is_recording_ = false;
    play_btn_->setText("Play");
    play_btn_->setStyleSheet(
        "QPushButton { background-color: #2a6e3f; color: white; border: 1px solid #3a8e4f; border-radius: 3px; padding: 4px; }"
        "QPushButton:hover { background-color: #3a8e4f; }"
    );
    record_btn_->setText("Rec");
    record_btn_->setStyleSheet(
        "QPushButton { background-color: #6e2a2a; color: white; border: 1px solid #8e3a3a; border-radius: 3px; padding: 4px; }"
        "QPushButton:hover { background-color: #8e3a3a; }"
    );
    emit stopRequested();
}

void TimelineWidget::onRecordClicked() {
    is_recording_ = !is_recording_;
    if (is_recording_) {
        record_btn_->setText("REC");
        record_btn_->setStyleSheet(
            "QPushButton { background-color: #cc0000; color: white; border: 1px solid #ff3333; border-radius: 3px; padding: 4px; }"
            "QPushButton:hover { background-color: #ff3333; }"
        );
        emit recordRequested();
    } else {
        record_btn_->setText("Rec");
        record_btn_->setStyleSheet(
            "QPushButton { background-color: #6e2a2a; color: white; border: 1px solid #8e3a3a; border-radius: 3px; padding: 4px; }"
            "QPushButton:hover { background-color: #8e3a3a; }"
        );
    }
}

QString TimelineWidget::formatTime(double seconds) const {
    int mins = static_cast<int>(seconds) / 60;
    double secs = seconds - mins * 60;
    return QString("%1:%2")
        .arg(mins, 2, 10, QChar('0'))
        .arg(secs, 6, 'f', 3, QChar('0'));
}

void TimelineWidget::updateTimeDisplay() {
    time_label_->setText(formatTime(current_time_));
    duration_label_->setText(formatTime(duration_));
}

}  // namespace mocap
