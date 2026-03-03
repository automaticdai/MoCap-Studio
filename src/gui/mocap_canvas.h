#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <vector>
#include "core/types.h"
#include "core/skeleton_definition.h"

namespace mocap {

class MoCapCanvas : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core {
    Q_OBJECT
public:
    explicit MoCapCanvas(QWidget* parent = nullptr);
    ~MoCapCanvas() override;

public slots:
    void onPose3DUpdate(const std::vector<Pose3D>& poses);
    void onSkeletonUpdate(const std::vector<SkeletonPose>& skeletons);
    void setRenderLayerVisible(const QString& layer, bool visible);
    void frameSelection();

signals:
    void personSelected(int global_person_id);
    void jointSelected(int global_person_id, int joint_index);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    // Orbit camera
    struct OrbitCamera {
        float azimuth = 45.0f;
        float elevation = 30.0f;
        float distance = 5.0f;
        QVector3D target = QVector3D(0, 1, 0);

        QMatrix4x4 viewMatrix() const;
    } camera_;

    QMatrix4x4 projection_;

    // Render layers
    struct RenderLayers {
        bool grid = true;
        bool markers = true;
        bool skeleton = true;
        bool trails = false;
    } layers_;

    // Shader
    QOpenGLShaderProgram* shader_ = nullptr;

    // Geometry buffers
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_;

    // Data
    std::vector<Pose3D> current_poses_;
    std::vector<SkeletonPose> current_skeletons_;
    SkeletonDefinition skeleton_def_;
    int selected_person_ = -1;

    // Per-person colors (Oklab hue rotation)
    QColor colorForPerson(int global_person_id) const;

    // Drawing methods
    void drawGrid();
    void drawMarkers();
    void drawSkeleton();

    // Immediate-mode style helpers
    void drawLine(const QVector3D& from, const QVector3D& to,
                  const QColor& color, float width = 1.0f);
    void drawPoint(const QVector3D& pos, const QColor& color, float size = 5.0f);

    void setUniformColor(const QColor& color);
    void uploadAndDraw(const std::vector<float>& vertices, GLenum mode);

    // Mouse interaction state
    QPoint last_mouse_pos_;
    bool orbiting_ = false;
    bool panning_ = false;
};

}  // namespace mocap
