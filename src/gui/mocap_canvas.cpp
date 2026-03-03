#include "gui/mocap_canvas.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mocap {

MoCapCanvas::MoCapCanvas(QWidget* parent)
    : QOpenGLWidget(parent)
    , vbo_(QOpenGLBuffer::VertexBuffer)
{
    setFocusPolicy(Qt::StrongFocus);
    skeleton_def_ = SkeletonDefinition::defaultBody25();
}

MoCapCanvas::~MoCapCanvas() {
    makeCurrent();
    vbo_.destroy();
    vao_.destroy();
    delete shader_;
    doneCurrent();
}

QMatrix4x4 MoCapCanvas::OrbitCamera::viewMatrix() const {
    QMatrix4x4 view;
    float az_rad = azimuth * static_cast<float>(M_PI) / 180.0f;
    float el_rad = elevation * static_cast<float>(M_PI) / 180.0f;

    float x = distance * std::cos(el_rad) * std::sin(az_rad);
    float y = distance * std::sin(el_rad);
    float z = distance * std::cos(el_rad) * std::cos(az_rad);

    QVector3D eye = target + QVector3D(x, y, z);
    view.lookAt(eye, target, QVector3D(0, 1, 0));
    return view;
}

void MoCapCanvas::initializeGL() {
    initializeOpenGLFunctions();

    glClearColor(0.12f, 0.12f, 0.15f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_PROGRAM_POINT_SIZE);

    // Simple shader for colored vertices
    shader_ = new QOpenGLShaderProgram(this);
    shader_->addShaderFromSourceCode(QOpenGLShader::Vertex, R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        uniform mat4 uMVP;
        uniform vec4 uColor;
        uniform float uPointSize;
        out vec4 vColor;
        void main() {
            gl_Position = uMVP * vec4(aPos, 1.0);
            gl_PointSize = uPointSize;
            vColor = uColor;
        }
    )");
    shader_->addShaderFromSourceCode(QOpenGLShader::Fragment, R"(
        #version 330 core
        in vec4 vColor;
        out vec4 FragColor;
        void main() {
            FragColor = vColor;
        }
    )");
    shader_->link();

    vao_.create();
    vbo_.create();
}

void MoCapCanvas::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    projection_.setToIdentity();
    projection_.perspective(45.0f, static_cast<float>(w) / std::max(h, 1), 0.1f, 100.0f);
}

void MoCapCanvas::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shader_->bind();
    vao_.bind();

    QMatrix4x4 vp = projection_ * camera_.viewMatrix();
    shader_->setUniformValue("uMVP", vp);
    shader_->setUniformValue("uPointSize", 6.0f);

    if (layers_.grid) drawGrid();
    if (layers_.markers) drawMarkers();
    if (layers_.skeleton) drawSkeleton();

    vao_.release();
    shader_->release();

    update();  // continuous repaint
}

void MoCapCanvas::drawGrid() {
    float extent = 5.0f;
    float step = 0.5f;

    // Grid lines (dark gray)
    QColor gridColor(80, 80, 80, 120);
    for (float i = -extent; i <= extent; i += step) {
        drawLine(QVector3D(i, 0, -extent), QVector3D(i, 0, extent), gridColor);
        drawLine(QVector3D(-extent, 0, i), QVector3D(extent, 0, i), gridColor);
    }

    // Axis lines
    drawLine(QVector3D(0, 0, 0), QVector3D(extent, 0, 0), QColor(220, 50, 50), 2.0f);   // X red
    drawLine(QVector3D(0, 0, 0), QVector3D(0, extent, 0), QColor(50, 220, 50), 2.0f);   // Y green
    drawLine(QVector3D(0, 0, 0), QVector3D(0, 0, extent), QColor(50, 50, 220), 2.0f);   // Z blue
}

void MoCapCanvas::drawMarkers() {
    for (const auto& pose : current_poses_) {
        QColor color = colorForPerson(pose.global_person_id);

        for (const auto& m : pose.markers) {
            int alpha = static_cast<int>(std::clamp(m.confidence, 0.2f, 1.0f) * 255);
            QColor mc = color;
            mc.setAlpha(alpha);

            float size = 4.0f + m.confidence * 4.0f;
            QVector3D pos(m.position.x(), m.position.y(), m.position.z());
            drawPoint(pos, mc, size);
        }
    }
}

void MoCapCanvas::drawSkeleton() {
    // BODY_25 connections
    static const std::vector<std::pair<int, int>> bones = {
        {0,1},{1,2},{2,3},{3,4},{1,5},{5,6},{6,7},
        {1,8},{8,9},{9,10},{10,11},{8,12},{12,13},{13,14},
        {0,15},{0,16},{15,17},{16,18},
        {14,19},{14,20},{14,21},{11,22},{11,23},{11,24}
    };

    for (const auto& pose : current_poses_) {
        QColor color = colorForPerson(pose.global_person_id);

        // Build index -> position map
        std::unordered_map<int, QVector3D> positions;
        for (const auto& m : pose.markers) {
            if (m.confidence > 0.1f) {
                positions[m.index] = QVector3D(m.position.x(), m.position.y(), m.position.z());
            }
        }

        // Draw bones
        for (const auto& [a, b] : bones) {
            auto itA = positions.find(a);
            auto itB = positions.find(b);
            if (itA == positions.end() || itB == positions.end()) continue;
            drawLine(itA->second, itB->second, color, 2.0f);
        }

        // Draw joint spheres
        for (const auto& [idx, pos] : positions) {
            bool is_selected = (pose.global_person_id == selected_person_);
            float size = is_selected ? 8.0f : 5.0f;
            QColor jc = is_selected ? QColor(255, 255, 255) : color;
            drawPoint(pos, jc, size);
        }
    }
}

void MoCapCanvas::drawLine(const QVector3D& from, const QVector3D& to,
                           const QColor& color, float width) {
    glLineWidth(width);
    setUniformColor(color);

    std::vector<float> verts = {
        from.x(), from.y(), from.z(),
        to.x(), to.y(), to.z()
    };
    uploadAndDraw(verts, GL_LINES);
}

void MoCapCanvas::drawPoint(const QVector3D& pos, const QColor& color, float size) {
    shader_->setUniformValue("uPointSize", size);
    setUniformColor(color);

    std::vector<float> verts = {pos.x(), pos.y(), pos.z()};
    uploadAndDraw(verts, GL_POINTS);
}

void MoCapCanvas::setUniformColor(const QColor& color) {
    shader_->setUniformValue("uColor", QVector4D(
        color.redF(), color.greenF(), color.blueF(), color.alphaF()));
}

void MoCapCanvas::uploadAndDraw(const std::vector<float>& vertices, GLenum mode) {
    vbo_.bind();
    vbo_.allocate(vertices.data(), static_cast<int>(vertices.size() * sizeof(float)));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

    glDrawArrays(mode, 0, static_cast<int>(vertices.size() / 3));

    glDisableVertexAttribArray(0);
    vbo_.release();
}

QColor MoCapCanvas::colorForPerson(int global_person_id) const {
    // Oklab-inspired hue rotation for perceptually uniform colors
    static const QColor palette[] = {
        QColor(0, 200, 120),    // teal-green
        QColor(255, 100, 50),   // orange
        QColor(100, 80, 255),   // violet
        QColor(255, 220, 0),    // yellow
        QColor(0, 180, 255),    // sky blue
        QColor(255, 50, 150),   // pink
        QColor(0, 255, 200),    // cyan
        QColor(200, 130, 0),    // amber
        QColor(150, 0, 255),    // purple
        QColor(100, 255, 0),    // lime
        QColor(255, 0, 80),     // red
        QColor(0, 130, 200),    // blue
    };
    return palette[global_person_id % 12];
}

// --- Slots ---

void MoCapCanvas::onPose3DUpdate(const std::vector<Pose3D>& poses) {
    current_poses_ = poses;
}

void MoCapCanvas::onSkeletonUpdate(const std::vector<SkeletonPose>& skeletons) {
    current_skeletons_ = skeletons;
}

void MoCapCanvas::setRenderLayerVisible(const QString& layer, bool visible) {
    if (layer == "grid") layers_.grid = visible;
    else if (layer == "markers") layers_.markers = visible;
    else if (layer == "skeleton") layers_.skeleton = visible;
    else if (layer == "trails") layers_.trails = visible;
}

void MoCapCanvas::frameSelection() {
    if (selected_person_ < 0 || current_poses_.empty()) {
        camera_.target = QVector3D(0, 1, 0);
        camera_.distance = 5.0f;
        return;
    }

    for (const auto& pose : current_poses_) {
        if (pose.global_person_id == selected_person_ && !pose.markers.empty()) {
            QVector3D center(0, 0, 0);
            for (const auto& m : pose.markers) {
                center += QVector3D(m.position.x(), m.position.y(), m.position.z());
            }
            center /= static_cast<float>(pose.markers.size());
            camera_.target = center;
            camera_.distance = 3.0f;
            break;
        }
    }
}

// --- Mouse interaction ---

void MoCapCanvas::mousePressEvent(QMouseEvent* event) {
    last_mouse_pos_ = event->pos();

    if (event->button() == Qt::LeftButton) {
        orbiting_ = true;
    } else if (event->button() == Qt::RightButton) {
        panning_ = true;
    } else if (event->button() == Qt::MiddleButton) {
        // Pick nearest person
        // Simplified: cycle through persons
        if (!current_poses_.empty()) {
            selected_person_++;
            bool found = false;
            for (const auto& p : current_poses_) {
                if (p.global_person_id >= selected_person_) {
                    selected_person_ = p.global_person_id;
                    found = true;
                    break;
                }
            }
            if (!found) selected_person_ = -1;
            emit personSelected(selected_person_);
        }
    }
}

void MoCapCanvas::mouseMoveEvent(QMouseEvent* event) {
    QPoint delta = event->pos() - last_mouse_pos_;
    last_mouse_pos_ = event->pos();

    if (orbiting_) {
        camera_.azimuth -= delta.x() * 0.5f;
        camera_.elevation += delta.y() * 0.5f;
        camera_.elevation = std::clamp(camera_.elevation, -89.0f, 89.0f);
    } else if (panning_) {
        float scale = camera_.distance * 0.002f;
        QMatrix4x4 view = camera_.viewMatrix();
        QVector3D right(view(0, 0), view(1, 0), view(2, 0));
        QVector3D up(view(0, 1), view(1, 1), view(2, 1));
        camera_.target -= right * delta.x() * scale;
        camera_.target += up * delta.y() * scale;
    }
}

void MoCapCanvas::mouseReleaseEvent(QMouseEvent* /*event*/) {
    orbiting_ = false;
    panning_ = false;
}

void MoCapCanvas::wheelEvent(QWheelEvent* event) {
    float delta = event->angleDelta().y() / 120.0f;
    camera_.distance *= (1.0f - delta * 0.1f);
    camera_.distance = std::clamp(camera_.distance, 0.5f, 50.0f);
}

void MoCapCanvas::keyPressEvent(QKeyEvent* event) {
    switch (event->key()) {
        case Qt::Key_F:
            frameSelection();
            break;
        case Qt::Key_1:
            layers_.grid = !layers_.grid;
            break;
        case Qt::Key_2:
            layers_.markers = !layers_.markers;
            break;
        case Qt::Key_3:
            layers_.skeleton = !layers_.skeleton;
            break;
        default:
            QOpenGLWidget::keyPressEvent(event);
    }
}

}  // namespace mocap
