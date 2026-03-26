#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QTimer>

// GlWidget renders a coloured rotating cube driven by an IMU quaternion.
// Uses modern OpenGL 3.3 core profile via QOpenGLFunctions.
class GlWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit GlWidget(QWidget* parent = nullptr);
    ~GlWidget() override;

public slots:
    void updateQuaternion(double w, double x, double y, double z);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    void buildCubeGeometry();
    void buildAxesGeometry();

    QOpenGLShaderProgram m_cubeShader;
    QOpenGLShaderProgram m_axisShader;

    QOpenGLVertexArrayObject m_cubeVao;
    QOpenGLBuffer            m_cubeVbo;

    QOpenGLVertexArrayObject m_axisVao;
    QOpenGLBuffer            m_axisVbo;

    QMatrix4x4  m_projection;
    QQuaternion m_quat{1.0f, 0.0f, 0.0f, 0.0f};  // identity
};
