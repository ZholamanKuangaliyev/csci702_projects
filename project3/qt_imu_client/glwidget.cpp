#include "glwidget.h"
#include <QSurfaceFormat>
#include <cstring>

// ── Vertex layout: 3 floats position + 3 floats colour ── stride = 6 floats
// ── Cube: 6 faces × 2 triangles × 3 vertices = 36 vertices ──────────────────
static const GLfloat k_cube[] = {
    // Front  (z+)  red
    -0.5f,-0.5f, 0.5f,  0.90f,0.20f,0.20f,
     0.5f,-0.5f, 0.5f,  0.90f,0.20f,0.20f,
     0.5f, 0.5f, 0.5f,  0.90f,0.20f,0.20f,
    -0.5f,-0.5f, 0.5f,  0.90f,0.20f,0.20f,
     0.5f, 0.5f, 0.5f,  0.90f,0.20f,0.20f,
    -0.5f, 0.5f, 0.5f,  0.90f,0.20f,0.20f,
    // Back   (z-)  green
    -0.5f,-0.5f,-0.5f,  0.20f,0.80f,0.20f,
     0.5f, 0.5f,-0.5f,  0.20f,0.80f,0.20f,
     0.5f,-0.5f,-0.5f,  0.20f,0.80f,0.20f,
    -0.5f,-0.5f,-0.5f,  0.20f,0.80f,0.20f,
    -0.5f, 0.5f,-0.5f,  0.20f,0.80f,0.20f,
     0.5f, 0.5f,-0.5f,  0.20f,0.80f,0.20f,
    // Left   (x-)  blue
    -0.5f,-0.5f,-0.5f,  0.20f,0.40f,0.90f,
    -0.5f,-0.5f, 0.5f,  0.20f,0.40f,0.90f,
    -0.5f, 0.5f, 0.5f,  0.20f,0.40f,0.90f,
    -0.5f,-0.5f,-0.5f,  0.20f,0.40f,0.90f,
    -0.5f, 0.5f, 0.5f,  0.20f,0.40f,0.90f,
    -0.5f, 0.5f,-0.5f,  0.20f,0.40f,0.90f,
    // Right  (x+)  yellow
     0.5f,-0.5f,-0.5f,  0.95f,0.85f,0.05f,
     0.5f, 0.5f, 0.5f,  0.95f,0.85f,0.05f,
     0.5f,-0.5f, 0.5f,  0.95f,0.85f,0.05f,
     0.5f,-0.5f,-0.5f,  0.95f,0.85f,0.05f,
     0.5f, 0.5f,-0.5f,  0.95f,0.85f,0.05f,
     0.5f, 0.5f, 0.5f,  0.95f,0.85f,0.05f,
    // Top    (y+)  cyan
    -0.5f, 0.5f,-0.5f,  0.10f,0.85f,0.85f,
    -0.5f, 0.5f, 0.5f,  0.10f,0.85f,0.85f,
     0.5f, 0.5f, 0.5f,  0.10f,0.85f,0.85f,
    -0.5f, 0.5f,-0.5f,  0.10f,0.85f,0.85f,
     0.5f, 0.5f, 0.5f,  0.10f,0.85f,0.85f,
     0.5f, 0.5f,-0.5f,  0.10f,0.85f,0.85f,
    // Bottom (y-)  magenta
    -0.5f,-0.5f,-0.5f,  0.85f,0.10f,0.85f,
     0.5f,-0.5f, 0.5f,  0.85f,0.10f,0.85f,
    -0.5f,-0.5f, 0.5f,  0.85f,0.10f,0.85f,
    -0.5f,-0.5f,-0.5f,  0.85f,0.10f,0.85f,
     0.5f,-0.5f,-0.5f,  0.85f,0.10f,0.85f,
     0.5f,-0.5f, 0.5f,  0.85f,0.10f,0.85f,
};

// ── Axes: 3 lines, each 2 vertices (origin → tip) ────────────────────────────
static const GLfloat k_axes[] = {
    0.0f,0.0f,0.0f, 1.0f,0.0f,0.0f,   // X start – red
    1.1f,0.0f,0.0f, 1.0f,0.0f,0.0f,   // X end
    0.0f,0.0f,0.0f, 0.0f,1.0f,0.0f,   // Y start – green
    0.0f,1.1f,0.0f, 0.0f,1.0f,0.0f,   // Y end
    0.0f,0.0f,0.0f, 0.0f,0.5f,1.0f,   // Z start – blue
    0.0f,0.0f,1.1f, 0.0f,0.5f,1.0f,   // Z end
};

// ── GLSL shaders (GLSL 1.30 / OpenGL 3.0, widely supported) ─────────────────
static const char* k_vs = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
out vec3 vColor;
uniform mat4 uMVP;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vColor = aColor;
}
)";

static const char* k_fs = R"(
#version 330 core
in  vec3 vColor;
out vec4 fragColor;
void main() {
    fragColor = vec4(vColor, 1.0);
}
)";

// ── Constructor / destructor ──────────────────────────────────────────────────
GlWidget::GlWidget(QWidget* parent)
    : QOpenGLWidget(parent),
      m_cubeVbo(QOpenGLBuffer::VertexBuffer),
      m_axisVbo(QOpenGLBuffer::VertexBuffer)
{
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    setFormat(fmt);
    setMinimumSize(480, 480);
}

GlWidget::~GlWidget() {
    makeCurrent();
    m_cubeVao.destroy(); m_cubeVbo.destroy();
    m_axisVao.destroy(); m_axisVbo.destroy();
    doneCurrent();
}

// ── Public slot ──────────────────────────────────────────────────────────────
void GlWidget::updateQuaternion(double w, double x, double y, double z) {
    // QQuaternion(scalar, x, y, z)
    m_quat = QQuaternion(static_cast<float>(w),
                         static_cast<float>(x),
                         static_cast<float>(y),
                         static_cast<float>(z)).normalized();
    update();   // schedule repaint
}

// ── OpenGL lifecycle ──────────────────────────────────────────────────────────
void GlWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.08f, 0.08f, 0.12f, 1.0f);

    // Compile shaders
    m_cubeShader.addShaderFromSourceCode(QOpenGLShader::Vertex,   k_vs);
    m_cubeShader.addShaderFromSourceCode(QOpenGLShader::Fragment, k_fs);
    m_cubeShader.link();

    m_axisShader.addShaderFromSourceCode(QOpenGLShader::Vertex,   k_vs);
    m_axisShader.addShaderFromSourceCode(QOpenGLShader::Fragment, k_fs);
    m_axisShader.link();

    buildCubeGeometry();
    buildAxesGeometry();
}

void GlWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    m_projection.setToIdentity();
    m_projection.perspective(60.0f, (h > 0) ? float(w)/float(h) : 1.0f,
                              0.1f, 100.0f);
}

void GlWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // View matrix: camera at (0,0,3) looking at origin
    QMatrix4x4 view;
    view.lookAt({0, 0, 3}, {0, 0, 0}, {0, 1, 0});

    // Model matrix: rotate by received quaternion
    QMatrix4x4 model;
    model.rotate(m_quat);

    QMatrix4x4 mvp = m_projection * view * model;

    // ── Draw cube ────────────────────────────────────────────────────────────
    m_cubeShader.bind();
    m_cubeShader.setUniformValue("uMVP", mvp);
    m_cubeVao.bind();
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Wire overlay (draw same VBO as lines, slightly scale to avoid z-fight)
    QMatrix4x4 wireModel;
    wireModel.scale(1.002f);
    wireModel.rotate(m_quat);
    QMatrix4x4 wireMVP = m_projection * view * wireModel;
    m_cubeShader.setUniformValue("uMVP", wireMVP);

    // Draw each face boundary as LINE_LOOP (6 faces × 4 vertices)
    // Our VBO has 36 vertices in triangle layout → use LINE_STRIP per face pair
    glLineWidth(1.5f);
    for (int face = 0; face < 6; ++face) {
        // Each face is 6 vertices (2 triangles). Draw as 4 unique edges.
        // Simplest: just use the 6 verts as a LINE_STRIP (shows diagonals too)
        // For a clean look, draw as wireframe triangles:
        glDrawArrays(GL_LINE_LOOP, face * 6, 3);
        glDrawArrays(GL_LINE_LOOP, face * 6 + 3, 3);
    }
    m_cubeVao.release();
    m_cubeShader.release();

    // ── Draw world-space coordinate axes (no model rotation) ─────────────────
    QMatrix4x4 axisMVP = m_projection * view;  // no model = world frame
    m_axisShader.bind();
    m_axisShader.setUniformValue("uMVP", axisMVP);
    m_axisVao.bind();
    glLineWidth(2.5f);
    glDrawArrays(GL_LINES, 0, 6);
    m_axisVao.release();
    m_axisShader.release();
}

// ── Geometry builders ─────────────────────────────────────────────────────────
// Helper: bind VBO + configure position (loc=0) and colour (loc=1) attributes
static void setupAttribs(QOpenGLShaderProgram& shader, int stride) {
    shader.bind();
    shader.enableAttributeArray(0);
    shader.setAttributeBuffer(0, GL_FLOAT, 0,                 3, stride);
    shader.enableAttributeArray(1);
    shader.setAttributeBuffer(1, GL_FLOAT, 3*sizeof(GLfloat), 3, stride);
    shader.release();
}

void GlWidget::buildCubeGeometry() {
    m_cubeVao.create();
    m_cubeVao.bind();
    m_cubeVbo.create();
    m_cubeVbo.bind();
    m_cubeVbo.allocate(k_cube, sizeof(k_cube));
    setupAttribs(m_cubeShader, 6 * sizeof(GLfloat));
    m_cubeVbo.release();
    m_cubeVao.release();
}

void GlWidget::buildAxesGeometry() {
    m_axisVao.create();
    m_axisVao.bind();
    m_axisVbo.create();
    m_axisVbo.bind();
    m_axisVbo.allocate(k_axes, sizeof(k_axes));
    setupAttribs(m_axisShader, 6 * sizeof(GLfloat));
    m_axisVbo.release();
    m_axisVao.release();
}
