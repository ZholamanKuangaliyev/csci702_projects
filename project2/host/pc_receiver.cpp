#include <GL/glut.h>
#include <iostream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cmath>
#include <array>

#define PORT 8080

struct Quat { double w{1}, x{0}, y{0}, z{0}; };
static Quat g_q = {1, 0, 0, 0};
int sockfd;

static Quat normalize(const Quat& q) {
    double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n <= 0) return {1,0,0,0};
    return {q.w/n, q.x/n, q.y/n, q.z/n};
}

static std::array<float, 16> quatToMat4(const Quat& q_in) {
    Quat q = normalize(q_in);
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    const double xx = x*x, yy = y*y, zz = z*z;
    const double xy = x*y, xz = x*z, yz = y*z;
    const double wx = w*x, wy = w*y, wz = w*z;

    double r00 = 1.0 - 2.0*(yy + zz), r01 = 2.0*(xy - wz), r02 = 2.0*(xz + wy);
    double r10 = 2.0*(xy + wz),       r11 = 1.0 - 2.0*(xx + zz), r12 = 2.0*(yz - wx);
    double r20 = 2.0*(xz - wy),       r21 = 2.0*(yz + wx),       r22 = 1.0 - 2.0*(xx + yy);

    std::array<float,16> m{};
    m[0] = r00; m[4] = r01; m[8]  = r02; m[12] = 0.0f;
    m[1] = r10; m[5] = r11; m[9]  = r12; m[13] = 0.0f;
    m[2] = r20; m[6] = r21; m[10] = r22; m[14] = 0.0f;
    m[3] = 0.0f; m[7] = 0.0f; m[11] = 0.0f; m[15] = 1.0f;
    return m;
}

void initUDP() {
    struct sockaddr_in servaddr;
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed\n"; exit(EXIT_FAILURE);
    }
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Bind failed\n"; exit(EXIT_FAILURE);
    }
    fcntl(sockfd, F_SETFL, O_NONBLOCK); // Non-blocking socket
    std::cout << "Listening for IMU data on port " << PORT << "...\n";
}

static void drawAxes(float len=1.2f) {
    glLineWidth(2.0f); glBegin(GL_LINES);
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(len,0,0); 
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,len,0); 
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,len); 
    glEnd();
}

static void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    gluLookAt(0, 0, 3.0, 0, 0, 0, 0, 1, 0);
    drawAxes();

    auto M = quatToMat4(g_q);
    glPushMatrix(); glMultMatrixf(M.data()); drawAxes(0.9f);
    
    glEnable(GL_POLYGON_OFFSET_FILL); glPolygonOffset(1.0f, 1.0f);
    glColor3f(0.7f, 0.7f, 0.9f); glutSolidCube(1.0f);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glColor3f(1,1,1); glutWireCube(1.01f);
    
    glPopMatrix(); glutSwapBuffers();
}

static void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluPerspective(60.0, (h>0)? (double)w/(double)h : 1.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

static void idle() {
    double packet[4];
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);
    int n = recvfrom(sockfd, (char *)packet, sizeof(packet), MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);

    if (n == sizeof(packet)) {
        g_q.w = packet[0]; g_q.x = packet[1]; g_q.y = packet[2]; g_q.z = packet[3];
        glutPostRedisplay();
    }
}

static void keyboard(unsigned char key, int, int) {
    if (key == 27 || key == 'q') { close(sockfd); std::exit(0); }
}

int main(int argc, char** argv) {
    initUDP();
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(900, 600);
    glutCreateWindow("Host PC: UDP Streamed IMU Visualization");

    glEnable(GL_DEPTH_TEST); glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
    
    glutDisplayFunc(display); glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard); glutIdleFunc(idle);

    glutMainLoop();
    return 0;
}