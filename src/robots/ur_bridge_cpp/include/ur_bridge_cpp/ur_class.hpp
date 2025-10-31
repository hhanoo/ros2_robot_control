#ifndef UR_CLASS_HPP
#define UR_CLASS_HPP

#include <arpa/inet.h>
#include <netinet/in.h>  // for sockaddr_in
#include <unistd.h>      // for close()

#include <Eigen/Dense>  // for matrix operations (행렬 연산용)
#include <array>
#include <atomic>
#include <cstring>   // for memcpy (바이트 변환용)
#include <iostream>  // for console output (콘솔 출력용)
#include <mutex>
#include <thread>  // for std::thread (쓰레드용)
#include <vector>

using namespace std;

class URClass {
   public:
    URClass(const string& ip = "127.0.0.1");
    ~URClass();

    bool connect();
    bool disconnect();
    bool isConnected() const;

    void moveJ(const array<double, 6>& des_q, double acc = 0, double vel = 0, double duration = 0, const string& mode = "abs");
    void moveL(const array<double, 6>& des_X, double acc = 0, double vel = 0, double duration = 0, const string& mode = "abs");

    void stopJ(double acc = 0);
    void stopL(double acc = 0);

    void controlboxDigitalOut(int port, bool value);
    bool controlboxDigitalIn(int port) const;

    void setVelocity(double vel);
    void waitMove(double timeout = 10.0);

    void printRobotMode();

   private:
    // Connection setup
    string       UR_IP;
    int          UR_PORT   = 30003;
    atomic<bool> connected = false;

    // Socket setup
    int       sockfd      = -1;
    const int buffer_size = 1220;

    // Robot state variables
    array<double, 6> act_q, des_q, act_X, des_X;
    Eigen::Matrix4f  act_T, des_T;

    // Orientation (rotation) parameters
    Eigen::Matrix3f R;

    // Force/Torque sensing
    array<double, 6> act_F;

    // Digital I/O
    array<bool, 8> digital_input;

    // Start background status thread
    atomic<bool> send_move_flag = false;

    // Thread setup
    thread update_thread;
    void   updateInfo();
    void   robotInfo();

    Eigen::Matrix3f  rotationVectorToMatrix(double rx, double ry, double rz);
    array<double, 3> rotationMatrixToVector(const Eigen::Matrix3f& R);

    mutex data_mutex;

    double velocity_percentage;
};

#endif  // UR_CLASS_HPP
