#include <iostream>
#include <fstream>
#include <pigpiod_if2.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <thread>
#include <string>
#include <vector>
#include <array>
#include <asio/ip/udp.hpp>
#include <asio/ip/tcp.hpp>
#include <asio/io_service.hpp>
#include <queue>
#include "main.h"
#include "navigation.h"

using namespace std;
using namespace asio::ip;

chrono::system_clock::time_point lastServoMove;
bool servoActive = false;
int pigpioID;
bool navigating = true;
bool running = true;
bool streaming = false;
bool needToTakePicture = false;
cv::Mat frame;
cv::Mat frameToSend;

cv::VideoCapture camera;
asio::io_service asio_service;
udp::socket udpSocket(asio_service);
udp::endpoint remote_endpoint;

double fps = 0;
chrono::high_resolution_clock::duration frameInterval;
chrono::high_resolution_clock::duration frameTime;
chrono::high_resolution_clock::duration waitTime;

void setAngle(int angle)
{
    lastServoMove = chrono::system_clock::now();
    servoActive = true;
    set_PWM_dutycycle(pigpioID, 24, 650 + (angle * 2000) / 180);
}

void servoDisableLoop()
{
    while (running)
    {
        auto now = chrono::system_clock::now();
        if (servoActive && (lastServoMove + 1s) < now)
        {
            servoActive = false;
            set_PWM_dutycycle(pigpioID, 24, 0);
        }
        this_thread::sleep_for(0.1s);
    }
}

void sendFrame(cv::Mat jpegFrame){
    array<uchar, 5> pattern = {0xFF, 0xD9, 0x00, 0x00, 0x00};
    const uchar *imageEnd = std::search(jpegFrame.datastart, jpegFrame.dataend, pattern.begin(), pattern.end()) + 2;
    int packetSize = min(imageEnd - jpegFrame.datastart, 65507);
    udpSocket.send_to(asio::buffer(jpegFrame.datastart, packetSize), remote_endpoint);
}

void saveImage(cv::Mat jpegFrame){
    array<uchar, 5> pattern = {0xFF, 0xD9, 0x00, 0x00, 0x00};
    const uchar *imageEnd = std::search(jpegFrame.datastart, jpegFrame.dataend, pattern.begin(), pattern.end()) + 2;
    ofstream fout;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream str;
    str << std::put_time(&tm, "/media/usb/img%d.%m.%Y_%H-%M-%S.jpg");
    fout.open(str.str().c_str(), ios::binary | ios::out);
    fout.write((const char *)jpegFrame.datastart, (streamsize)(imageEnd - jpegFrame.datastart));
    fout.close();
}

void cameraLoop()
{
    while (running)
    {
        chrono::high_resolution_clock::time_point waitStartTime = chrono::high_resolution_clock::now();
        camera.grab();
        chrono::high_resolution_clock::time_point frameStartTime = chrono::high_resolution_clock::now();
        camera.retrieve(frame);

        if(navigating){
            detectPosition(frame);
        }

        frame.copyTo(frameToSend);

        chrono::high_resolution_clock::time_point frameStopTime = chrono::high_resolution_clock::now();
        frameInterval = frameStopTime-waitStartTime;
        frameTime = frameStopTime-frameStartTime;
        waitTime = frameStartTime-waitStartTime;
        fps = 1000000.0/chrono::duration_cast<chrono::microseconds>(frameInterval).count();
        cout << "fps: " << fps << "\t";
        cout << "load: " << 100.0*(double)frameTime.count()/(double)frameInterval.count() << endl;
    }
}

void streamLoop(){
    while (true){
        while(!streaming){
            this_thread::sleep_for(100ms);
        }
        //TODO: use hardware jpeg encoder
        vector<uchar> buffer;
        vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(80);
        cv::imencode(".jpg",frameToSend,buffer,params);
        cv::Mat frameJpeg(buffer);

        sendFrame(frameJpeg);
    
        if (needToTakePicture)
        {
            needToTakePicture = false;
            saveImage(frameJpeg);
        }
    }
}

void networkLoop()
{
    tcp::endpoint localEndpoint(asio::ip::tcp::v4(), 1234);
    tcp::acceptor acceptor(asio_service, localEndpoint);
    while (running)
    {
        tcp::socket socket = acceptor.accept();
        bool connected = true;
        while (connected)
        {
            uchar command;

            try
            {
                socket.receive(asio::buffer(&command, sizeof(uchar)));
            }
            catch (exception e)
            {
                streaming = false;
                connected = false;
                socket.close();
                break;
            }

            switch (command)
            {
            case 20: // start streaming
            {
                remote_endpoint = udp::endpoint(socket.remote_endpoint().address(), 1234);
                streaming = true;
                break;
            }
            case 21: // stop streaming
                streaming = false;
                break;
            case 22: // move camera
            {
                uchar a;
                socket.receive(asio::buffer(&a, sizeof(uchar)));
                setAngle((int)a);
                break;
            }
            case 23: // start recording
                cout << "recording...\n";
                break;
            case 24: // stop recording
                cout << "stopped recording...\n";
                break;
            case 25: // take picture
                needToTakePicture = true;
                cout << "took picture\n";
                break;
            case 26: // take HD picture
                cout << "took HD picture\n";
                break;
            case 27: // disconnect
                streaming = false;
                connected = false;
                socket.close();
                break;
            }
        }
    }
}

int main()
{
    navigationInit();

    //servo init
    pigpioID = pigpio_start(NULL, NULL);
    set_mode(pigpioID, 24, PI_OUTPUT);
    set_PWM_frequency(pigpioID, 24, 50);
    set_PWM_range(pigpioID, 24, 20000); // 1,000,000 / 50 = 20,000us for 100% duty cycle
    setAngle(135);
    thread servoPowerSaveThread(servoDisableLoop);

    //camera init
    camera = cv::VideoCapture(0, cv::CAP_V4L2);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    char format[] = "BGR3";
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(format[0],format[1],format[2],format[3]));
    camera.set(cv::CAP_PROP_FPS, 30);
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    camera.set(cv::CAP_PROP_EXPOSURE, 100);
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
    camera.set(cv::CAP_PROP_CONVERT_RGB, 0);
    thread cameraThread(cameraLoop);

    //network init
    udpSocket.open(udp::v4());
    udpSocket.bind(udp::endpoint(address_v4({0, 0, 0, 0}), 1234));
    thread networkThread(networkLoop);
    thread streamThread(streamLoop);

    while (running)
    {
        this_thread::sleep_for(0.1s);
    }

    stop();
    return 0;
}

void stop()
{
    running = false;
    this_thread::sleep_for(1s);
    udpSocket.close();
    set_PWM_dutycycle(pigpioID, 24, 0);
    pigpio_stop(pigpioID);
}