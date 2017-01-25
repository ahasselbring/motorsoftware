/*
 * This file has been extended from the original version of Motor001.cpp
 * as provided by the Institute of Control Systems at the Hamburg University of Technology.
 * The primary goal was to implement Linux compatibility. There could potentially be macOS
 * support, too, but this is not tested. I also did some code cleanup and formatting.
 * Arne Hasselbring <arne.hasselbring@googlemail.com>
 */

#include <cstdio>
#include <iostream>
#include <stdexcept>

#ifdef _WIN32

#include <windows.h>

#elif (defined __linux__) || (defined __APPLE__)

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#ifdef __APPLE__
#include <sys/ioctl.h>
#endif

#else
#error "Neither _WIN32 nor __linux__ nor __APPLE__ are defined."
#endif

/**
 * @class MotorConnection encapsulates sending and receiving characters to/from the arduino on the motor board.
 */
class MotorConnection {
public:
    /**
     * @brief MotorConnection opens a port to the arduino on the motor board and sets it up
     * @param portNumber an OS-specific number identifying the port that the arduino is connected to
     * @param baudrate the baudrate of the connection
     */
    MotorConnection(int portNumber, int baudrate)
    {
        char portname[30];
#ifdef _WIN32
        snprintf(portname, 30, "COM%d", portNumber);
#elif defined __APPLE__
        snprintf(portname, 30, "/dev/tty.usbmodem%d", portNumber);
#else
        snprintf(portname, 30, "/dev/ttyACM%d", portNumber);
#endif

#ifdef _WIN32
        hSerial_ = CreateFileA(portname, GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (hSerial_ == INVALID_HANDLE_VALUE) {
            std::stringstream ss;
            ss << "No device is connected to port " << portNumber << ". You probably entered a wrong port number for the arduino. Go to your PC's device manager to find the arduino's port number. Double click on the USB block in the Simulink and change the port number.";
            throw std::runtime_error(ss.str().c_str());
        }

        DCB dcbSerialParams = { 0 };
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        if (!GetCommState(hSerial_, &dcbSerialParams)) {
            CloseHandle(hSerial_);
            hSerial_ = INVALID_HANDLE_VALUE;
            throw std::runtime_error("GetCommState failed.");
        }
        dcbSerialParams.BaudRate = baudrate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
        if (!SetCommState(hSerial_, &dcbSerialParams)) {
            CloseHandle(hSerial_);
            hSerial_ = INVALID_HANDLE_VALUE;
            throw std::runtime_error("SetCommState failed.");
        }

        COMMTIMEOUTS timeouts = { 0 };
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        if (!SetCommTimeouts(hSerial_, &timeouts)) {
            CloseHandle(hSerial_);
            hSerial_ = INVALID_HANDLE_VALUE;
            throw std::runtime_error("SetCommTimeouts failed.");
        }

        if (!SetCommMask(hSerial_, EV_RING)) {
            CloseHandle(hSerial_);
            hSerial_ = INVALID_HANDLE_VALUE;
            throw std::runtime_error("SetCommMask failed.");
        }

        if (!PurgeComm(hSerial_, PURGE_RXCLEAR | PURGE_TXCLEAR)) {
            CloseHandle(hSerial_);
            hSerial_ = INVALID_HANDLE_VALUE;
            throw std::runtime_error("PurgeComm failed.");
        }
#else
#ifdef __APPLE__
        fd_ = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
#else
        fd_ = open(portname, O_RDWR | O_NOCTTY);
#endif
        if (fd_ < 0) {
            throw std::runtime_error("Could not open tty.");
        }

#ifdef __APPLE__
        if (ioctl(fd_, TIOCEXCL) < 0) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("Could not get exlusive access to the tty.");
        }
        if (fcntl(fd_, F_SETFL, 0) < 0) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("Could not reset file flags.");
        }
#endif

        termios attrs;
        if (tcgetattr(fd_, &attrs) < 0) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("Could not get tty attributes.");
        }
        switch (baudrate) {
            case 110:
                attrs.c_cflag = B110;
                break;
            case 300:
                attrs.c_cflag = B300;
                break;
            case 600:
                attrs.c_cflag = B600;
                break;
            case 1200:
                attrs.c_cflag = B1200;
                break;
            case 2400:
                attrs.c_cflag = B2400;
                break;
            case 4800:
                attrs.c_cflag = B4800;
                break;
            case 9600:
                attrs.c_cflag = B9600;
                break;
            case 19200:
                attrs.c_cflag = B19200;
                break;
            case 38400:
                attrs.c_cflag = B38400;
                break;
            case 57600:
                attrs.c_cflag = B57600;
                break;
            case 115200:
                attrs.c_cflag = B115200;
                break;
            default:
                close(fd_);
                fd_ = -1;
                throw std::runtime_error("The baudrate you want to have is not possible.");
        }
        attrs.c_cflag |= CS8 | CREAD | CLOCAL;
        attrs.c_lflag = 0;
        attrs.c_oflag = 0;
        attrs.c_iflag = 0;
        attrs.c_cc[VMIN] = 0;
        attrs.c_cc[VTIME] = 0;
        if (tcflush(fd_, TCIOFLUSH) < 0) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("Could not flush tty.");
        }
        if (tcsetattr(fd_, TCSANOW, &attrs) < 0) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("Could not set tty attributes.");
        }
#endif
    }
    /**
     * @brief ~MotorConnection closes the port to the arduino
     */
    ~MotorConnection()
    {
#ifdef _WIN32
        if (hSerial_ != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial_);
            hSerial_ = INVALID_HANDLE_VALUE;
        }
#else
        if (fd_ >= 0) {
            tcflush(fd_, TCIOFLUSH);
            close(fd_);
            fd_ = -1;
        }
#endif
    }
    /**
     * @brief send sends a single character to the arduino
     * @param word the character that is sent
     */
    void send(const unsigned char word)
    {
#ifdef _WIN32
        DWORD n;
        WriteFile(hSerial_, &word, 1, &n, nullptr);
#else
        write(fd_, &word, 1);
#endif
    }
    /**
     * @brief recv receives up to n characters from the arduino
     * @param buf the buffer where the incoming characters are stored
     * @param n the number of characters that are in the buffer
     */
    void recv(void *buf, unsigned int n)
    {
#ifdef _WIN32
        DWORD n2;
        ReadFile(hSerial_, buf, n, &n2, nullptr);
#else
        read(fd_, buf, n);
#endif
    }
private:
#ifdef _WIN32
    /// the handle to the arduino port
    HANDLE hSerial_ = INVALID_HANDLE_VALUE;
#else
    /// the file descriptor of the arduino port
    int fd_ = -1;
#endif
};

#define S_FUNCTION_NAME  Motor001
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"


#define SPEED_CONTROL 1
#define POSITION_CONTROL 0

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 4);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    ssSetInputPortRequiredContiguous(S, 1, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDataType(S, 0, SS_UINT8);
    ssSetInputPortDataType(S, 1, SS_UINT8);

    if (!ssSetNumOutputPorts(S, 3)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);

    ssSetOutputPortWidth(S, 2, 1);
    ssSetOutputPortDataType(S, 2, SS_UINT8);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 2); // to save operation_mode and first step
    ssSetNumPWork(S, 1); // reserve element in the pointers vector to store a C++ object
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, 0)));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    MotorConnection *c = nullptr;
    try {
        c = new MotorConnection(static_cast<int>(mxGetScalar(ssGetSFcnParam(S, 1))), static_cast<int>(mxGetScalar(ssGetSFcnParam(S, 2))));
    } catch (const std::exception& e) {
        static char str[300];
        strncpy(str, e.what(), 300);
        str[299] = '\0';
        ssSetErrorStatus(S, str);
        return;
    }
    ssGetPWork(S)[0] = static_cast<void*>(c);

    char_T buf[10];
    mxGetString(ssGetSFcnParam(S, 3), buf, 10);
    if (strncmp(buf, "Velocity", 10) == 0) {
          ssGetIWork(S)[0] = SPEED_CONTROL;
          c->send(0x83); //0b1000 0011
    } else if (strncmp(buf, "Angle", 10) == 0) {
          ssGetIWork(S)[0] = POSITION_CONTROL;
          c->send(0x81); //0b1000 0001
    } else {
          ssGetIWork(S)[0] = POSITION_CONTROL;
          c->send(0x81); //0b1000 0001
    }
    ssGetIWork(S)[1] = 0;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    MotorConnection *c = static_cast<MotorConnection*>(ssGetPWork(S)[0]);
    if (c == nullptr) {
        return;
    }
    double *y1 = static_cast<double*>(ssGetOutputPortSignal(S, 0));
    double *y2 = static_cast<double*>(ssGetOutputPortSignal(S, 1));
    unsigned char *y3 = static_cast<unsigned char*>(ssGetOutputPortSignal(S, 2));
    unsigned char first, second, third;
    int operation_mode = ssGetIWorkValue(S, 0);

    // TODO: Is this really necessary? These values are already sent in mdlStart.
    if (ssGetIWorkValue(S, 1) == 0) {
        ssGetIWork(S)[1] = 1;
        if (operation_mode == SPEED_CONTROL) {
            c->send(0x83); //0b1000 0011
            c->send(0x83);
            c->send(0x83);
            c->send(0x83);
            c->send(0x83);
        } else {
            c->send(0x81); //0b1000 0001
            c->send(0x81);
            c->send(0x81);
            c->send(0x81);
            c->send(0x81);
        }
    }

    // Send the control values to the motor. They are explicitly marked as low and high byte by the surrounding MATLAB code.
    const unsigned char *u1 = static_cast<const unsigned char*>(ssGetInputPortSignal(S, 0));
    const unsigned char *u2 = static_cast<const unsigned char*>(ssGetInputPortSignal(S, 1));
    c->send(u1[0]);
    c->send(u2[0]);

    // Read the sensor values.
    unsigned char y[3];
    c->recv(y, 3);

    // Reorder the values (first byte has 7th bit not set).
    // The second and third case is still undesired since the values are not from the same measurement, I assume.
    if (!(y[0] & 0x80)) {
        first = y[0];
        second = y[1];
        third = y[2];
    } else if (!(y[1] & 0x80)) {
        first = y[1];
        second = y[2];
        third = y[0];
    } else {
        first = y[2];
        second = y[0];
        third = y[1];
    }

    int firstByte = static_cast<int>(first);
    int secondByte = static_cast<int>(second);
    int thirdByte = static_cast<int>(third);

    y1[0] = static_cast<double>((firstByte << 3) | ((secondByte & 0x70) >> 4));
    int tempOut2 = ((secondByte & 0xf) << 6) | ((thirdByte & 0x7e) >> 1);

    if (operation_mode == SPEED_CONTROL) {
        if (tempOut2 & 0x200) {
            y2[0] = -static_cast<double>(tempOut2 & 0x1ff);
        } else {
            y2[0] = static_cast<double>(tempOut2);
        }
    } else if (operation_mode == POSITION_CONTROL) {
        y2[0] = static_cast<double>(tempOut2);
    }

    y3[0] = operation_mode;
}

static void mdlTerminate(SimStruct *S)
{
    MotorConnection *c = static_cast<MotorConnection*>(ssGetPWork(S)[0]);
    if (c == nullptr) {
        return;
    }
    // This leads to stopping the motor as both the high byte and the low byte are set to zero.
    c->send(0x00);
    c->send(0x80);
    // Set pointer to null to prevent double free / use after free.
    ssGetPWork(S)[0] = nullptr;
    delete c;
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
