#include <iostream>
#include <iomanip>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <queue>
#define RECEIVEBUFFERSIZE 256

namespace omni3wdbot_hardware_interface
{
    struct FeedbackStruct
    {
        int DelayMs = 0;
        int F = 0;
        int RR = 0;
        int RL = 0;
    };

    struct GainStruct
    {
        double KP = 0.0;
        double KI = 0.0;
        double KD = 0.0;
    };

    class OmniBotComm
    {
    public:
        OmniBotComm(const std::string &portname, int baudrate);
        ~OmniBotComm();
        void ReadStart();
        bool WriteGain(GainStruct f_motor, GainStruct rr_motor, GainStruct rl_motor);
        void WriteTarget(int f_motor, int rr_motor, int rl_motor);
        FeedbackStruct GetLatestFeedback();
        FeedbackStruct GetFeedbackMergeAllQueue();

    private:
        boost::thread thread;
        unsigned char receive_data[RECEIVEBUFFERSIZE];
        unsigned char last_surplus_data_[RECEIVEBUFFERSIZE];
        int last_surplus_data_size_ = 0;
        boost::asio::io_service io;
        boost::asio::serial_port serial;
        boost::asio::io_service::work w;
        FeedbackStruct latestFeedback = {0, 0, 0, 0};
        std::queue<FeedbackStruct> feedbackQueue;
        void readasync();
    };
}