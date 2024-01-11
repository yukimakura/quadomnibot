#include <iostream>
#include <iomanip>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include "omni3wdbot_hardware_interface/json.hpp"
#include "omni3wdbot_hardware_interface/omnibot_comm.hpp"
using json = nlohmann::json;
using namespace omni3wdbot_hardware_interface;

OmniBotComm::OmniBotComm(const std::string &portname, int baudrate) : io(), serial(io), w(io)
{
    serial.open(portname);
    serial.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
    thread = boost::move(t);
}

OmniBotComm::~OmniBotComm()
{
    io.stop();
    thread.join();
}
void OmniBotComm::ReadStart()
{
    readasync();
}

bool OmniBotComm::WriteGain(GainStruct f_motor, GainStruct rr_motor, GainStruct rl_motor)
{
    char sendbuff[512];
    memset(sendbuff, 0, sizeof(sendbuff));
    sprintf(sendbuff, "{\"F_P\":%.4f,\"F_I\":%.4f,\"F_D\":%.4f,\"RL_P\":%.4f,\"RL_I\":%.4f,\"RL_D\":%.4f,\"RR_P\":%.4f,\"RR_I\":%.4f,\"RR_D\":%.4f}\n",
            f_motor.KP,
            f_motor.KI,
            f_motor.KD,
            rl_motor.KP,
            rl_motor.KI,
            rl_motor.KD,
            rr_motor.KP,
            rr_motor.KI,
            rr_motor.KD);
    auto wb = boost::asio::buffer(sendbuff, std::strlen(sendbuff));
    boost::asio::write(serial, wb);
    std::cout << "write gain : " << (char *)wb.data() << std::endl;

    boost::asio::streambuf response_buf;
    boost::asio::read_until(serial, response_buf, '\n');

    std::string msg = std::string(boost::asio::buffer_cast<const char *>(response_buf.data()));

    int cnt = 0;
    // 表示して終わり
    while (msg != "GainChanged\n")
    {
        if (cnt++ > 1000)
            return false;
        sleep(0.01);
        msg = std::string(boost::asio::buffer_cast<const char *>(response_buf.data()));
    }

    return true;
}

void OmniBotComm::WriteTarget(int f_motor, int rr_motor, int rl_motor)
{
    char sendbuff[128];
    memset(sendbuff, 0, sizeof(sendbuff));
    sprintf(sendbuff, "{\"F\":%d,\"RR\":%d,\"RL\":%d}\n", f_motor, rr_motor, rl_motor);
    auto wb = boost::asio::buffer(sendbuff, std::strlen(sendbuff));
    boost::asio::write(serial, wb);
    std::cout << "write target : " << (char *)wb.data() << std::endl;
}

FeedbackStruct OmniBotComm::GetLatestFeedback()
{
    return this->latestFeedback;
}

FeedbackStruct OmniBotComm::GetFeedbackMergeAllQueue()
{
    FeedbackStruct retFeedback = {0, 0, 0, 0};
    while (this->feedbackQueue.size() > 0)
    {
        retFeedback.DelayMs += this->feedbackQueue.front().DelayMs;
        retFeedback.F += this->feedbackQueue.front().F;
        retFeedback.RR += this->feedbackQueue.front().RR;
        retFeedback.RL += this->feedbackQueue.front().RL;
        this->feedbackQueue.pop();
    }

    return retFeedback;
}

void OmniBotComm::readasync()
{
    boost::asio::async_read(serial, boost::asio::buffer(receive_data),
                            [this](boost::system::error_code ec, std::size_t recv_size)
                            {
                                if (!ec)
                                {
                                    unsigned char rawbytes_with_surplus_data[1024];
                                    memset(rawbytes_with_surplus_data, 0, 1024);

                                    int rawbytes_with_surplus_data_count = recv_size + last_surplus_data_size_;
                                    std::copy(last_surplus_data_, last_surplus_data_ + last_surplus_data_size_, rawbytes_with_surplus_data);
                                    std::copy(receive_data, receive_data + recv_size, rawbytes_with_surplus_data + last_surplus_data_size_);

                                    std::vector<std::string> tokens;
                                    boost::split(tokens, rawbytes_with_surplus_data, boost::is_any_of("\n"), boost::token_compress_on);

                                    bool is_last_discarded = false;
                                    for (auto &s : tokens)
                                    {
                                        auto data = json::parse(s, nullptr, false);
                                        if (!data.is_discarded())
                                        {
                                            is_last_discarded = false;
                                            if (!data["F"].is_null() && !data["RR"].is_null() && !data["RL"].is_null() && !data["DelayMs"].is_null())
                                            {
                                                this->latestFeedback = {data["DelayMs"], data["F"], data["RR"], data["RL"]};
                                                this->feedbackQueue.push(this->latestFeedback);
                                            }
                                        }
                                        else
                                            is_last_discarded = true;
                                    }

                                    if (is_last_discarded)
                                    {
                                        std::vector<std::string> recvtokens;
                                        boost::split(recvtokens, receive_data, boost::is_any_of("\n"), boost::token_compress_on);
                                        auto last_discard_data = recvtokens.back();
                                        last_surplus_data_size_ = last_discard_data.length();
                                        // このターンで受信したデータだけ次回用にバッファリングしておく(配列溢れ対策)
                                        std::copy(last_discard_data.c_str(), last_discard_data.c_str() + last_surplus_data_size_, last_surplus_data_);
                                    }
                                    else
                                    {
                                        last_surplus_data_size_ = 0;
                                        memset(last_surplus_data_, 0, RECEIVEBUFFERSIZE);
                                    }
                                }
                                memset(receive_data, 0, RECEIVEBUFFERSIZE);
                                this->readasync();
                            });
}
