// 2018 (c) Juan G. Victores
// CopyPolicy: released under the terms of the LGPLv2.1
// URL: https://github.com/jgvictores/snippets

#include <chrono>
#include <thread>
//#include <iostream>

#include "RateThread.hpp"

namespace ratethread
{

class RateThreadHelper
{
public:
    RateThreadHelper(const int intervalPeriodMillis);
    void start(IRunnable* iRunnable);
    void stop();

private:
    void loop();
    bool _isStopping;
    std::chrono::milliseconds _intervalPeriodMillis;
    IRunnable* _iRunnable;
};

RateThreadHelper::RateThreadHelper(const int intervalPeriodMillis) : _isStopping(false)
{
    _intervalPeriodMillis = std::chrono::milliseconds{intervalPeriodMillis};
}

void RateThreadHelper::start(IRunnable* iRunnable)
{
    /*const double microsPerClkTic{ 1.0E6 * std::chrono::system_clock::period::num / std::chrono::system_clock::period::den };
    std::cout << "[RateThreadHelper] system_clock precision = "
              << microsPerClkTic
              << " microseconds/tic"
              << std::endl
              << "Desired Wakeup Period = "
              << _intervalPeriodMillis.count()
              << " milliseconds"
              << std::endl;*/
    _iRunnable = iRunnable;
    loop();
}

void RateThreadHelper::stop()
{
    _isStopping = true;
}

void RateThreadHelper::loop()
{
    std::chrono::system_clock::time_point currentStartTime{ std::chrono::system_clock::now() };
    std::chrono::system_clock::time_point nextStartTime{ currentStartTime };

    while ( ! _isStopping )
    {
        //Get our current "wakeup" time
        currentStartTime = std::chrono::system_clock::now();

        //std::cout << "Here: " << loopNum << std::endl;
        _iRunnable->run();

        //Determine the point in time at which we want to wakeup for the next pass through the loop.
        nextStartTime = currentStartTime + _intervalPeriodMillis;

        //Sleep till our next period start time
        std::this_thread::sleep_until(nextStartTime);

    } //end while
}


RateThread::RateThread(const int intervalPeriodMillis)
{
    _rateThreadHelperPtr = new RateThreadHelper(intervalPeriodMillis);
}

RateThread::~RateThread()
{
    stop();
    delete reinterpret_cast<RateThreadHelper*>(_rateThreadHelperPtr);
    _rateThreadHelperPtr = nullptr;
}

void RateThread::start()
{
    _threadPtr = new std::thread( &RateThreadHelper::start, reinterpret_cast<RateThreadHelper*>(_rateThreadHelperPtr), this );
    //std::cout<<"[RateThread] Created new thread..." << std::endl;
}

void RateThread::stop()
{
    if( ! _threadPtr )
        return;
    reinterpret_cast<RateThreadHelper*>(_rateThreadHelperPtr)->stop();
    //std::cout<<"[RateThread] Waiting For thread to join..." << std::endl;
    _threadPtr->join();
    delete _threadPtr;
    _threadPtr = nullptr;
}

}  // namespace ratethread
