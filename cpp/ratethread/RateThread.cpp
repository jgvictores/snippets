// 2018 (c) Juan G. Victores
// CopyPolicy: released under the terms of the LGPLv2.1
// URL: https://github.com/jgvictores/snippets

#include <chrono>
#include <thread>
//#include <iostream>

#include "RateThread.hpp"

namespace ratethread
{

void threadFunction(RateThread* callerRateThreadPtr)
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

    std::chrono::system_clock::time_point currentStartTime{ std::chrono::system_clock::now() };
    std::chrono::system_clock::time_point nextStartTime{ currentStartTime };

    while ( ! callerRateThreadPtr->isStopping() )
    {
        //Get our current "wakeup" time
        currentStartTime = std::chrono::system_clock::now();

        //std::cout << "[threadFunction] Here!" << std::endl;
        callerRateThreadPtr->run();

        //Determine the point in time at which we want to wakeup for the next pass through the loop.
        std::chrono::milliseconds millis{callerRateThreadPtr->getRate()};
        nextStartTime = currentStartTime + millis;

        //Sleep till our next period start time
        std::this_thread::sleep_until(nextStartTime);

    } //end while
}


RateThread::RateThread(const int intervalPeriodMillis) :
     _isStopping(false),
    _intervalPeriodMillis(intervalPeriodMillis)
{
}

RateThread::~RateThread()
{
    stop();
}

void RateThread::start()
{
    _threadPtr = new std::thread( threadFunction, this );
    //std::cout<<"[RateThread] Created new thread..." << std::endl;
}

void RateThread::stop()
{
    if( ! _threadPtr )
        return;
    _isStopping = true;
    //std::cout<<"[RateThread] Waiting For thread to join..." << std::endl;
    _threadPtr->join();
    delete _threadPtr;
    _threadPtr = nullptr;
}

int RateThread::getRate() const
{
    return _intervalPeriodMillis;
}

int RateThread::isStopping() const
{
    return _isStopping;
}

}  // namespace ratethread
