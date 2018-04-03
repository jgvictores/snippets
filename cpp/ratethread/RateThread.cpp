// 2018 (c) Juan G. Victores
// CopyPolicy: released under the terms of the LGPLv2.1
// URL: https://github.com/jgvictores/snippets

#include <chrono>
#include <thread>
//#include <iostream>

#include "RateThread.hpp"

namespace ratethread
{

RateThreadHelper::RateThreadHelper(const int intervalPeriodMillis) : isStopping(false)
{
    std::chrono::milliseconds a{intervalPeriodMillis};
    _intervalPeriodMillis = a;
}

void RateThreadHelper::start(IRunnable* iRunnable)
{
    /*std::cout << "[RateThreadHelper] system_clock precision = "
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
    isStopping = true;
}

void RateThreadHelper::loop()
{
    std::chrono::system_clock::time_point currentStartTime{ std::chrono::system_clock::now() };
    std::chrono::system_clock::time_point nextStartTime{ currentStartTime };

    while ( ! isStopping )
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



RateThread::RateThread(const int intervalPeriodMillis) : rateThreadHelper(intervalPeriodMillis) {}

void RateThread::start()
{
    threadPtr = new std::thread( &RateThreadHelper::start, &rateThreadHelper, this );
    //std::cout<<"[RateThread] Created new thread..." << std::endl;
}

void RateThread::stop()
{
    rateThreadHelper.stop();
    //std::cout<<"[RateThread] Waiting For thread to join..." << std::endl;
    threadPtr->join();
    delete threadPtr;
    threadPtr = 0;
}

}  // namespace ratethread
