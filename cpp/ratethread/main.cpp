// 2018 (c) Juan G. Victores
// CopyPolicy: released under the terms of the LGPLv2.1
// URL: https://github.com/jgvictores/snippets

// Help (partial inspiration):
// - http://thispointer.com/c-11-multithreading-part-1-three-different-ways-to-create-threads/
// - https://bulldozer00.com/2013/12/27/periodic-processing-with-standard-c11-facilities/
//     - Copied at: https://gist.github.com/catnapgames/ac0a9060c9d5d637b452f9ee15a6a466
// - Some naming conventions from https://github.com/robotology/yarp RateThread class

#include <chrono>
#include <iostream>
#include <thread>

//Determine our platform's tic period
const double microsPerClkTic
{
    1.0E6 * std::chrono::system_clock::period::num / std::chrono::system_clock::period::den
};


class RateThreadHelper
{
public:
    RateThreadHelper(const int intervalPeriodMillis) : isStopping(false)
    {
        std::chrono::milliseconds a{intervalPeriodMillis};
        _intervalPeriodMillis = a;
    }

    void start()
    {
        std::cout << "system_clock precision = "
                  << microsPerClkTic
                  << " microseconds/tic"
                  << std::endl
                  << "Desired Wakeup Period = "
                  << _intervalPeriodMillis.count()
                  << " milliseconds"
                  << std::endl;
        loop();
    }

    void stop()
    {
        isStopping = true;
    }

    void loop()
    {

        std::chrono::system_clock::time_point currentStartTime{ std::chrono::system_clock::now() };
        std::chrono::system_clock::time_point nextStartTime{ currentStartTime };

        int32_t loopNum{ 0 };

        while ( ! isStopping )
        {
            //Get our current "wakeup" time
            currentStartTime = std::chrono::system_clock::now();

            std::cout << "Here: " << loopNum << std::endl;

            //Determine the point in time at which we want to wakeup for the next pass through the loop.
            nextStartTime = currentStartTime + _intervalPeriodMillis;

            //Sleep till our next period start time
            std::this_thread::sleep_until(nextStartTime);
            ++loopNum;
        } //end while
    }

private:
    bool isStopping;
    std::chrono::milliseconds _intervalPeriodMillis;
};

class RateThread
{
public:

    RateThread(const int intervalPeriodMillis) : rateThreadHelper(intervalPeriodMillis) {}

    void start()
    {
        pthread = new std::thread( &RateThreadHelper::start, &rateThreadHelper );
        std::cout<<"[RateThread] Created new Thread..." << std::endl;
    }

    void stop()
    {
        rateThreadHelper.stop();
        std::cout<<"[RateThread] Waiting For Thread to join..." << std::endl;
        pthread->join();
        delete pthread;
        pthread = 0;
    }

private:
    RateThreadHelper rateThreadHelper;
    std::thread* pthread;
};


int main()
{
    RateThread rateThread(1000);
    rateThread.start();

    std::cout<<"[main] At cin..."<<std::endl;
    char ch{};
    std::cin >> ch;
    rateThread.stop();

    std::cout<<"[main] Waiting For Thread to complete..."<<std::endl;
    std::cout<<"[main] Exiting from Main Thread"<<std::endl;
    return 0;
}
