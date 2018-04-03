// 2018 (c) Juan G. Victores
// CopyPolicy: released under the terms of the LGPLv2.1
// URL: https://github.com/jgvictores/snippets

#include <iostream>

#include "RateThread.hpp"

class MyRateThread : public ratethread::RateThread
{
public:
    MyRateThread(const int intervalPeriodMillis) : ratethread::RateThread(intervalPeriodMillis) {}

    void run()
    {
        std::cout<<"[MyRateThread] here..." << std::endl;
    }
};

int main()
{
    MyRateThread rateThread(1000);

    std::cout<<"[main] Start MyThread..."<<std::endl;
    rateThread.start();

    std::cout<<"[main] In blocking std::cin..."<<std::endl;
    char ch{};
    std::cin >> ch;

    std::cout<<"[main] Stopping MyThread..."<<std::endl;
    rateThread.stop();
    std::cout<<"[main] MyThread stopped. Exiting main()."<<std::endl;
    return 0;
}
