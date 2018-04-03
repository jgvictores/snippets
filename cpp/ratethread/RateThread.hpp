// 2018 (c) Juan G. Victores
// CopyPolicy: released under the terms of the LGPLv2.1
// URL: https://github.com/jgvictores/snippets

#include <chrono>
#include <thread>

namespace ratethread
{

class IRunnable
{
public:
    virtual void run() = 0;
    virtual ~IRunnable() {}
};

class RateThreadHelper;

class RateThread : public IRunnable
{
public:

    RateThread(const int intervalPeriodMillis);

    /***
     * Loop function.
     *
     * This is the thread itself. The thread calls the run() function every <period> ms. At the end of each run, the thread will sleep
     * the amount of time required, taking into account the time spent inside the loop function.
     ***/
    virtual void run() = 0;

    /// Call this to start the thread
    void start();

    /// Call this to stop the thread, this call blocks until the thread is terminated
    void stop();

private:
    RateThreadHelper* _rateThreadHelperPtr;
    std::thread* _threadPtr;
};


}  // namespace ratethread
