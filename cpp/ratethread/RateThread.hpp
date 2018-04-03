// 2018 (c) Juan G. Victores
// CopyPolicy: released under the terms of the LGPLv2.1
// URL: https://github.com/jgvictores/snippets

// Help (partial inspiration):
// - http://thispointer.com/c-11-multithreading-part-1-three-different-ways-to-create-threads/
// - https://bulldozer00.com/2013/12/27/periodic-processing-with-standard-c11-facilities/
//     - Copied at: https://gist.github.com/catnapgames/ac0a9060c9d5d637b452f9ee15a6a466
// - Some naming conventions from https://github.com/robotology/yarp RateThread class

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

class RateThreadHelper
{
public:

    RateThreadHelper(const int intervalPeriodMillis);

    void start(IRunnable* iRunnable);

    void stop();

    void loop();

private:
    bool isStopping;
    std::chrono::milliseconds _intervalPeriodMillis;
    IRunnable* _iRunnable;
};

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
    RateThreadHelper rateThreadHelper;
    std::thread* threadPtr;
};


}  // namespace ratethread
