// 2018 (c) Juan G. Victores
// CopyPolicy: released under the terms of the LGPLv2.1
// URL: https://github.com/jgvictores/snippets

#include <thread>

namespace ratethread
{

class RateThread
{
public:
    RateThread(const int intervalPeriodMillis);
    ~RateThread();

    /// Call this to start the thread
    void start();

    /// Call this to stop the thread, this call blocks until the thread is terminated
    void stop();

    /// Return the current rate of the thread [ms].
    int getRate() const;

    /// Return if is stopping.
    int isStopping() const;

    /***
     * Loop function.
     *
     * This is the thread itself. The thread calls the run() function every <period> ms. At the end of each run, the thread will sleep
     * the amount of time required, taking into account the time spent inside the loop function.
     ***/
    virtual void run() = 0;

private:
    int _intervalPeriodMillis;
    bool _isStopping;
    std::thread* _threadPtr;
};


}  // namespace ratethread
