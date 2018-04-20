#ifndef WORKERTHREAD_H
#define WORKERTHREAD_H

#include <tr1/functional>
#include <queue>

typedef std::tr1::function<void()> WorkHandler;
typedef std::queue<WorkHandler> WorkQueue;

class WorkerThread
{
public:
    WorkerThread();
    virtual ~WorkerThread();

    void Start();
    void DoWork(WorkHandler&);
    bool IsBusy();
    void Stop();
    void run();

private:
    volatile bool exitThread;
    volatile bool busy;
    WorkQueue workQueue;
};

#endif // WORKERTHREAD_H
