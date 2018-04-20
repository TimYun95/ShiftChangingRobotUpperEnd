#include "workerthread.h"

#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>

/*usage: see exynos4412 RobotController
    WorkHandler h=std::tr1::bind(&Action::ReadTeachFile,action,msgData);
    workerThread.DoWork(h);
    while(workerThread.IsBusy()){
        SleepMs(1);
    }
*/

void* workerThreadFunc(void* arg)
{
    WorkerThread* wt=static_cast<WorkerThread*>(arg);
    wt->run();
    return NULL;
}

WorkerThread::WorkerThread()
{
}

WorkerThread::~WorkerThread()
{
}

void WorkerThread::Start()
{
    pthread_t tid;
    pthread_create(&tid,NULL,workerThreadFunc,this);
}

void WorkerThread::DoWork(WorkHandler &h)
{
    busy=true;
    workQueue.push(h);
}

bool WorkerThread::IsBusy()
{
    return busy;
}

void WorkerThread::Stop()
{
    exitThread=true;
}

void WorkerThread::run()
{
    while(!exitThread){
        usleep(100*1000);
        if(!workQueue.empty()){
            busy=true;
            WorkHandler handler=workQueue.front();
            workQueue.pop();
            handler();
        }else{
            busy=false;
        }
    }
}
