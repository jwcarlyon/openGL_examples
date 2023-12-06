#ifndef KILLALLTHREADSSIGNAL_H_INCLUDED
#define KILLALLTHREADSSIGNAL_H_INCLUDED



class KillAllThreadsSignal
{
public:
    KillAllThreadsSignal() : killAllThreads(false) {}

    bool checkSignal()
    {
        std::unique_lock<std::mutex> lock(signalMutex);
        return killAllThreads;
    }

    void setSignal()
    {
        std::unique_lock<std::mutex> lock(signalMutex);
        killAllThreads = true;
    }

private:
    bool killAllThreads;
    std::mutex signalMutex;
};

#endif // KILLALLTHREADSSIGNAL_H_INCLUDED
