#ifndef __WORKER_THREAD_H__
#define __WORKER_THREAD_H__

#include <queue>
#include <functional>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

class WorkerThread
{
public:
	typedef std::function<void()> Work;

public:
	WorkerThread(size_t threadCount = 4)
		: mIsExited(false)
	{
		auto threadFunc = boost::bind(&WorkerThread::ProcessWork, this);

		for (size_t i = 0; i < threadCount; i++)
		{
			mThreadGroup.create_thread(threadFunc);
		}
	}

	~WorkerThread()
	{
		mIsExited = true;

		Clear();

		mThreadGroup.join_all();
	}

public:
	void Enqueue(Work work)
	{
		mWorkQueueMutex.lock();

		mWorkQueue.push(work);

		mWorkerQueueCondition.notify_one();

		mWorkQueueMutex.unlock();
	}

	void Clear()
	{
		mWorkQueueMutex.lock();

		mWorkQueue = std::queue<Work>();

		mThreadGroup.interrupt_all();
		
		mWorkQueueMutex.unlock();
	}

private:
	void ProcessWork()
	{
		for(; !mIsExited;)
		{
			try
			{
				boost::mutex::scoped_lock lock(mWorkQueueMutex);

				for (;; mWorkerQueueCondition.wait(lock))
				{
					if (!mWorkQueue.empty())
					{
						auto work = mWorkQueue.front();
						mWorkQueue.pop();

						lock.unlock();	

						work();

						lock.lock();
					}
				}
			}
			catch(boost::thread_interrupted&)
			{}
			catch(...)
			{
				std::cout << "Unknown Error" << std::endl;
			}
		}
	}

private:
	boost::thread_group mThreadGroup;

	std::queue<Work> mWorkQueue;
	boost::mutex mWorkQueueMutex;	
	boost::condition_variable mWorkerQueueCondition;

	bool mIsExited;
};

#endif

