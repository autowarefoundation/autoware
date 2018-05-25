//
// SickThread.hpp
//

#ifndef SICKTHREAD_HPP
#define SICKTHREAD_HPP

#include "../BasicDatatypes.hpp"
#include <pthread.h>
#include <unistd.h>


extern "C" void* wrapper_prerun(void*);
class ThreadWrapperBase
{
    pthread_t t_id;
    friend void* wrapper_prerun(void*);
    virtual void thread_entry() = 0;
  protected:
    void* pthis;
  public:
	
	ThreadWrapperBase() {pthis = NULL;};
	virtual ~ThreadWrapperBase() {};
	  
    void run(void* classptr)
	{
		if (pthis == NULL)
		{
			pthis = classptr;
			pthread_create(&t_id, NULL, wrapper_prerun, this);
		}
    }
    
    bool isRunning()
	{
		if (pthis == NULL)
		{
			return false;
		}
		
		return true;
	}
	
    void join()
	{
		pthread_join(t_id, NULL);
		pthis = NULL;
    }
    
    pthread_t* get_thread_id()
	{
      return &t_id;
    }
};


/// Wrapper class for posix threads
/**
 * Usage: Using object must create an instance of this class, and then
 * call start() with its callback function as argument (see start()
 * for more details).  To stop the thread execution, call stop().
 *
 * Setting the parameter m_beVerbose to true (e.g. via
 * enableVerboseDebugOutput in function start()) will turn on *very*
 * verbose output that should be useful for debugging.
 *
 * The thread callback function itself has 2 parameters:
 *
 * endThisThread: A bool flag that may be set by the callback function
 *  to "false" in case the thread function decides this thread
 *  needs to end.
 *
 * sleepTimeMs: The sleep time, in ms, that will be spent between
 *  subsequent calls to the callback function. Default is 10 ms, but
 *  other times may be set. Note that not all operating systems may be
 *  able to schedule very short sleep times.
 */
template <typename T, void (T::*M)(bool&, UINT16&)>
class SickThread : public ThreadWrapperBase
{
 	void thread_entry()
 	{
 		T* pt = static_cast<T*>(pthis);
		
		m_threadShouldRun = true;
		bool endThread = false;
		UINT16 sleepTimeMs = 0;
		
		while ((m_threadShouldRun == true) && (endThread == false))
		{
			usleep(((UINT32)sleepTimeMs) * 1000);
			(pt->*M)(endThread, sleepTimeMs);
		}
 	}
 	
		
public:
 	void join()
	{
		m_threadShouldRun = false;
		ThreadWrapperBase::join();
	}
	
	SickThread(){m_threadShouldRun = true;}
	virtual ~SickThread(){};
	bool m_threadShouldRun;
};

/*
template <typename T, void (T::*M)()>
class SickThread : public ThreadWrapperBase
{
 	void thread_entry()
 	{
 		T* pt = static_cast<T*>(pthis);
		
		
 		(pt->*M)();
 	}
public:
	SickThread(){}
	virtual ~SickThread(){};
};
*/



// class SickThread
// {
// public:
// 	/**
// 	 * The thread callback function.
// 	 *
// 	 * \param endThisThread A bool flag that may be set by the callback function
// 	 *  to "false" in case the thread function decides this thread
// 	 *  needs to end.
// 	 *
// 	 * \param sleepTimeMs The sleep time, in ms, that will be spent between
// 	 *  subsequent calls to the callback function. Default is 10 ms, but
// 	 *  other times may be set. Note that not all operating systems may be
// 	 *  able to schedule very short sleep times.
// 	 */
// //	int (*comp)(const void *, const void *)
// 	typedef void (*ThreadFunction) (bool& endThisThread, UINT16& sleepTimeMs);
// 
// 	/**
// 	 * The thread callback function (simpler version).
// 	 *
// 	 * \return True if the thread should continue to run and
// 	 * continuously call this function (after potentially some waiting
// 	 * time). False if this thread should end now.
// 	 */
// //	typedef boost::function < bool (void) > ThreadFunctionSimple;
// 
// 	/// Default constructor.
// 	SickThread();
// 
// 	/// Destructor. Will call stop() if thread is not yet stopped, and
// 	/// wait for its completion before destructing this object.
// 	~SickThread();
// 
// 	/// Start the thread.
// 	bool start();
// 
// 	/// Start the thread and also set the thread function. \sa start()
// 	bool start(ThreadFunction function);
// 
// 	/// Returns true if this thread was started and is running.
// 	bool isRunning() const;
// 
// 	/// Stops this thread and waits for its completion.
// 	void stop();
// 
// 	/// Set whether we want verbose debug output
// 	void setVerboseDebugOutput(bool enableVerboseDebugOutput);
// 
// 	/// Set the thread's "run" function
// 	void setFunction(ThreadFunction threadFunction);
// 
// 	/// Set the thread's "run" function, simpler version
// //	void setFunctionSimple(ThreadFunctionSimple threadFunctionSimple);
// 
// 	/// Set the sleep time between subsequent ThreadFunctionSimple calls in [milliseconds]
// 	void setSleepTimeMilliseconds(unsigned v);
// 
// 	/// Returns the sleep time between subsequent ThreadFunctionSimple
// 	/// calls in [milliseconds]. (Default value is zero.)
// 	unsigned getSleepTimeMilliseconds() const { return m_sleepTimeMilliseconds; }
// 
// private:
// 	static void* thread(void* ptr);			// The thread function
// 	void thread2();							// The member thread function
// 
// //	typedef boost::mutex Mutex;
// 
// 	pthread_mutex_t m_mutex;	//  = PTHREAD_MUTEX_INITIALIZER;
// //	mutable Mutex m_threadMutex;
// //	boost::condition m_threadCondition;
// 	ThreadFunction m_function;
// //	ThreadFunctionSimple m_functionSimple;
// 
// //	boost::scoped_ptr<boost::thread> m_threadPtr;
// 	bool m_threadShouldRun;
// 	bool m_threadIsRunning;
// 	bool m_beVerbose;
// 	unsigned m_sleepTimeMilliseconds;
// 	
// 	// The thread
// 	pthread_t m_thread;
// 
// };

/*
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
 
#define NUM_THREADS     5
 
void *TaskCode(void *argument)
{
   int tid;
 
   tid = *((int *) argument);
   printf("Hello World! It's me, thread %d!\n", tid);
 
   // optionally: insert more useful stuff here
 
   return NULL;
}
 
int main(void)
{
   pthread_t threads[NUM_THREADS];
   int thread_args[NUM_THREADS];
   int rc, i;
 
   // create all threads
   for (i=0; i<NUM_THREADS; ++i) {
      thread_args[i] = i;
      printf("In main: creating thread %d\n", i);
      rc = pthread_create(&threads[i], NULL, TaskCode, (void *) &thread_args[i]);
      assert(0 == rc);
   }
 
   // wait for all threads to complete
   for (i=0; i<NUM_THREADS; ++i) {
      rc = pthread_join(threads[i], NULL);
      assert(0 == rc);
   }
 
   exit(EXIT_SUCCESS);
}
*/

#endif // SICKTHREAD_HPP
