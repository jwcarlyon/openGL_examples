#ifndef THREADPOOL_H_INCLUDED
#define THREADPOOL_H_INCLUDED

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#include <queue>
// A class that represents a thread pool
class ThreadPool
{
public:
    // Constructor that creates a specified number of threads
    ThreadPool(std::size_t num_threads) : stop_(false) {
      // Create the threads and make them wait for tasks
      for (std::size_t i = 0; i < num_threads; ++i) {
        threads_.emplace_back([this] {
          // Loop until the thread pool is stopped
          while(true) {
            // Define a task variable
            std::function<void()> task;
            // Lock the mutex and wait for a task or a stop signal
            {
              std::unique_lock<std::mutex> lock(mutex_);
              condition_.wait(lock, [this] {
                return stop_ || !tasks_.empty();
              });
              // If the thread pool is stopped and there are no tasks, exit the loop
              if(stop_ && tasks_.empty()) {
                return;
              }
              // Otherwise, get the next task from the queue
              task = std::move (tasks_.front());
              tasks_.pop();
            }
            // Execute the task
            task();
          }
        });
      }
    }

    ThreadPool() : stop_(false) {}
    // Destructor that joins the threads and stops the thread pool.
    // This won't work for my main thread strategy.
    ~ThreadPool () {
      // Lock the mutex and set the stop flag to true
      {
          std::unique_lock<std::mutex> lock(mutex_);
          stop_ = true;
      }
      // Notify all threads to wake up
      condition_.notify_all();
      // Join all threads
      for (std::thread& thread : threads_) {
        thread.join();
      }
    }

    // A function that adds a task to the thread pool
    template <class F, class... Args>
    void enqueue(F&& f, Args&&... args)
    {
      // Create a task from the given function and arguments
      std::function<void()> task = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
      // Lock the mutex and push the task to the queue
      {
        std::unique_lock<std::mutex> lock(mutex_);
        tasks_.push(std::move(task));
      }
      // Notify one thread to wake up and execute the task
      condition_.notify_one();
    }

    bool isStopped()
    {
        std::unique_lock<std::mutex> lock(mutex_); //guaranteed the lock will be held while the copy is performed
        return stop_;
    }

    void join()
    {
        condition_.notify_all();
        for (std::thread& thread : threads_) {
          thread.join();
        }
    }
private:
    // A vector of threads
    std::vector<std::thread> threads_;
    // A queue of tasks
    std::queue<std::function<void()>> tasks_;
    // A mutex to synchronize access to the queue
    std::mutex mutex_;
    // A condition variable to signal threads
    std::condition_variable condition_;
    // A flag to indicate whether the thread pool is stopped
    bool stop_;
};

#endif // THREADPOOL_H_INCLUDED
