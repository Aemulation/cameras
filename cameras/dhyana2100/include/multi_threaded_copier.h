#include <array>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

static std::array<unsigned char, 10321920> dummy_buffer;

class MultiThreadedCopier {
  public:
    MultiThreadedCopier(unsigned num_threads) : keep_running(true) {
        for (unsigned thread_id{}; thread_id != num_threads; ++thread_id) {
            threads.push_back(std::thread(thread_copy_data_caller, this));
        }
    }

    ~MultiThreadedCopier() {
        keep_running = false;
        copy_requests_queue_condition_variable.notify_all();

        for (std::thread &thread : threads) {
            thread.join();
        }
    }

    void queue_copy(unsigned char *destination, unsigned char *const source,
                    std::size_t size) {
        {
            std::lock_guard queued_copies_lock(queued_copies_mutex);
            ++queued_copies_counter;
        }

        {
            std::lock_guard copy_request_queue_lock(copy_requests_queue_mutex);
            copy_requests_queue.push_back(CopyRequest{
                .destination = destination, .soure = source, .size = size});
        }

        copy_requests_queue_condition_variable.notify_all();
    }

    void wait_all() {
        std::unique_lock queued_copies_lock(queued_copies_mutex);
        if (queued_copies_counter) {
            queued_copies_conditional_variable.wait(
                queued_copies_lock, [this] { return !queued_copies_counter; });
        }
    }

  private:
    static void thread_copy_data_caller(MultiThreadedCopier *data_copier) {
        data_copier->thread_copy_data();
    }

    void thread_copy_data() {
        while (true) {
            std::unique_lock copy_requests_queue_lock(
                copy_requests_queue_mutex);
            if (copy_requests_queue.empty() && keep_running) {
                copy_requests_queue_condition_variable.wait(
                    copy_requests_queue_lock, [this] {
                        return !copy_requests_queue.empty() || !keep_running;
                    });
            }
            if (!keep_running) {
                return;
            }

            CopyRequest copy_request = copy_requests_queue.front();
            copy_requests_queue.pop_front();
            copy_requests_queue_lock.unlock();

            memcpy(copy_request.destination, copy_request.soure,
                   copy_request.size);

            {
                std::unique_lock queued_copies_lock(queued_copies_mutex);
                --queued_copies_counter;
            }
            queued_copies_conditional_variable.notify_all();
        }
    }

  private:
    struct CopyRequest {
        unsigned char *destination;
        unsigned char *const soure;
        std::size_t size;
    };

    std::vector<std::thread> threads;
    std::deque<CopyRequest> copy_requests_queue;
    std::mutex copy_requests_queue_mutex;
    std::condition_variable copy_requests_queue_condition_variable;

    volatile bool keep_running;

    std::mutex queued_copies_mutex;
    std::condition_variable queued_copies_conditional_variable;
    std::size_t queued_copies_counter = 0;
};
