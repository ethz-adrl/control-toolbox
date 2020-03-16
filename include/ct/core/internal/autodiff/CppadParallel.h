/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <mutex>
#include <map>
#include <thread>

namespace ct {
namespace core {

#ifdef CPPAD

/**
 * @brief      This class provides static methods to initialize a parallel
 *             section containing CppAD objects
 */
class CppadParallel
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    static CppadParallel& getInstance()
    {
        static CppadParallel instance;  // Guaranteed to be destroyed, instantiated on first use.
        return instance;
    }

    static void initParallel(size_t numThreads) { CppadParallel::getInstance().initParallelImpl(numThreads); }
    static void resetParallel() { CppadParallel::getInstance().resetParallelImpl(); }

private:
    /**
     * @brief      Call this function before entering a parallel section
     *             containing CppAD objects
     *
     * @param[in]  numThreads  The number of threads. Make sure to count the main thread in
     */
    void initParallelImpl(size_t numThreads)
    {
        isSequential_ = true;
        numThreads_ = numThreads;

        CppAD::thread_alloc::parallel_setup(numThreads_, in_parallel, thread_number);

        CppAD::thread_alloc::hold_memory(true);
        CppAD::parallel_ad<double>();
        isSequential_ = false;
    }

    void resetParallelImpl()
    {
        threadMap_.clear();
        isSequential_ = true;
        CppAD::thread_alloc::parallel_setup(1, CPPAD_NULL, CPPAD_NULL);
        CppAD::thread_alloc::hold_memory(false);
        CppAD::parallel_ad<double>();
    }


    CppadParallel() : isSequential_(false), numThreads_(100) {}
    bool isSequential_;
    std::map<size_t, size_t> threadMap_;
    size_t numThreads_;
    std::mutex hashMutex_;

    /**
     * @brief      This method gets called by Cppad::parallel_setup and
     *             indicates whether the program is executing a parallel section, thus it needs to be static
     *
     * @return     True if in parallel
     */
    static bool in_parallel(void) { return !(CppadParallel::getInstance().isSequential_); }
    /**
     * @brief      This method gets called by Cppad::parallel_setup and
     *             indicates the thread number of the current thread
     *
     * @return     The thread number
     */
    static size_t thread_number(void)
    {
        std::thread::id this_id = std::this_thread::get_id();
        std::hash<std::thread::id> hasher;
        size_t hashId = hasher(this_id);

        CppadParallel& instance = CppadParallel::getInstance();

        instance.hashMutex_.lock();
        if (instance.threadMap_.size() < instance.numThreads_)
            if (!instance.threadMap_.count(hashId))
                instance.threadMap_.insert(std::make_pair(hashId, instance.threadMap_.size()));

        size_t threadNum = instance.threadMap_[hashId];
        instance.hashMutex_.unlock();

        return threadNum;
    }

    // singleton stuff
public:
    CppadParallel(CppadParallel const&) = delete;
    void operator=(CppadParallel const&) = delete;
};

#endif
}  // namespace core
}  // namespace ct
