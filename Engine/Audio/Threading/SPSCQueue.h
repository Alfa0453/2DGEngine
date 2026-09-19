#pragma once

#include <atomic>
#include <cstddef>
#include <limits>
#include <memory.h>
#include <memory>
#include <type_traits>

namespace Engine
{
    template<typename T>
    class SPSCQueue
    {
    public:

        static_assert(std::is_trivially_copyable_v<T>, "SPSCQueue requires trivially copyable items.");

        SPSCQueue() = default;

        SPSCQueue(const SPSCQueue&) = delete;

        SPSCQueue& operator=(const SPSCQueue&) = delete;

        bool Initialize(std::size_t capacity)
        {
            if (capacity == 0 || capacity == std::numeric_limits<std::size_t>::max())
            {
                return false;
            }

            Shutdown();

            m_StorageCapacity = capacity + 1;

            m_Buffer = std::make_unique<T[]>(m_StorageCapacity);

            m_Head.store(0, std::memory_order_relaxed);

            m_Tail.store(0, std::memory_order_relaxed);

            return true;
        }

        void Shutdown()
        {
            m_Buffer.reset();

            m_StorageCapacity = 0;

            m_Head.store(0, std::memory_order_relaxed);

            m_Tail.store(0, std::memory_order_relaxed);
        }

        bool IsInitialized() const
        {
            return m_Buffer != nullptr && m_StorageCapacity > 1;
        }

        bool Push(const T& item)
        {
            if (!IsInitialized())
            {
                return false;
            }

            const std::size_t head = m_Head.load(std::memory_order_relaxed);

            const std::size_t nextHead = IncrementIndex(head);

            const std::size_t tail = m_Tail.load(std::memory_order_acquire);

            if (nextHead == tail)
            {
                return false;
            }

            m_Buffer[head] = item;

            m_Head.store(nextHead, std::memory_order_release);

            return true;
        }

        bool TryPop(T& outItem)
        {
            if (!IsInitialized())
            {
                return false;
            }

            const std::size_t tail = m_Tail.load(std::memory_order_relaxed);

            const std::size_t head = m_Head.load(std::memory_order_acquire);

            if (tail == head)
            {
                return false;
            }

            outItem = m_Buffer[tail];

            m_Tail.store(IncrementIndex(tail), std::memory_order_release);

            return true;
        }

        bool IsEmpty() const
        {
            if (!IsInitialized())
            {
                return true;
            }

            return m_Head.load(std::memory_order_acquire) == m_Tail.load(std::memory_order_acquire);
        }

        std::size_t GetCapacity() const
        {
            if (m_StorageCapacity <= 1)
            {
                return 0;
            }

            return m_StorageCapacity - 1;
        }

        std::size_t GetApproximateCount() const
        {
            if (!IsInitialized())
            {
                return 0;
            }

            const std::size_t head = m_Head.load(std::memory_order_acquire);

            const std::size_t tail = m_Tail.load(std::memory_order_acquire);

            if (head >= tail)
            {
                return head - tail;
            }

            return m_StorageCapacity - (tail - head);
        }

        void Reset()
        {
            m_Head.store(0, std::memory_order_relaxed);

            m_Tail.store(0, std::memory_order_relaxed);
        }

    private:

        std::size_t IncrementIndex(std::size_t index) const
        {
            ++index;

            if (index >= m_StorageCapacity)
            {
                index = 0;
            }

            return index;
        }

    private:

        std::unique_ptr<T[]> m_Buffer;

        std::size_t m_StorageCapacity = 0;

        std::atomic<std::size_t> m_Head{0};

        std::atomic<std::size_t> m_Tail{0};
    };
}