#pragma once

#include <ctime>

// Simple CPU clock timer.
class ClockTimer
{
private:
    std::size_t start;

public:
    ClockTimer (void);

    void reset (void);
    static std::size_t now (void);
    std::size_t get_elapsed (void) const;
};

/* ------------------------ Implementation ------------------------ */

inline
ClockTimer::ClockTimer (void)
{
    this->reset();
}

inline void
ClockTimer::reset (void)
{
    this->start = ClockTimer::now();
}

inline std::size_t
ClockTimer::now (void)
{
    return static_cast<std::size_t>(std::clock()) * 1000
        / static_cast<std::size_t>(CLOCKS_PER_SEC);
}

inline std::size_t
ClockTimer::get_elapsed (void) const
{
    return ClockTimer::now() - start;
}
