#pragma once

#include <Arduino.h>

class TaskScheduler {
public:
    using TaskFn = void (*)();
    static constexpr uint8_t MAX_TASKS = 12;

    bool add(uint32_t interval_ms, TaskFn fn, const char* name) {
        if (m_count >= MAX_TASKS || fn == nullptr || interval_ms == 0) {
            return false;
        }
        m_tasks[m_count++] = Task{fn, name, interval_ms, 0, 0};
        return true;
    }

    void tick(uint32_t now_ms) {
        for (uint8_t i = 0; i < m_count; ++i) {
            Task& task = m_tasks[i];
            if (now_ms - task.last_ms >= task.interval_ms) {
                task.last_ms = now_ms;
                const uint32_t start = micros();
                task.fn();
                task.busy_us += micros() - start;
            }
        }
    }

    void reset() {
        for (uint8_t i = 0; i < m_count; ++i) {
            m_tasks[i].last_ms = 0;
            m_tasks[i].busy_us = 0;
        }
    }

private:
    struct Task {
        TaskFn fn;
        const char* name;
        uint32_t interval_ms;
        uint32_t last_ms;
        uint32_t busy_us;
    };

    Task m_tasks[MAX_TASKS]{};
    uint8_t m_count = 0;
};
