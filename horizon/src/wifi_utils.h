#pragma once
#include <functional>

void initWiFi(const std::function<void()> &disconnectedCallback,
              const std::function<void()> &connectedCallback,
              const std::function<void()> &gotIPCallback);
