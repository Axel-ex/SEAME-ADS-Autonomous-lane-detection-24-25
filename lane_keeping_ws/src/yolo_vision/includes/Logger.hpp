#pragma once

#include <NvInfer.h>

class Logger : public nvinfer1::ILogger
{
        void log(Severity severity, const char* msg) noexcept override;
};
