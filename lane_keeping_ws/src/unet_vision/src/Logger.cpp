#include <Logger.hpp>
#include <iostream>

void Logger::log(Severity severity, const char* msg) noexcept
{
    std::cerr << "[TensorRT] ";
    switch (severity)
    {
    case Severity::kINTERNAL_ERROR:
        std::cerr << "[ERROR] ";
        break;
    case Severity::kERROR:
        std::cerr << "[ERROR] ";
        break;
    case Severity::kWARNING:
        std::cerr << "[WARNING] ";
        break;
    case Severity::kINFO:
        std::cerr << "[INFO] ";
        break;
    case Severity::kVERBOSE:
        std::cerr << "[VERBOSE] ";
        break;
    default:
        break;
    }
    std::cerr << msg << std::endl;
}
