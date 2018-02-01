#include "exception.h"


Exception::Exception(const std::string &arg, const char *file, int line) :
        std::runtime_error(arg)
{
    std::ostringstream o;
    o << file << ":" << line << ": " << arg;
    msg = o.str();
}

Exception::~Exception() throw() {}

const char* Exception::what()  const throw() {
    return msg.c_str();
}
