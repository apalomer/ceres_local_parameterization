#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <iostream>
#include <sstream>
#include <stdexcept>

class Exception : public std::runtime_error {
    std::string msg;
public:
    Exception(const std::string &arg, const char *file, int line) ;
    ~Exception() throw();
    const char *what()  const throw();
};

#define EXCEPTION(arg) throw Exception(arg, __FILE__, __LINE__);

#endif // EXCEPTION_H
