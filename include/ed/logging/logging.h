#ifndef ED_LOGGING_H_
#define ED_LOGGING_H_

#include <ostream>

namespace ed
{

namespace log
{

void info(const std::string& str);

void warning(const std::string& str);

void error(const std::string& str);

}

}

#endif
