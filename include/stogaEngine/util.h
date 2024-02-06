#ifndef UTIL_SE_H
#define UTIL_SE_H

void sleep();

void __debug(int line, const char* file, const char* message);

inline void debug(const char* message) {
    int line = __LINE__;
    const char* file = __FILE__;
    __debug(line, file, message);
};

#endif // UTIL_SE_H