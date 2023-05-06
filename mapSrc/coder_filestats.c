/* Copyright 2022 The MathWorks, Inc. */
#include <sys/types.h>
#include <sys/stat.h>

#ifdef _WIN32
#define stat _stat
#endif

#if !defined(S_ISREG) && defined(S_IFMT) && defined(S_IFREG)
#define S_ISREG(m) (((m)&S_IFMT) == S_IFREG)
#endif
#if !defined(S_ISDIR) && defined(S_IFMT) && defined(S_IFDIR)
#define S_ISDIR(m) (((m)&S_IFMT) == S_IFDIR)
#endif

int coderIsFileOrFolder(const char* path, const int checkFile) {

    struct stat sb;
    int isErr = stat(path, &sb);

    if (isErr) {
        return isErr;
    }

    if (checkFile) {
        return S_ISREG(sb.st_mode);
    } else {
        return S_ISDIR(sb.st_mode);
    }
}
