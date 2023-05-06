/* Copyright 2022 The MathWorks, Inc. */
#ifndef CODER_FILESTATS_H
#define CODER_FILESTATS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief checks if input path is file or folder.
 * @param path Path as relative path from current directory or absolute path.
 * @param checkFile 1 to check for regular file, 0 for folder.
 * @return int 1/0 if file or folder, -1 if file not found, 22 if invalid input.
 */
int coderIsFileOrFolder(const char* path, const int checkFile);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CODER_FILEOPS_H */
