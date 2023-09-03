#include "common.h"

#if NN_SAFE
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#endif

// https://stackoverflow.com/a/253874
int float_eq(float a, float b) {
  // return fabs(a - b) <= ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) *
  // FLT_EPSILON);
  return fabs(a - b) <= ((fabs(a) > fabs(b) ? fabs(b) : fabs(a)) * FLT_EPSILON);
  // return fabs(a - b) <= ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) *
  // FLT_EPSILON);
}

// backtrace* is not a libc standard.
// macOS has it, glibc has it (but we can't check at compile time)
#if defined(__MACH__) && defined(__APPLE__) && NN_SAFE
#define HAS_BACKTRACE
#include <execinfo.h>
#endif

#if NN_SAFE
// From:
// https://github.com/rsms/compis/blob/3946ed5c1da01addf5b07faa872cc6463df2521d/src/panic.c#L22
void _panic(const char *file, int line, const char *fun, const char *fmt, ...) {
  FILE *fp = stderr;
  flockfile(fp);

  fprintf(fp, "\npanic: ");

  va_list ap;
  va_start(ap, fmt);
  vfprintf(fp, fmt, ap);
  va_end(ap);

  fprintf(fp, " (%s at %s:%d)\n", fun, file, line);

#ifdef HAS_BACKTRACE
  void *buf[32];
  int framecount = backtrace(buf, countof(buf));
  if (framecount > 1) {
    char **strs = backtrace_symbols(buf, framecount);
    if (strs != NULL) {
      for (int i = 1; i < framecount; ++i) {
        fwrite(strs[i], strlen(strs[i]), 1, fp);
        fputc('\n', fp);
      }
      free(strs);
    } else {
      fflush(fp);
      backtrace_symbols_fd(buf, framecount, fileno(fp));
    }
  }
#endif

  funlockfile(fp);
  fflush(fp);
  fsync(STDERR_FILENO);

  abort();
}
#endif
