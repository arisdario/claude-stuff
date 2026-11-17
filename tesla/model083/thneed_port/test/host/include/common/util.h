#pragma once
// HOST TESTS ONLY: the one function of openpilot's common/util.h that runners/snpemodel.cc uses
#include <stdio.h>
#include <stdlib.h>
static inline void *read_file(const char *path, size_t *out_len) {
  FILE *f = fopen(path, "rb");
  if (!f) return NULL;
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  char *buf = (char *)malloc(n > 0 ? n + 1 : 1);
  size_t got = (buf && n > 0) ? fread(buf, 1, n, f) : 0;
  fclose(f);
  if (out_len) *out_len = got;
  return buf;
}
