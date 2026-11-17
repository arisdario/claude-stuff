#pragma once
// Matches thneed's input buffers (found inside SNPE, in SNPE's own order) to the runner's inputs BY SIZE.
// 0.8.3 assumed a fixed order {recurrent, traffic convention, desire, image}; with a different SNPE a wrong order would
// copy 1.5 MB out of an 8-byte buffer. Here: both lists are ranked by size and paired; SNPE may pad a buffer, so each
// of theirs must be at least ours and at most 4x + 256 bytes (otherwise it is not the same tensor).
// theirs[i] = bytes of thneed input i, ours[j] = bytes of our input j. On success perm[i] = j and true.
#include <stddef.h>

inline bool thneed_match_inputs(const size_t *theirs, int n_theirs, const size_t *ours, int n_ours, int *perm) {
  if (n_theirs != n_ours || n_ours <= 0 || n_ours > 8) return false;
  int a[8], b[8];
  for (int i = 0; i < n_ours; i++) { a[i] = i; b[i] = i; }
  for (int i = 1; i < n_ours; i++) {           // insertion sort, largest first
    for (int k = i; k > 0 && theirs[a[k]] > theirs[a[k - 1]]; k--) { int t = a[k]; a[k] = a[k - 1]; a[k - 1] = t; }
    for (int k = i; k > 0 && ours[b[k]] > ours[b[k - 1]]; k--) { int t = b[k]; b[k] = b[k - 1]; b[k - 1] = t; }
  }
  for (int r = 1; r < n_ours; r++) {
    if (ours[b[r]] == ours[b[r - 1]] || theirs[a[r]] == theirs[a[r - 1]]) return false;   // ambiguous
  }
  for (int r = 0; r < n_ours; r++) {
    const size_t t = theirs[a[r]], o = ours[b[r]];
    if (t < o || t > 4 * o + 256) return false;
    perm[a[r]] = b[r];
  }
  return true;
}

// Which of thneed's inputs receives the camera image (the runner's input 0): its index in `perm`, or -1 if none does.
// The runner never replays with -1 (it would index thneed_in[-1]); with a valid thneed_match_inputs() perm it cannot
// happen, this is the check that keeps it so.
inline int thneed_image_slot(const int *perm, int n) {
  for (int i = 0; i < n; i++) {
    if (perm[i] == 0) return i;
  }
  return -1;
}
