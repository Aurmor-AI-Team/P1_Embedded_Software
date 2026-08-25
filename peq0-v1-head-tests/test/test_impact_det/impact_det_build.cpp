// PlatformIO's build_src_filter cannot reach outside src_dir, so the module
// under test is pulled in here directly. The ESP-IDF headers it includes
// resolve to the shims in test/support (see the native env's build_flags).
#include "impact_det.cpp"
