FILE(REMOVE_RECURSE
  "../src/coax_simple_control/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/coax_simple_control/SetNavMode.h"
  "../srv_gen/cpp/include/coax_simple_control/SetControlMode.h"
  "../srv_gen/cpp/include/coax_simple_control/SetWaypoint.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
