file(REMOVE_RECURSE
  "libnanogui.a"
  "libnanogui.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang C CXX)
  include(CMakeFiles/nanogui.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
