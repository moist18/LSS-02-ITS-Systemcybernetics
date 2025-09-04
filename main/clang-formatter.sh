#!/bin/bash

echo "🔧 Formatting all C++ files with clang-format..."
find . -name "*.cpp" -exec clang-format -i {} \;
echo "✅ Done!"