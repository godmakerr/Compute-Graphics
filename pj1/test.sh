#!/bin/bash

# 设置目录路径
SWP_DIR="swp"
EXEC="./build/a1"

# 检查可执行文件是否存在
if [ ! -f "$EXEC" ]; then
    echo "错误：$EXEC 不存在，请先编译项目。"
    exit 1
fi

# 遍历所有 .swp 文件
for file in "$SWP_DIR"/*.swp; do
    echo ">>> 测试中: $file"
    $EXEC "$file"
    echo "--------------------------------------------------"
done
