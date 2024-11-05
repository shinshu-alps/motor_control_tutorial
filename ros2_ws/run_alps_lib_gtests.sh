#!/bin/bash

# alps_libパッケージ内のテストを実行するスクリプト

directory="build/alps_lib"

# スクリプトのあるディレクトリに移動
cd "$(dirname "$0")" || { echo "ディレクトリの移動に失敗しました"; exit 1; }

# ビルド
if ! colcon build --symlink-install ; then
    echo "ビルドに失敗しました"
    exit 1
fi

# テストの実行
# 実行ファイル名が`*_test`であることが条件
for file in "$directory"/*_test; do
    if [ -f "$file" ] && [ -x "$file" ]; then
        if "$file"; then
            echo "テスト $file が成功しました"
        else
            echo "テスト $file が失敗しました"
            exit 1
        fi
    fi
done
