# alps_dev_base

[![ROS2 Build & Test](https://github.com/shinshu-alps/alps_dev_base/actions/workflows/ros2_build_and_test.yml/badge.svg)](https://github.com/shinshu-alps/alps_dev_base/actions/workflows/ros2_build_and_test.yml)
[![PIO Build](https://github.com/shinshu-alps/alps_dev_base/actions/workflows/pio_build.yml/badge.svg)](https://github.com/shinshu-alps/alps_dev_base/actions/workflows/pio_build.yml)

ロボ研の基本開発環境  
運用については以下のesa記事参照
> [ロボ研での基本開発環境 - roblab-shinshu.esa.io](https://roblab-shinshu.esa.io/posts/2093)

## クローン

sshでクローンする．またサブモジュールを含めるため`--recurse-submodules`オプションを付ける．

```bash
git clone --recurse-submodules git@github.com:shinshu-alps/alps_dev_base.git
```

## ディレクトリ構成

```tree
alps_dev_base/
├── README.md                           - このファイル
│
├── .clang-format                       - C++フォーマット設定
├── .flake8                             - Pythonフォーマット設定
├── .github                             
│   └── workflows                       - GitHub Actionsワークフロー
│       ├── pio_build.yml                   - PlatformIOビルドCI
│       └── ros2_build_and_test.yml         - ROS2ビルド&テストCI
├── .gitmodules
├── .vscode
│   ├── c_cpp_properties.json           - VSCode C/C++設定
│   └── extensions.json                 - VSCode推奨拡張機能
│
├── alps_dev_base.code-workspace        - VSCodeワークスペース設定
│
├── alps_core                           - ソフト共通リソース（submodule）
│
├── pio_project                         - PlatformIOプロジェクト
│   ├── include
│   ├── lib                             - ライブラリ置き場（シンボリックリンク）
│   ├── mbed_app.json                   - Mbed設定
│   ├── platformio.ini                  - PlatformIO設定
│   ├── src
│   │   ├── led_blink.cpp
│   │   ├── sample_pio_codes            - サンプルコード置き場(シンボリックリンク)
│   │   └── tools                       - ツール用コード置き場
│   └── test
│   
└── ros2_ws
    ├── .clang-format
    ├── .gitignore
    └── src
        ├── alps_ros2_pkgs/         - ロボ研共通ROS2パッケージ（シンボリックリンク）
        ├── behaviortree_ros2       - BehaviorTree.CPPのROS2パッケージ
        ├── cpp_pubsub
        └── sample_ros2_pkgs        - サンプルコード置き場(シンボリックリンク)
```
