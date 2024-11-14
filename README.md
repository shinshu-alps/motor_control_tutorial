> [!NOTE]
> 整備中．．．

# motor_control_tutorial

[![ROS2 Build & Test](https://github.com/shinshu-alps/motor_control_tutorial/actions/workflows/ros2_build_and_test.yml/badge.svg)](https://github.com/shinshu-alps/motor_control_tutorial/actions/workflows/ros2_build_and_test.yml)
[![PIO Build](https://github.com/shinshu-alps/motor_control_tutorial/actions/workflows/pio_build.yml/badge.svg)](https://github.com/shinshu-alps/motor_control_tutorial/actions/workflows/pio_build.yml)

モーター制御チュートリアル用リポジトリ  

## クローン

sshでクローンする．またサブモジュールを含めるため`--recurse-submodules`オプションを付ける．

```bash
git clone --recurse-submodules git@github.com:shinshu-alps/motor_control_tutorial.git
```

## ディレクトリ構成

```tree
motor_control_tutorial/
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
├── motor_control_tutorial.code-workspace        - VSCodeワークスペース設定
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
