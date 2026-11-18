# MachineCoordinator

## 概要

Arduino GPIO出力とXYZ移動など、独立した機器ライブラリを組み合わせるための
C++17オーケストレーション補助ライブラリです。ArduinoControllerやMarlinController、
BeltConveyorController、EasyMachineLanguageには依存しません。

## 現在の連携モデル

`runWithOutputs()`は指定したArduino出力を**動作の前に**有効にし、渡された同期動作を実行し、
成功時も例外時もArduino出力を停止します。MarlinControllerの同期`moveTo()`を渡すと、
XYZ移動中その出力が有効になります。移動後に出力する場合は`runOutputAfterMotion()`を使います。

```cpp
#include "GpioController.hpp"
#include "MarlinTransport.hpp"
#include "MachineCoordinator.hpp"
#include "MotionController.hpp"

// Create/connect the controllers and verify motion limits before this sequence.
machine::MachineCoordinator coordinator(
    [&arduino](const std::string& name, int value) {
        arduino.setOutput(name, value);
    },
    [&arduino] {
        arduino.stopAll();
    });

coordinator.runWithOutputs({{"indicator", 1}}, [&] {
    xyz.moveTo({100.0, 50.0, 20.0}, 600.0);
});
```

## 移動完了後にポンプを動かす

`runOutputAfterMotion()`は同期動作の完了を待ってからArduino出力を有効にし、指定時間後に全出力を停止します。
OSのスケジューリングの影響で、出力時間は指定より少し長くなる場合があります。
移動が失敗した場合はポンプを有効にせず、出力停止を試みます。出力開始に失敗した場合も停止を試みます。

```cpp
coordinator.runOutputAfterMotion(
    {"pump_driver", 1}, std::chrono::seconds(5), [&xyz] {
        xyz.moveTo(motion::Axis::X, 110.0, 600.0);
    });
```

この例は「X軸を座標110 mmへ移動し、到着後に5秒間ポンプのDIGITAL出力をHIGHにする」処理です。
MarlinControllerの座標が既知で、移動範囲・速度制限が実機に合わせて設定済みである必要があります。
時間だけでは水量は保証されません。ポンプと配管ごとに供給量を測定・校正してください。
Arduino出力には適切な外部ドライバーを使います。

Link the independent libraries from the consuming CMake project:

```cmake
add_subdirectory(/path/to/ArduinoController arduino-controller-build)
add_subdirectory(/path/to/MarlinController marlin-controller-build)
add_subdirectory(/path/to/MachineCoordinator machine-coordinator-build)

target_link_libraries(my_app PRIVATE
    ArduinoController::arduino_controller
    marlincontroller
    MachineCoordinator::machine_coordinator
)
```

開始・完了応答は各デバイスAPIで同期確認します。Arduino出力はXYZ動作と物理的に同時に
有効ですが、XYZ API自体は同期式です。`interruptOutputs()`はArduino出力をLOWにしますが、
実行中のXYZ移動を止めません。現在のMarlinController APIは移動中の安全な割り込みを保証
しないため、機械の非常停止回路を別途用意してください。

同時に複数の動作タスクを起動・join/cancelする汎用シーケンサーは現段階では含めません。
必要となる機器・キャンセル契約が明らかになった時点で、このプロジェクトに追加します。

## 実機接続アプリ

`BUILD_COORDINATED_EXAMPLE=ON`でArduinoController、MarlinController、EasyMachineLanguageも一緒にビルドし、
ASM形式の動作プログラムを実機で実行する`machine_coordinator_run`を生成します。

```sh
cmake -S . -B build -DBUILD_COORDINATED_EXAMPLE=ON
cmake --build build --target machine_coordinator_run
```

`make build`は統合アプリ、ArduinoController/MarlinController/EasyMachineLanguageのライブラリに加え、
UNO R4 Minima向けArduinoファームウェアもまとめてコンパイルします。
Arduino CLIが`PATH`にない場合は実行ファイルを指定します。

```sh
make build
```

Projects直下からなら`make machine`で同じ統合ビルドとテストを実行できます。
Arduino CLIはPATH上、または`$HOME/.local/bin/arduino-cli`にあるものを自動検出します。

ArduinoController用ファームウェアを書き込み、Arduinoの設定に`pump_driver DIGITAL <ピン番号>`を
登録します。設定ファイルの雛形をコピーし、Marlin/Arduinoのポートと実機の安全上限を編集します。
Arduino設定ファイルも別途用意してください。相対パスは設定ファイルのディレクトリから解決されます。

```sh
cp machine-coordinator.conf.example machine-coordinator.conf
cp ../ArduinoController/arduino-controller.conf.example \
   ../ArduinoController/arduino-controller.conf
# machine-coordinator.conf と ArduinoController/arduino-controller.conf を編集
```

ビルド後はMachineCoordinatorディレクトリで次の短いコマンドを実行します。

```sh
./build/machine_coordinator_run
```

既定の`machine-coordinator.conf`以外を使う場合だけ`--config`でパスを指定します。

```sh
./build/machine_coordinator_run --config /path/to/machine-coordinator.conf
```

MarlinとArduinoは別々のUSBシリアルポートを使います。起動直後、設定ファイルとASM全体を読み、
原点(X/Y/Z=0)を前提に軸範囲・送り速度・移動距離・所要時間などの上限を先に検証します。
この事前検証に失敗した場合は、Arduino/Marlinへ接続せず、原点復帰も行いません。
検証に通った場合だけArduino出力をLOWにし、アプリは対話確認を行ってから
ホーミングするかを尋ねます。最初の`[y/N]`で`y`または`Y`を入力すると
Marlinへ`G28`を送りXYZ全軸を原点復帰し、完了後に現在位置がX=0/Y=0/Z=0であることを確認します。
`n`、Enter、その他の入力ではホーミングを省略し、Marlinから現在位置を読み取ります。
ホーミングを省略した場合は、表示された座標が実機位置と一致するか確認します。
`N`またはEnterを選ぶと、機械を停止した状態で実測した`X Y Z`座標を入力できます。
入力した座標は移動を起こさず、Marlinの論理座標を`G92`で更新します。
このため、物理的な位置を確認できている場合に限って訂正値を入力してください。
どちらの場合も、確定した開始位置からASMを再検証した後、もう一度`[y/N]`で実行を確認します。
したがって、ホーミング確認で`N`を選んでも、次の実行確認へ進みます。
ホーミングを省略する場合、Marlinの報告座標は実機の物理位置や原点を保証しません。
正確な実位置を確認できない場合は座標を推測して入力せず、実行確認で`N`を選んでください。
ホーミング動作のため、機械の可動範囲を空け、非常停止をすぐ操作できる状態にしてください。
原点復帰がタイムアウトまたは失敗した場合はプログラムを実行せず、`M410`による停止要求を試みます。
停止応答は物理的な停止の保証ではないため、動きが続く場合は独立した非常停止を使用してください。

設定ファイルの移動範囲・速度は雛形の例であり、実機で確認済みの既定値ではありません。
必ず機械の仕様に合わせて編集してください。設定ファイルは実行時に読み込まれるため、
編集後に再ビルドする必要はありません。

アプリはASM全体を事前検証します。軸範囲、1移動ごとの距離・速度・所要時間、
DIGITAL出力名、ポンプ時間を検査し、検証に通らないプログラムは実行しません。
プログラム全体の移動距離の合計では制限しません。
DISPENSEは1回30秒以下、合計60秒以下・最大12回に制限されます。条件分岐の両方の経路が
異なる座標に終わるプログラムも拒否します。

ASMファイルの拡張子は`.asm`です。制御構文（`FOR` / `IF` / `ELSE` / `END`）と移動命令は
[BeltConveyorControllerのASM](../BeltConveyorController/docs/USAGE.md)と同じ書式です。
移動は`<ベルト名> X<mm> [Y<mm>] [Z<mm>] [F<mm/min>]`と書きます。
ベルト名は設定の`belt_name`（省略時は`A`）で、Fを省略すると
`max_feedrate_mm_min`が使われます。1行の複数軸は補間付きの同時移動です。
`ASSERT <ベルト名>.<X|Y|Z> <比較> <数値>`、`DISPENSE`はMachineCoordinator固有命令です。
例:

```text
ASSERT A.X == 0
A X110 F600
DISPENSE pump_driver 5000
A X0 F600
```

同梱の[fill-one-cup.asm](programs/fill-one-cup.asm)はX=0から始め、X=110 mmへ600 mm/minで移動し、
完了後`pump_driver`をHIGHで5秒間有効にして、最後にX=0へ戻ります。
各移動は`max_move_mm`以下、送り速度は`max_feedrate_mm_min`以下にしてください。
軸ごとの移動先は各軸の上限内である必要があります。このサンプルは開始時にX=0であることを要求します。
ポンプには適切な外部ドライバーを使い、配線・水量・停止状態を負荷を接続せずに先に確認してください。
Arduino UNO R4 Minima向けファームウェアのコンパイルは確認済みですが、ボードへの書き込み、
ポンプを含む実機動作は未確認です。
Marlin移動中の非常停止は保証されないため、独立した非常停止回路を使用してください。

### UNO R4 Minimaへのファームウェア書き込み

PC側の`make build` / `cmake --build`には、UNO R4用ファームウェアのコンパイルを含みますが、
実機への書き込みは含めません。
書き込みはUSB接続されたボードを書き換える操作なので、誤ポートへの書き込みや負荷の予期しない
動作を避けるため、明示的な専用ターゲットに分けています。書き込み前にポンプ・外部ドライバーを
切り離し、他のシリアルモニターを閉じてください。まずボードとポートを確認します。

```sh
$HOME/.local/bin/arduino-cli board list
```

UNO R4 Minimaが`/dev/ttyACM0`に表示された場合、次のコマンドで書き込みます。
ポートは実際の`board list`結果に合わせてください。

```sh
make upload-arduino \
  ARDUINO_FQBN=arduino:renesas_uno:minima \
  ARDUINO_UPLOAD_PORT=/dev/ttyACM0
```

同じ操作をCMakeから直接実行する場合:

```sh
cmake -S . -B build -DBUILD_COORDINATED_EXAMPLE=ON \
  -DARDUINO_CLI_EXECUTABLE="$HOME/.local/bin/arduino-cli" \
  -DARDUINO_FQBN=arduino:renesas_uno:minima \
  -DARDUINO_UPLOAD_PORT=/dev/ttyACM0
cmake --build build --target upload_arduino_firmware
```

このアップロードターゲットは通常の`make build`やデフォルトのCMakeビルドからは実行されません。
UNO R4 Minima用のFQBNは`arduino:renesas_uno:minima`です。

### ASM命令

```text
ASSERT A.X == 0
A X110 F600
DISPENSE pump_driver 5000
A X0 F600
```

- `ASSERT A.X == 0`: 開始時にX=0であることを検証（`A`は`belt_name`の設定値）
- `<ベルト名> X<mm> [Y<mm>] [Z<mm>] [F<mm/min>]`: ASMと同じ書式の同期移動。F省略時は設定の`max_feedrate_mm_min`
- `DISPENSE <DIGITAL出力名> <時間ms>`: HIGHにして指定時間後に全出力を停止（LOW）
- `WAIT <時間ms>`: 待機（最大60秒）
- `FOR` / `IF` / `ELSE` / `END`: EasyMachineLanguageのループ・条件構文
- `IF`条件はASMと同じ`A.X`形式（例:`IF A.X >= 110`）。旧形式の`position.x`も利用可能

制御構文の詳細は[EasyMachineLanguage利用ガイド](../EasyMachineLanguage/docs/USAGE.md)を参照してください。

## ビルドとテスト

```sh
make build
make test
```

テストはコールバックとシリアル疑似端末を使い、実機を操作しません。統合アプリのビルドは
次でも確認できます。

```sh
cmake -S . -B build -DBUILD_COORDINATED_EXAMPLE=ON
cmake --build build
```

ビルドは実機の接続や物理動作を検証しません。
