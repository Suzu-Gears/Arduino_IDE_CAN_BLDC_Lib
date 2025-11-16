# CANDemux - Virtual CAN Bus Library for Arduino

An Arduino library based on `arduino::HardwareCAN` that splits one CAN bus into multiple virtual buses by filtering CAN IDs.

CAN IDでフィルタリングすることで、1つのCANバスを複数の仮想バスに分割する`arduino::HardwareCAN` に基づくArduinoライブラリです。

## Features / 特徴

*   **Virtualization:** Treat a single physical CAN bus as multiple independent virtual CAN buses.
    *   **仮想化:** 単一の物理CANバスを、複数の独立した仮想CANバスとして扱えます。
*   **ID Filtering:** Each virtual bus can subscribe to specific CAN IDs or ID ranges.
    *   **IDフィルタリング:** 各仮想バスは、特定のCAN IDまたはID範囲を購読できます。
*   **Modular Code:** Simplifies code organization for projects with multiple CAN devices.
    *   **モジュール化されたコード:** 複数のCANデバイスを持つプロジェクトのコード構成を簡素化します。
*   **Overflow Policy:** Configurable message queue overflow handling (drop newest/oldest).
    *   **オーバーフローポリシー:** メッセージキューのオーバーフロー処理（最新を破棄/最古を破棄）を設定可能です。

## Installation / インストール

<!--
### Arduino IDE Library Manager / Arduino IDE ライブラリマネージャー

1.  Open the Arduino IDE. / Arduino IDE を開きます。
2.  Go to `Sketch > Include Library > Manage Libraries...`. / `スケッチ > ライブラリをインクルード > ライブラリを管理...` に移動します。
3.  Search for "CANDemux" and install the latest version. / "CANDemux" を検索し、最新バージョンをインストールします。
-->

### Manual Installation / 手動インストール

#### Using Arduino IDE's "Add .ZIP Library" / Arduino IDE の「.ZIPライブラリをインポート」を使用

1.  Download the latest release from the [GitHub repository](https://github.com/Suzu-Gears/CANDemux) as a `.zip` file.
    *   [GitHubリポジトリ](https://github.com/Suzu-Gears/CANDemux) から最新リリースを `.zip` ファイルでダウンロードします。
2.  In the Arduino IDE, go to `Sketch > Include Library > Add .ZIP Library...`.
    *   Arduino IDE で、`スケッチ > ライブラリをインクルード > .ZIPライブラリをインポート...` に移動します。
3.  Navigate to the downloaded `.zip` file and select it.
    *   ダウンロードした `.zip` ファイルを選択します。
4.  Restart the Arduino IDE. / Arduino IDE を再起動します。

#### Direct Placement / 直接配置

1.  Download the latest release from the [GitHub repository](https://github.com/Suzu-Gears/CANDemux).
    *   [GitHubリポジトリ](https://github.com/Suzu-Gears/CANDemux) から最新リリースをダウンロードします。
2.  Unzip the downloaded file and place the `CANDemux` folder into your Arduino libraries directory (e.g., `~/Documents/Arduino/libraries/`).
    *   ダウンロードしたファイルを解凍し、`CANDemux` フォルダをArduinoのライブラリディレクトリ（例: `~/Documents/Arduino/libraries/`）に配置します。
3.  Restart the Arduino IDE. / Arduino IDE を再起動します。

## Usage / 使い方

Here's a basic example of how to use `CANDemux` to create virtual CAN buses:

`CANDemux` を使って仮想CANバスを作成する基本的な例です。

```cpp
#include <Arduino_CAN.h> // Or your specific HardwareCAN library
#include <CANDemux.h>

// Assuming 'CAN' is your arduino::HardwareCAN instance
CANDemux canDemux(&CAN);

void setup() {
  // Initialize your physical CAN bus
  CAN.begin(CanBitRate::BR_1000k);

  // Create a virtual CAN bus that subscribes to ID 0x100
  VirtualCAN myVirtualBus = canDemux.createClientWithIds({ 0x100, 0x101, 0x102, 0x103 });
  // VirtualCAN myVirtualBus = canDemux.createClientWithRange(0x100, 4);

  // You can use myVirtualBus just like a regular HardwareCAN instance
  // For example, to set up another library that expects a HardwareCAN object:
  // someMotorDriver.setCAN(&myVirtualBus);
}

void loop() {
  // In your loop, make sure to poll the demultiplexer
  canDemux.poll();

  // Read messages from your virtual bus
  if (myVirtualBus.available()) {
    CanMsg msg = myVirtualBus.read();
    // Process message...
  }

  // Write messages to the physical bus through the demultiplexer
  CanMsg txMsg;
  txMsg.id = 0x200;
  txMsg.len = 8;
  // ... fill txMsg data ...
  canDemux.write(txMsg);
}
```

For a more comprehensive example, refer to the `multiCAN_Motor` example:
より包括的な例については、`multiCAN_Motor` サンプルを参照してください。
`CANDemux/examples/multiCAN_Motor/multiCAN_Motor.ino`

## API Reference / APIリファレンス

*   **`CANDemux`**: The main class that manages the physical `arduino::HardwareCAN` interface and distributes messages to `VirtualCAN` instances.
    *   物理的な `arduino::HardwareCAN` インターフェースを管理し、`VirtualCAN` インスタンスにメッセージを分配する主要クラスです。
*   **`VirtualCAN`**: Represents a virtual CAN bus. It inherits from `arduino::HardwareCAN` and can be used wherever a `HardwareCAN` object is expected.
    *   仮想CANバスを表すクラスです。`arduino::HardwareCAN` を継承しており、`HardwareCAN` オブジェクトが期待される場所で利用できます。

## License / ライセンス

This library is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

このライブラリはMITライセンスの下で公開されています。詳細は [LICENSE](LICENSE) ファイルを参照してください。

## Compatible Hardware / 対応ハードウェア

`CANDemux` は `arduino::HardwareCAN` インターフェースを実装している任意のボードで使用できます。以下に一般的な例を挙げます。

`CANDemux` can be used with any board that implements the `arduino::HardwareCAN` interface. Here are some common examples:

*   **Arduino Uno R4 WiFi / Minima:**
    *   Library: `<Arduino_CAN.h>` (Built-in)
    *   ライブラリ: `<Arduino_CAN.h>` (組み込み)

*   **ESP32:**
    *   Library: `<ESP32_TWAI.h>` (Available via Arduino IDE Library Manager)
    *   GitHub: [eyr1n/ESP32_TWAI](https://github.com/eyr1n/ESP32_TWAI)
    *   ライブラリ: `<ESP32_TWAI.h>` (Arduino IDE ライブラリマネージャーから入手可能)
    *   GitHub: [eyr1n/ESP32_TWAI](https://github.com/eyr1n/ESP32_TWAI)

*   **Raspberry Pi Pico (RP2040):**
    *   Library: `<RP2040PIO_CAN.h>` (Available via Arduino IDE Library Manager)
    *   GitHub: [eyr1n/RP2040PIO_CAN](https://github.com/eyr1n/RP2040PIO_CAN)
    *   ライブラリ: `<RP2040PIO_CAN.h>` (Arduino IDE ライブラリマネージャーから入手可能)
    *   GitHub: [eyr1n/RP2040PIO_CAN](https://github.com/eyr1n/RP2040PIO_CAN)
