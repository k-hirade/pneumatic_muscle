// 必要なヘッダーファイルをインクルード
#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>

// バルブ制御用のGPIOピン番号を定義
#define pin_spi_cs1 10  // 1_19=51
#define pin_spi_other 0 // 0_2=2
#define pin_spi_mosi 11 // 3_15=112
#define pin_spi_sclk 13 // 0_3=3
#define pin_spi_cs2 9   // 0_7 =7
#define NUM_OF_CHANNELS 16

// SPI通信のクロックエッジを管理するための変数
bool clock_edge = false;

// データの解像度を指定
unsigned short resolution = 0x0FFF;

// 各GPIOピンを制御する関数・digitalWrite関数を使用して、特定のGPIOピンに値を設定
void set_SCLK(bool value) { digitalWrite(pin_spi_sclk, value); }
void set_OTHER(bool value) { digitalWrite(pin_spi_other, value); }
void set_MOSI(bool value) { digitalWrite(pin_spi_mosi, value); }
void setCS1(bool value) { digitalWrite(pin_spi_cs1, value); }
void setCS2(bool value) { digitalWrite(pin_spi_cs2, value); }

// SPI通信のクロックエッジを設定するための関数
void set_clock_edge(bool value) { clock_edge = value; }

// MISO（Master In Slave Out）ピンの状態を取得するダミー関数。実際の値は取得しない
bool get_MISO(void) { return false; }

// SPI通信の待機時間を管理する関数。
void wait_SPI(void) {}

// チップセレクト1の制御を行う関数です。指定された値を反転させてCS1ピンに設定し、SPI通信の待機時間を挿入します。
void chipSelect1(bool value)
{
  setCS1(!value);
  wait_SPI();
  wait_SPI();
}

// チップセレクト2の制御を行う関数です。指定された値を反転させてCS2ピンに設定し、SPI通信の待機時間を挿入します。
void chipSelect2(bool value)
{
  setCS2(!value);
  wait_SPI();
  wait_SPI();
}
// 8ビットデータを送信し、応答を受け取る関数です。
unsigned char transmit8bit(unsigned char output_data)
{
  unsigned char input_data = 0;
  int i;
  for (i = 7; i >= 0; i--)
  {
    // MOSIピン（Master Out Slave In）にデータを送信し、MISOピン（Master In Slave Out）からデータを受け取ります。
    // ここでクロックエッジに応じてSCLK（シリアルクロック）を設定します。
    set_SCLK(!clock_edge);
    set_MOSI((bool)((output_data >> i) & 0x01));
    input_data <<= 1;
    wait_SPI();
    set_SCLK(clock_edge);
    input_data |= get_MISO() & 0x01;
    wait_SPI();
  }
  return input_data;
}

// 16ビットデータを送信し、応答を受け取る関数です。
unsigned short transmit16bit(unsigned short output_data)
{
  unsigned char input_data_H, input_data_L;
  unsigned short input_data;
  input_data_H = transmit8bit((unsigned char)(output_data >> 8));
  input_data_L = transmit8bit((unsigned char)(output_data));
  input_data = (((unsigned short)input_data_H << 8) & 0xff00) | (unsigned short)input_data_L;
  return input_data;
}

// DAレジスタにデータを設定する関数です。チャンネル番号とDACデータを受け取り、対応するレジスタに値を設定します。
void setDARegister(unsigned char ch, unsigned short dac_data)
{
  unsigned short register_data;

  // チャンネル番号に基づいて適切なレジスタにデータを設定します。
  if (ch < 8)
  {
    register_data = (((unsigned short)ch << 12) & 0x7000) | (dac_data & 0x0fff);
    chipSelect1(true);
    transmit16bit(register_data);
    chipSelect1(false);
  }
  else if (ch >= 8)
  {
    register_data = (((unsigned short)(ch & 0x0007) << 12) & 0x7000) | (dac_data & 0x0fff);
    chipSelect2(true);
    transmit16bit(register_data);
    chipSelect2(false);
  }
}
// 特定のチャンネルに圧力係数を設定するための関数です。
void setState(unsigned int ch, double pressure_coeff)
{
  // 圧力係数に解像度を乗じて、適切な値を計算し、DACレジスタに設定します。
  setDARegister(ch, (unsigned short)(pressure_coeff * resolution));
}

// SPI通信に使用するピンの初期化を行う関数です。
void init_pin_valves()
{
  // 各GPIOピンを出力モードに設定します。
  pinMode(pin_spi_cs1, OUTPUT);
  pinMode(pin_spi_other, OUTPUT);
  pinMode(pin_spi_mosi, OUTPUT);
  pinMode(pin_spi_sclk, OUTPUT);
  pinMode(pin_spi_cs2, OUTPUT);
}

// SPI通信に使用するピンの初期状態を設定する関数です。
void init_pins()
{
  // 各ピンの初期値を設定します。
  set_SCLK(LOW);
  set_MOSI(LOW);
  set_OTHER(LOW);
  setCS1(HIGH);
  setCS2(HIGH);
}

// AD5328 DACの初期化を行う関数です。
void init_DAConvAD5328(void)
{
  // SPI通信のクロックエッジを負に設定します。
  set_clock_edge(false);

  // チップセレクト1を使ってDACを初期化します。
  chipSelect1(true);
  transmit16bit(0xa000); // コントロールコマンド
  chipSelect1(false);
  chipSelect1(true);
  transmit16bit(0x8003); // 別のコントロールコマンド
  chipSelect1(false);

  // チップセレクト2を使って別のDACを初期化します。
  chipSelect2(true);
  transmit16bit(0xa000);
  chipSelect2(false);
  chipSelect2(true);
  transmit16bit(0x8003);
  chipSelect2(false);
}
class SerialReader
{
public:
  // コンストラクタです。特別な初期化処理は行いません。
  explicit constexpr SerialReader() {}

  // シリアルポートから整数を読み取るメソッドです。
  int readInt() const
  {
    char buf[40] = {0};                     // データを格納するバッファです。
    wait_();                                // データが利用可能になるまで待機します。
    int ret = 0;                            // 返却する整数値です。
    int size = min(40, Serial.available()); // 読み取るデータのサイズを決定します。

    // シリアルポートからデータを読み取ります。
    for (int i = 0; i < size; ++i)
    {
      buf[i] = (char)Serial.read();
    }

    // 数字の開始位置を探します。
    int now = 0;
    char c = buf[now];
    while ((c < '0' || '9' < c) && c != '-')
    {
      c = buf[++now];
    }

    // 負の値かどうかを判定します。
    const bool f = (c == '-') && (c = buf[++now]);

    // 数字を解析して整数値に変換します。
    while (now + 1 < size)
    {
      ret = 10 * ret + c - '0';
      c = buf[++now];
    }

    return f ? -ret : ret; // 読み取った整数値を返却します。
  }

  // シリアルポートから浮動小数点数を読み取るメソッドです。
  float readFloat() const
  {
    char buf[40] = {0};
    wait_();
    int size = min(40, Serial.available());
    for (int i = 0; i < size; ++i)
    {
      buf[i] = (char)Serial.read();
    }
    int dot_idx = size - 1;
    for (int i = 0; i < size; ++i)
    {
      if (buf[i] == '.')
      {
        dot_idx = i;
        break;
      }
    }
    float ret = 0;
    float e = pow(10, dot_idx - 1);
    int now = 0;
    char c = buf[now];
    const bool f = (buf[now] == '-') && (c = buf[++now], e /= 10);
    while (now + 1 < size)
    {
      if (c != '.')
      {
        ret += (c - '0') * e;
        e /= 10;
      }
      c = buf[++now];
    }
    return f ? -ret : ret;
  }

private:
  // シリアルポートからデータが利用可能になるまで待機するメソッドです。
  void wait_() const
  {
    while (Serial.available() == 0)
    {
      ;
    }
    delay(50); // 小さな遅延を挿入してデータが安定するのを待ちます。
  }
};

// SerialReaderクラスのインスタンスを作成します。
constexpr SerialReader Reader;

// Arduinoの初期設定を行うsetup関数です。
void setup()
{
  Serial.begin(115200); // シリアル通信を115200ボーで開始します。
  init_pin_valves();    // バルブ制御用のピンを初期化します。
  init_pins();          // その他のピンを初期化します。
  init_DAConvAD5328();  // AD5328 DACを初期化します。

  // すべてのチャンネルを初期化します。
  int i;
  for (i = 0; i < NUM_OF_CHANNELS; i++)
  {
    setState(i, 0.0); // 各チャンネルの圧力係数を0に設定します。
    delay(100);       // 少し待機します。
  }
}

// 継続的な処理を行うloop関数です。
void loop()
{
  // 8チャンネル分の圧力係数を格納する変数を初期化します。
  float p1 = 0, p2 = 0, p3 = 0, p4 = 0, p5 = 0, p6 = 0, p7 = 0, p8 = 0;

  // シリアルポートから8バイトのデータが利用可能になるまで待機します。
  while (Serial.available() >= 8)
  {
    int i;
    float pp[8]; // 受信した生データを格納します。
    float p[8];  // 変換された圧力係数を格納します。

    // 8チャンネル分のデータを読み取り、圧力係数に変換します。
    for (i = 0; i < 8; i++)
    {
      pp[i] = Serial.read(); // 生データを読み取ります。
      p[i] = pp[i] / 255;    // 0から1の範囲に正規化します。
      setState(i, p[i]);     // 設定された圧力係数をDACに設定します。
    }

    delay(50); // 少し待機します。
  }
}

// 通信方法と信号の流れ
// SPI通信の設定:
// このコードは、ArduinoのGPIOピンを使用してSPI通信を手動で実装しています。
// pin_spi_cs1, pin_spi_cs2, pin_spi_mosi, pin_spi_sclkは、それぞれSPI通信のためのチップセレクト1、チップセレクト2、MOSI（Master Out Slave In）、SCLK（シリアルクロック）に対応しています。

// データの送信:
// transmit8bit と transmit16bit 関数は、8ビットまたは16ビットのデータをD/A変換基板に送信するために使用されます。
// これらの関数は、ビットごとにデータをMOSIピンに送り、SCLKピンをトグルすることでデータをシフトします。

// D/A変換基板へのデータ送信:
// setDARegister 関数は、特定のチャンネルにDACデータを設定するために使用されます。
// この関数は、チャンネル番号とDACデータを受け取り、それを適切なフォーマットに変換してD/A変換基板に送信します。

// 圧力比例制御弁の制御:
// setState 関数は、特定のチャンネルに圧力係数を設定するために使用されます。
// この関数は、圧力係数をDACデータに変換し、それをsetDARegister 関数を通じてD/A変換基板に送信します。

// 人工筋肉の操作:
// DACからのアナログ出力は、圧力比例制御弁を通じてPAMに圧力を供給するために使用されます。
// これにより、PAMの伸縮が制御され、結果として電気自動車の充電プラグの抜き差しが行われます。

// 回路の概要
// Arduinoは、SPI通信を介してD/A変換基板にデジタル信号を送信します。
// D/A変換基板は、このデジタル信号をアナログ信号に変換し、圧力比例制御弁に送ります。
// 圧力比例制御弁は、受け取ったアナログ信号に基づいてPAMに適切な圧力を供給します。
// PAMは、供給された圧力に応じて伸縮し、充電プラグの操作を行います。

// 1. ArduinoからD/A変換基板へのデジタル信号の送信
// init_pin_valves, init_pins, init_DAConvAD5328:
// これらの関数は、SPI通信に使用されるピンの初期化と設定を行います。これにより、ArduinoがD/A変換基板と通信するための準備が整います。
// transmit8bit, transmit16bit:
// これらの関数は、8ビットまたは16ビットのデータをD/A変換基板に送信するために使用されます。これにより、ArduinoからD/A変換基板へのデジタル信号の送信が行われます。

// 2. D/A変換基板によるアナログ信号への変換
// setDARegister:
// この関数は、特定のチャンネルにDACデータを設定するために使用されます。これにより、D/A変換基板は受け取ったデジタル信号をアナログ信号に変換します。

// 3. 圧力比例制御弁によるPAMへの圧力供給
// setState:
// この関数は、特定のチャンネルに圧力係数を設定するために使用されます。圧力係数はDACデータに変換され、D/A変換基板を通じて圧力比例制御弁に送信されます。その結果、弁はPAMに適切な圧力を供給します。

// 4. PAMによる充電プラグの操作
// このコードには、PAMの伸縮を直接制御する関数は含まれていません。しかし、setState 関数によって設定された圧力は、PAMを動作させ、結果として充電プラグの操作が行われます。
