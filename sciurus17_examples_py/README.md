# sciurus17_examples_py

このパッケージはSciurus17 ROS 2パッケージのPythonサンプルコード集です。

- [sciurus17\_examples\_py](#sciurus17_examples_py)
  - [準備（実機を使う場合）](#準備実機を使う場合)
    - [1. Sciurus17本体をPCに接続する](#1-sciurus17本体をpcに接続する)
    - [2. USB通信ポートの接続を確認する](#2-usb通信ポートの接続を確認する)
    - [3. move\_groupとcontrollerを起動する](#3-move_groupとcontrollerを起動する)
  - [準備 (Gazeboを使う場合)](#準備-gazeboを使う場合)
    - [1. move\_groupとGazeboを起動する](#1-move_groupとgazeboを起動する)
  - [準備（Mock Componentsを使う場合）](#準備mock-componentsを使う場合)
    - [1. move\_groupとcontrollerを起動する](#1-move_groupとcontrollerを起動する)
  - [サンプルプログラムを実行する](#サンプルプログラムを実行する)
    - [Gazeboでサンプルプログラムを実行する場合](#gazeboでサンプルプログラムを実行する場合)
  - [Examples](#examples)
    - [gripper\_control](#gripper_control)
    - [neck\_control](#neck_control)
    - [waist\_control](#waist_control)
    - [pick\_and\_place\_right\_arm\_waist](#pick_and_place_right_arm_waist)
    - [pick\_and\_place\_left\_arm](#pick_and_place_left_arm)


## 準備（実機を使う場合）

### 1. Sciurus17本体をPCに接続する
Sciurus17本体をPCに接続します。
接続方法は製品マニュアルを参照してください。

**※Sciurus17本体が接触しないように、十分なスペースを確保してください。**

### 2. USB通信ポートの接続を確認する

USB通信ポートの設定については`sciurus17_control`の
[README](../sciurus17_control/README.md)
を参照してください。

**正しく設定できていない場合、Sciurus17が動作しないので注意してください**

### 3. move_groupとcontrollerを起動する

次のコマンドでmove_group (`sciurus17_moveit_config`)と
controller (`sciurus17_control`)を起動します。

```sh
ros2 launch sciurus17_examples demo.launch.py
```

## 準備 (Gazeboを使う場合)

### 1. move_groupとGazeboを起動する

次のコマンドでmove_group (`sciurus17_moveit_config`)と
Gazeboを起動します。

```sh
ros2 launch sciurus17_gazebo sciurus17_with_table.launch.py
```

頭部カメラや胸部カメラのシミュレーションを行わない場合は、
`use_head_camera`、`use_chest_camera`オプションを`false`に設定します。

```sh
ros2 launch sciurus17_gazebo sciurus17_with_table.launch.py use_head_camera:=false use_chest_camera:=false
```

## 準備（Mock Componentsを使う場合）

### 1. move_groupとcontrollerを起動する

次のコマンドでmove_group (`sciurus17_moveit_config`)と
controller (`sciurus17_control`)を起動します。

```sh
ros2 launch sciurus17_examples demo.launch.py use_mock_components:=true
```

Mock Componentsではカメラを使ったサンプルを実行することはできません。

## サンプルプログラムを実行する

準備ができたらサンプルプログラムを実行します。
例えばグリッパを開閉するサンプルは次のコマンドで実行できます。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='gripper_control'
```

終了するときは`Ctrl+c`を入力します。

### Gazeboでサンプルプログラムを実行する場合

Gazeboでサンプルプログラムを実行する場合は`use_sim_time`オプションを付けます。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='gripper_control' use_sim_time:='true'
```

## Examples

`demo.launch`を実行している状態で各サンプルを実行できます。

- [gripper\_control](#gripper_control)
- [neck\_control](#neck_control)
- [waist\_control](#waist_control)
- [pick\_and\_place\_right\_arm\_waist](#pick_and_place_right_arm_waist)
- [pick\_and\_place\_left\_arm](#pick_and_place_left_arm)

実行できるサンプルの一覧は、`example.launch.py`にオプション`-s`を付けて実行することで表示できます。

```sh
ros2 launch sciurus17_examples_py example.launch.py -s
```

### gripper_control

ハンドを開閉させるコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='gripper_control'
```

[back to example list](#examples)

---

### neck_control

首を上下左右へ動かすコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='neck_control'
```

[back to example list](#examples)

---

### waist_control

腰を左右へひねる動作をするコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='waist_control'
```

[back to example list](#examples)

---

### pick_and_place_right_arm_waist

右手でターゲットを掴んで動かすコード例です。腰の回転も使用します。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='pick_and_place_right_arm_waist'
```

[back to example list](#examples)

---

### pick_and_place_left_arm

左手でターゲットを掴んで動かすコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='pick_and_place_left_arm'
```

[back to example list](#examples)

---
