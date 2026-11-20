# fanuc_lerobot_ros2
本リポジトリは，FANUC CRXを実機やIsaac Sim上で動かし，模倣学習を行うためのものです．FANUC CRXをROS2から制御し，LeRobotと連携することで，実機とシミュレーションの双方で統一された模倣学習システムの構築を目指します．

## Install
本リポジトリを任意のワークスペースにクローンしてください．
```
git clone https://github.com/ayakasakurai0213/fanuc_lerobot_ros2.git --recursive
```

## Create the environment
本リポジトリは，ROS2とLeRobotとIsaac Simを連携し，模倣学習を実行するためのDocker環境を提供します．

1. 以下のコマンドを実行して，Dockerコンテナを構築してください．ビルドには時間がかかります．
    ```
    cd fanuc_lerobot_ros2/docker
    docker compose build
    ```

2. 以下のコマンドを実行してdockerコンテナを起動します．
    ```
    docker compose up
    ```

3. 新しいターミナルを開き，以下のコマンドを実行してコンテナ内に接続します．
    ```
    docker exec -it fanuc_lerobot_ros2 bash
    ```

4. コンテナ内で以下のコマンドを実行し，ROS2パッケージをビルドします．
    ```
    ./build.sh
    ```

5. コンテナ内で以下のコマンドを実行し，conda環境にLeRobotに関連するパッケージをインストールします．
    ```
    conda activate lerobot-ros
    ./install.sh
    ```

## Control Robot using ROS2
本リポジトリは，FANUC CRXをROS2から制御するパッケージを提供します．以下のコマンドを実行することでMoveItからロボットアームを動かすことができます．<br>
実機ロボットを使用する場合は，引数に**robot_ip**を指定してください．Isaac Sim上のロボットを動かす場合は，引数に**sim_isaac**をTrueに指定してください．どちらも使用しない場合，use_mockをTrueに指定することでモックを使用することができ，Rviz上のみでロボットの動きを確認できます．

```
ros2 launch fanuc_moveit_config fanuc_moveit.launch.py robot_model:=crx5ia_2f_85gripper
```
**[args]**
- robot_model：FANUCのロボットのモデル
- robot_ip (eg: 192.168.1.100)：実機ロボットのIP
- sim_isaac (T/F)：Isaac Simを使用するかどうか
- use_mock (T/F)：mockを使用するかどうか


## ROS2 × LeRobot
本リポジトリは，ROS2とLeRobotを連携し模倣学習を実行するために必要なパッケージを提供します．以下では，実機で模倣学習を行う方法とIsaac Sim上のロボットで模倣学習を行う方法の2つを提示します．


### Actual
整備中


### Isaac Sim
Isaac Sim上のロボットにおいて，データ収集，学習，推論までの模倣学習の一連のフェーズを実行する方法を説明します．<br>
本リポジトリを用いて模倣学習を行うためには，別途Isaac Sim上でROS2と連携しFANUCのロボットが動く環境を用意する必要があります．[isaacsim_ros2_docker](https://github.com/ayakasakurai0213/isaacsim_ros2_docker/tree/jazzy)にIsaac Sim上でROS2 Jazzyを動かすDocker環境および，サンプルのUSD（[robot_data/crx5ia_2f_85gripper.usd](https://github.com/ayakasakurai0213/isaacsim_ros2_docker/tree/jazzy/robot_data)）が提供されているため，そちらを使用していただいてかまいません．


#### Teleop
データ収集の前段階として，ロボットのテレオペレーション方法について説明します．本リポジトリでは，キーボードを用いる方法と，Meta Quest 3などのVRコントローラを用いる方法の2つを提供します．<br>
Isaac Simを起動してロボットのシミュレーションを開始し，以下の手順に従ってテレオペレーションを行ってください．

**keyboardを用いる方法**<br>
1. 以下のコマンドを実行し，LeRobotのテレオペレーション機能を起動します．
    ```
    cd ~/lerobot_ws

    python scripts/teleop.py \
        --robot.type=fanuc_ros \
        --robot.id=fanuc_arm \
        --teleop.type=crx_keyboard_joint \
        --teleop.id=fanuc_arm \
        --display_data=true \
        --robot.use_isaac=true
    ```
    **[args]**
    - robot.type：使用するロボットの名前
    - robot_id：使用するロボットのID
    - teleop.type：使用するコントローラの名前（キーボードの場合はcrx_keyboard_joint）
    - teleop.id：使用するコントローラのID
    - display_data (T/F)：取得データを表示するかどうか
    - robot.use_isaac (T/F)：Isaac Simを使用するかどうか

2. キーボードを操作して，シミュレーション上のロボットが動くことを確認します．具体的なキー操作は以下の通りです．<br>
    **[キー操作]**

    |Joint名|キー操作|
    |:---|:---:|
    |J1|Q： -　A： +|
    |J2|W： -　S： +|
    |J3|E： -　D： +|
    |J4|R： -　F： +|
    |J5|T： -　G： +|
    |J6|Y： -　H： +|
    |gripper|O： -　L： +|


**VR controllerを用いる方法**<br>
VRコントローラを用いる方法では，Moveit Servoを活用することで，コントローラから得られる姿勢情報をロボットの動作指令へマッピングしています．実行手順は以下の通りです．

1. 以下のコマンドを実行し，Moveit Servoを起動します．
    ```
    ros2 launch fanuc_moveit_config moveit_servo.launch.py robot_model:=crx5ia_2f_85gripper sim_isaac:=true
    ```

2. Moveit Servoのからの動作指令をロボットへ送信するため，データ形式を変換する必要があります．新しいターミナルを開き，以下のコマンドを実行し，データ形式変換ノードを起動します．
    ```
    cd ~/lerobot_ws

    python scripts/servo_to_joint_bridge.py
    ```

3. 新しいターミナルを開き，以下のコマンドを実行して，LeRobotのテレオペレーション機能を起動します．
    ```
    cd ~/lerobot_ws

    python scripts/teleop.py \
        --robot.type=fanuc_ros \
        --robot.id=xr_fanuc_arm \
        --teleop.type=xr_ros_controller \
        --teleop.id=fanuc_arm \
        --display_data=true \
        --robot.use_isaac=true
    ```
    **[args]**
    - robot_id：使用するロボットのID（VRコントローラの場合は"xr"をつける）
    - teleop.type：使用するコントローラの名前（VRコントローラの場合はxr_ros_controller）

4. VRを起動し，Isaac Sim上のロボットと接続します．詳しい接続方法は，NVIDIA Isaac Captureの公式ドキュメント[IsaacCapture_connect-xr-headset](https://nvidia.github.io/IsaacCapture/main/teleop/quick_start.html#connect-an-xr-headset)を参照してください．


#### Record
データ収集は，基本的にテレオペレーションと同様の手順で実行することができます．teleop.pyを実行していたコマンドを以下のものに変更することで，データ収集を実行できます．<br>
本データ収集スクリプトは，LeRobotのものをベースとしているため，LeRobotと同様の方法でデータを保存できます．詳しい操作はHugging Faseが提供するLeRobotの公式ドキュメント[Keyboard Controls During Recording](https://huggingface.co/docs/lerobot/il_robots#4-keyboard-controls-during-recording)を参照してください．

```
python scripts/record.py \
    --robot.type=fanuc_ros \
    --robot.id=fanuc_arm \
    --teleop.type=crx_keyboard_joint \
    --teleop.id=fanuc_arm \
    --display_data=true \
    --robot.use_isaac=true \
    --dataset.repo_id=/test \
    --dataset.num_episodes=5 \
    --dataset.single_task="test" \
    --dataset.root=./datasets/test \
    --dataset.push_to_hub=false
```
**[args]**
- robot.type：使用するロボットの名前
- robot_id：使用するロボットのID
- teleop.type：使用するコントローラの名前（キーボードの場合はcrx_keyboard_joint）
- teleop.id：使用するコントローラのID
- display_data (T/F)：取得データを表示するかどうか
- robot.use_isaac (T/F)：Isaac Simを使用するかどうか
- dataset.repo_id：Hugging Fase HubのデータセットのリポジトリID
- dataset.num_episodes：収集するエピソード数
- dataset.single_task：タスク名
- dataset.root：データセットを保存するディレクトリパス
- dataset.push_to_hub (T/F)：収集したデータセットをHugging Fase Hubにアップロードするかどうか

#### Replay data
収集したLeRobotのデータセットに正しい動作が収集できているかどうかを，エピソードを再生することで確認できます．

1. Isaac Simを起動してロボットのシミュレーションを開始してください．

2. 以下のコマンドを実行し，データの再現を開始してください．
    ```
    cd ~/lerobot_ws

    python scripts/replay.py \
        --repo_id test \
        --root datasets/test/ \
        --episode_num 1 \
        --use_isaac True
    ```
    **[args]**
    - repo_id：Hugging Fase HubのデータセットのリポジトリID
    - root：LeRobotのデータセットが保存されているディレクトリパス
    - episode_num：再生したいエピソードの番号
    - use_isaac (T/F)：Isaac Simを使用するかどうか


#### Train
以下のコマンドを実行して，LeRobotの学習を行うことができます．パラメータなどさらに詳しい情報はHugging Faseが提供するLeRobotの公式ドキュメント[Train a policy](https://huggingface.co/docs/lerobot/il_robots#train-a-policy)を参照してください．
```
lerobot-train \
    --policy.type=act \
    --dataset.repo_id="test" \
    --dataset.root="datasets/test" \
    --output_dir="outputs/train/test" \
    --batch_size=8 \
    --steps=300000 \
    --policy.push_to_hub False
```
**[args]**
- policy.type：使用する模倣学習ポリシーの名前
- dataset.repo_id：Hugging Fase HubのデータセットのリポジトリID
- dataset.root：LeRobotのデータセットが保存されているディレクトリパス
- output_dir：学習後のモデルを保存するディレクトリパス
- batch_size：バッチサイズ
- steps：ステップ数
- policy.push_to_hub (T/F)：学習後のモデルをHugging Fase Hubにアップロードするかどうか


#### Eval
1. Isaac Simを起動してロボットのシミュレーションを開始してください．

2. 以下のコマンドを実行し，推論を開始してください．
    ```
    cd ~/lerobot_ws

    python scripts/eval.py \
        --ckpt_path outputs/train/test/checkpoints/last/pretrained_model/ \
        --policy act \
        --use_isaac True
    ```
    **[args]**
    - ckpt_path：使用する学習済みモデルのパス
    - policy：使用する模倣学習ポリシーの名前
    - use_isaac (T/F)：Isaac Simを使用するかどうか