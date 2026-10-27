# V2X com2obu 静态场景测试

本目录用于在 **RSU / OBU 实物不移动** 的情况下，通过 MQTT `com2obu` 修改 OBU 的逻辑位置、航向和速度，并配合 RSU 下发的 RTE / RTS / RSM 数据验证 OBU 的 `event2hmi` 场景触发。

## 目录

- `01_hlw/` 道路危险状况 / 道路湿滑，预期 HLW，event_sort=8
- `02_ivs/` 车内标牌 / 施工提醒，预期 IVS，event_sort=13
- `03_jam/` 前方拥堵，预期 HLW，event_sort=8，event_type=43
- `04_avw/` 异常车辆提醒，预期 AVW，event_sort=6
- `05_ebw/` 紧急制动预警，预期 EBW，event_sort=5
- `06_fcw/` 前向碰撞预警，预期 fFCW，event_sort=0
- `07_vrucw/` 弱势交通参与者 / 行人碰撞，预期 VRUCW，event_sort=10

公共 MQTT / 坐标插值逻辑在 `common.py`。

## 默认 OBU MQTT

- IP: `192.168.66.113`
- Port: `1883`
- Username: `dctek`
- Password: `123456`
- Topic: `com2obu`

## 使用方式

先向 RSU 下发对应场景 JSON，再运行对应 Python：

```bash
python3 01_hlw/com2obu_hlw.py
python3 02_ivs/com2obu_ivs.py
python3 03_jam/com2obu_jam.py
python3 04_avw/com2obu_avw.py
python3 05_ebw/com2obu_ebw.py
python3 06_fcw/com2obu_fcw.py
python3 07_vrucw/com2obu_vrucw.py
```

同时监听：

```bash
mosquitto_sub -h 192.168.66.113 -p 1883 -u dctek -P 123456 -t event2hmi -v
```

> 注意：前三个场景是已知的 RSI(RTE/RTS) 类触发方式；后四个依赖 OBU 固件内部的 RSM 场景算法阈值，JSON 和主车轨迹用于构造触发条件，实际阈值以设备固件为准。
