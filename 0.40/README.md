# 0.40

ESP32-S3R8N8 平衡轮腿可分离机械狗固件 + 配套 App。

## 目录结构

```
0.40/
├── main/          # 入口
├── hardware/      # 硬件外设驱动 (每个外设独立子文件夹)
├── user/          # 用户应用代码 (控制算法 / 参数 / 配平 / OneNet 客户端 / 屏 UI / 跟踪)
├── app/           # 配套 App (uni-app, H5)
├── test/          # 离线单元测试
└── docs/          # 版本文档
```

## 编译与烧录

```bash
cd 0.40
pio run                 # 编译
pio run --target upload # 烧录
```

## 离线测试

```bash
cd ..                    # 工程根目录
scripts/run_tests.sh     # 默认跑 0.40
```

## 参数一致性

车端 `user/param/param.c` 与 App `app/common/paramDef.js` 的参数表必须逐项一致，
改默认值后必须同步两处并升 `PARAM_DATA_VERSION`:

```bash
python3 scripts/check_param_sync.py 0.40
```

## 版本号

- `version.txt`: 0.40.0
- App `versionName`: 0.40.0, `versionCode`: 150
- 参数表版本 `PARAM_DATA_VERSION`: 24
