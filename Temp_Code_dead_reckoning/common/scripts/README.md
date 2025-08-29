># 百度云平台自定义评测
## 用户自定义修改评测参数：
**__注意：__**

为了防止混淆，除特殊说明，所有参数均使用 **__国际单位（SI）__**


打开 ./scripts/conf/metrics_conf.json文件：例如
```json
    {
    "version": "0.1",
    "last_update": "2022/11/28",
    "criterias": [
        {
            "name": "ParkCruiseBackwardSpeed",
            "expect_min_speed": -1.25 
        },
        {
            "name": "ParkCruiseForwardSpeed",
            "expect_max_speed": 1.25
        }
    ]
    }
```
expect_min_speed为最小泊车巡航后退速度 -1.25 * 3.6 = -4.5 km/h

expect_max_speed为最小泊车巡航后退速度 1.25 * 3.6 = 4.5 km/h


## 本地测试使用
cyber_py3的路径需要添加到PYTHONPYTH(例如 PYTHONPYTH = /home/simulation/Desktop/data/apollo/cyber，其下需要cyber_py3的python包),例如：
```bash
export PYTHONPATH=$PYTHONPATH:/home/simulation/Desktop/data/apollo/cyber
```
或使用规控团队提供的setup.bash文件

**__注意：__**
此cyber需要支持 **__百度云平台的cyber bag包，非合众record cyber包__**

 添加 --local_record + {record_name.record}，比如：
### 1. cd 到 ./common/scripts/
### 2. bash 或 python3 执行
#### bash
bash run_self_defined_metrics.sh

bash内如如下：

```bash
python3 apollo_reports_manager.py --reportconf reports_conf.json --local_record new_63762a7403d47d29599a86ed_apollo_0.record
```
#### python3
terminator中执行python3：

    python3 apollo_reports_manager.py --reportconf reports_conf.json --local_record new_63762a7403d47d29599a86ed_apollo_0.record
        
## 百度云平台仿真
默认平台会执行 run_self_defined_metrics.sh 
bash内如如下：
```bash
    python3 /apollo/cyber/scripts/apollo_reports_manager.py --reportconf reports_conf.json
```
># 合众自定义评测
## 用户自定义修改评测参数：
**__注意：__**

为了防止混淆，除特殊说明，所有参数均使用 **__国际单位（SI）__**


打开 ./scripts/conf/metrics_conf.json文件：例如
```json
    {
    "version": "0.1",
    "last_update": "2022/11/28",
    "criterias": [
        {
            "name": "NnpAccelerateLateral",
            "expect_Ax": [-0.5,0.5]  
        }
    ]
    }
```
expect_Ax为指标**NnpAccelerateLateral**可修改参数：表示为横向加速度范围[-0.5,0.5] m/^2



## 本地测试使用
cyber_py3的路径需要添加到PYTHONPYTH 
此cyber需要支持 **__合众record cyber包非百度云平台的cyber bag包__**,例如
```bash
source /home/simulation/Desktop/data/pnc/setup.bash
```

### 1. cd 到 ./common/scripts/
### 2.  执行 bash

```bash
python3 neta_reports_manager.py --reportconf nnp_reports_conf.json --scan_logs_folder /media/simulation/data/records/SIL/NNP/Cyber/2022-11-29-10-19-49 --log_reports_folder /media/simulation/data/records/SIL/NNP/Cyber/2022-11-29-10-19-49/records 
```
># Scripts使用说明

## 1.neta_reports_manager

**--reportconf**

json文件，用来对本次评测任务的参数配置，存放再/scripts/conf/文件夹下，用户可以定义自己的配置文件，例如：
```json
{
    "metrics_conf_file_name": "metrics_conf.json",
    "metrics": [
        "NnpAccelerateLateral"
    ]
}
```
metrics_conf_file_name:是当前使用的（每一个）指标的配置数据库,同样是json文件，例如metrics_conf.json

metrics：用来指示，当前任务参加评测的具体指标名称的列表，例如**NnpAccelerateLateral**（指标必须在/scripts/metrics/下已定义）。

**--scan_logs_folder**

用来扫描用户指定的**绝对路径**下的所有“record_name.record.[nums]"文件

**--log_reports_folder**

次路径用来存储指标生存的所有报告，如果当前路径不存在，系统会自动创建；如果创建失败，系统使用 scan_logs_folder路径，即如records文件路径相同。

## 2.metrics_manager

用来调试 **单个** 记录文件下的 **单个** neta/baidu 的 **metirc**. 执行，例如：

bash：
```bash
python3 metrics_manager.py --local_record /media/simulation/data/records/SIL/NNP/Cyber/2022-11-29-10-19-49/20221129100948.record.00000 --recordtype 0 --metric NnpAccelerateLateral --metrics_conf metrics_conf.json
```

**--local_record**

用户指定的**绝对路径**下的Cyber bag/record文件

**--recordtype**

Cyber bag/record文件类型，0 是合众record，1是百度云apollo record

**--metric**

指定调试的在/scripts/metrics/下定义的指标名称

**--metrics_conf**

指定/scripts/conf下，当前使用的（每一个）指标的配置数据库，例如metrics_conf.json
