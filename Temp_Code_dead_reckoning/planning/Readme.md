# 配置环境

## 安装依赖

```
sudo apt-get install -y libcurlpp-dev libblas-dev liblapack-dev gfortran autoconf automake libtool curl make g++ unzip git uuid-dev autoconf libssl-dev openssl libncurses5-dev libsqlite3-dev sqlite3 libtiff-dev libcurl4-openssl-dev python3-setuptools python3-dev libpython3.8-dev
```

## 安装cmake

```
wget "https://cmake.org/files/v3.12/cmake-3.12.4-Linux-x86_64.sh" -P /tmp/
sudo sh /tmp/cmake-3.12.4-Linux-x86_64.sh --prefix=/usr/local --skip-license
cmake --version
```




# 拉取代码库并构建

在远程开发机自己的用户空间下，建立~/ad文件夹，在ad下拉所有的代码库，包括cyebr、common、proto、third_party等
拉代码库的指令如下

```

git clone git@10.0.6.43:ad/planning.git

git clone git@10.0.6.43:ad/cyber.git

git clone git@10.0.6.43:ad/common.git

git clone git@10.0.6.43:ad/proto.git

git clone git@10.0.6.43:MDC_SOC/ap.git

git clone git@10.0.6.43:ad/control.git

git clone git@10.0.6.43:ad/prediction.git

git clone git@10.0.6.43:ad/map.git

git clone git@10.0.6.43:ad/third_party.git

git clone git@10.0.6.43:ad/canbus.git

git clone git@10.0.6.43:ad/cmake.git

git clone git@10.0.6.43:ad/dead_reckoning.git

git clone git@10.0.6.43:ad/dreamview.git

git clone git@10.0.6.43:ad/tools.git

git clone git@10.0.6.43:ad/mbd_control.git

git clone git@10.0.6.43:ad/parking_path.git

```

环境OK的话，执行planning下面的集成编译指令试试

```
bash pnc_build.sh release cyber
```

#构建顺序

```
    checkout_master cmake
    checkout_master cyber
    checkout_master ap
    checkout_master proto
    checkout_master common
    checkout_master map
    checkout_master control
    checkout_master planning
    checkout_master canbus
```