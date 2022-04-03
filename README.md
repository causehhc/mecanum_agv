# mecanum_agv
## 1.创建工作空间并初始化
```bash
mkdir -p xxx_ws/src
cd xxx_ws
catkin_make
```
## 2.get
```bash
cd src
git clone https://github.com/causehhc/mecanum_agv.git
```
mv ./mecanum_agv/* .
mv ./mecanum_agv/.[^.]* .
rm -rf ./mecanum_agv
## 3.set
