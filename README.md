```bash
mkdir build # build 폴더가 없을 경우

cd build
cmake ..
make

source ../env && ./bartender_robot
```

fatal error: i2c/smbus.h: No such file or directory 에러 발생 시

```bash
$ apt install libi2c-dev
```