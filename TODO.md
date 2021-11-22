- wbc
  - webots proto 加 joint limit(可能需要加 transform)
- feature
  - [ ] 自动档
  - [ ] fsm 状态切换增加鲁棒性
    - EStop 的处理
    - PreCheck 内容
    - PostCheck 内容
- 写死的常量分离出来
  - [ ] quadruped_impl.cc
  - 其他(调到哪里改到哪里)
- 确认各种参数是opt传入，还是常量，是常量的话，定义在哪里
  - 默认值可以 hardcode
- [CMake 超级构建](https://www.bookstack.cn/read/CMake-Cookbook/content-chapter10-10.4-chinese.md)


- fbmodel vector -> array
- drive_mode opt -> drive(类似gait，增加函数更改 drive_mode)
