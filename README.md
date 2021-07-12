指针用法规范约定：
1.需要多态的才用指针
2.指针用std智能指针
3.不可以 delete 智能指针

TODO
- del useless(or just for sim) code
- 优化性能
  - dynamic vector/matrix -> fixed vector/matrix
  - qp algorithm
- 调整架构
  - 核心逻辑独立成库，ros2调用独立库
  - 核心库export 头文件接口化
- 测试
- 仿真
